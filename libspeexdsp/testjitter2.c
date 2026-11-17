/* Unit tests for the adaptive jitter buffer (jitter.c).

   These tests are fully deterministic: they drive the buffer with fixed
   timestamp sequences, disable the automatic delay adjustment (so tick()
   never changes state behind the tests' back) and use no randomness or
   wall-clock time. Covered behaviours:

   - timestamp wraparound (32-bit) and packet attribution across the wrap
   - out-of-order arrival (before and after the initial sync)
   - packets with different spans, gaps and non-zero start_offset
   - consecutive packet loss and resynchronisation after >20 losses
   - insertion compensation (buffering increase) and its boundary
   - drop decision (buffering decrease) through jitter_buffer_update_delay()
   - an already-consumed speech interval is never replayed
   - a late packet must never move the playback pointer backwards
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "speex/speex_jitter.h"
#include <stdio.h>

#define STEP 10          /* delay step / concealment size used everywhere */
#define DESIRED_SPAN 10  /* desired_span passed to jitter_buffer_get()   */

static int failed = 0;

#define CHECK(cond) do {                                        \
   if (!(cond)) {                                               \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
      failed++;                                                 \
   }                                                            \
} while (0)

static void make_packet(JitterBufferPacket *p, spx_uint32_t timestamp,
                        spx_uint32_t span, spx_uint16_t sequence)
{
   static char payload[4] = {1, 2, 3, 4};
   p->data = payload;
   p->len = sizeof(payload);
   p->timestamp = timestamp;
   p->span = span;
   p->sequence = sequence;
   p->user_data = 0;
}

static spx_uint32_t pointer_of(JitterBuffer *jb)
{
   return (spx_uint32_t)jitter_buffer_get_pointer_timestamp(jb);
}

static int get_packet(JitterBuffer *jb, JitterBufferPacket *out, spx_int32_t *start_offset)
{
   static char out_buf[64];
   out->data = out_buf;
   out->len = sizeof(out_buf);
   return jitter_buffer_get(jb, out, DESIRED_SPAN, start_offset);
}

static JitterBuffer *new_jb(void)
{
   JitterBuffer *jb = jitter_buffer_init(STEP);
   /* Calling update_delay() once disables the automatic adjustment in
      tick(); with no timings collected yet it is otherwise a no-op. */
   jitter_buffer_update_delay(jb, NULL, NULL);
   return jb;
}

/* In-order stream: baseline pointer progression. */
static void test_in_order_baseline(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int i, ret;

   for (i = 0; i < 3; i++) {
      make_packet(&in, i*STEP, STEP, i);
      jitter_buffer_put(jb, &in);
   }
   for (i = 0; i < 3; i++) {
      ret = get_packet(jb, &out, &off);
      CHECK(ret == JITTER_BUFFER_OK);
      CHECK(out.timestamp == (spx_uint32_t)(i*STEP));
      CHECK(out.span == STEP);
      CHECK(off == 0);
      CHECK(pointer_of(jb) == (spx_uint32_t)((i+1)*STEP));
      jitter_buffer_tick(jb);
   }
   jitter_buffer_destroy(jb);
}

/* Packets arriving out of order before the initial sync are reordered. */
static void test_out_of_order_before_sync(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret;

   make_packet(&in, 20, STEP, 2);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 0, STEP, 0);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 10, STEP, 1);
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0);
   CHECK(pointer_of(jb) == 10);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 10);
   CHECK(pointer_of(jb) == 20);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 20);
   CHECK(pointer_of(jb) == 30);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* Out-of-order arrival after the initial sync is reordered as well. */
static void test_out_of_order_after_sync(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret;

   make_packet(&in, 0, STEP, 0);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0);
   jitter_buffer_tick(jb);

   /* Packet with timestamp 20 arrives before the one with timestamp 10. */
   make_packet(&in, 20, STEP, 2);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 10, STEP, 1);
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 10);
   CHECK(pointer_of(jb) == 20);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 20);
   CHECK(pointer_of(jb) == 30);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* Packets with different spans move the pointer by their own span. */
static void test_varying_spans(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret;

   make_packet(&in, 0, 10, 0);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 10, 20, 1);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 30, 30, 2);
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0 && out.span == 10);
   CHECK(pointer_of(jb) == 10);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 10 && out.span == 20);
   CHECK(pointer_of(jb) == 30);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 30 && out.span == 30);
   CHECK(pointer_of(jb) == 60);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* A gap in the stream is skipped: the next packet inside the current chunk
   is returned with a positive start_offset. */
static void test_gap_and_start_offset(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret;

   make_packet(&in, 0, 10, 0);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 10, 20, 1);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 35, 5, 2);   /* gap: [30,35) is missing */
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 10);
   CHECK(pointer_of(jb) == 30);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 35 && out.span == 5);
   CHECK(off == 5);
   CHECK(pointer_of(jb) == 40);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* A hopelessly late packet (its whole interval was already played) must not
   be replayed and must not move the pointer backwards. A partially late
   packet only contributes its unplayed tail (negative start_offset). */
static void test_late_packet_no_replay_no_rewind(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret, i;

   for (i = 0; i < 3; i++) {
      make_packet(&in, i*STEP, STEP, i);
      jitter_buffer_put(jb, &in);
   }
   for (i = 0; i < 3; i++) {
      ret = get_packet(jb, &out, &off);
      CHECK(ret == JITTER_BUFFER_OK);
      jitter_buffer_tick(jb);
   }
   CHECK(pointer_of(jb) == 30);

   /* Duplicate of the already-consumed interval [0,10): dropped on arrival,
      the next get() must not return OK and the pointer must stay at 30. */
   make_packet(&in, 0, STEP, 0);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_INSERTION); /* deterministic: one late timing */
   CHECK(out.timestamp == 30);
   CHECK(pointer_of(jb) == 30);
   jitter_buffer_tick(jb);

   /* Interval [25,35) overlaps consumed media up to 30: only the tail is
      usable, signalled by the negative start_offset. */
   make_packet(&in, 25, STEP, 3);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK);
   CHECK(out.timestamp == 25);
   CHECK(off == -5);
   CHECK(pointer_of(jb) == 35);
   jitter_buffer_destroy(jb);
}

/* Two packets carrying the same timestamp (retransmit): the interval is
   played exactly once. */
static void test_duplicate_timestamp_not_replayed(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret;

   make_packet(&in, 0, STEP, 0);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 0, STEP, 1); /* duplicate of the same interval */
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0);
   CHECK(pointer_of(jb) == 10);
   jitter_buffer_tick(jb);

   /* The duplicate must not be returned again for the consumed interval. */
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_MISSING);
   CHECK(out.timestamp == 10);
   CHECK(pointer_of(jb) == 20);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* Timestamps wrap around the 32-bit boundary: packets just before and just
   after the wrap must be attributed to the right side of the boundary and
   played in stream order, even when they arrive out of order. */
static void test_timestamp_wraparound(void)
{
   const spx_uint32_t base = 0xFFFFFFF6u; /* 10 units before the wrap */
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret;

   /* Stream order is base, base+10 (==0), base+20 (==10); put them shuffled. */
   make_packet(&in, base + 20, STEP, 2);
   jitter_buffer_put(jb, &in);
   make_packet(&in, base, STEP, 0);
   jitter_buffer_put(jb, &in);
   make_packet(&in, base + 10, STEP, 1);
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == base);
   CHECK(pointer_of(jb) == 0u); /* wrapped */
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0u);
   CHECK(pointer_of(jb) == 10u);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 10u);
   CHECK(pointer_of(jb) == 20u);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* After the wrap, a late packet whose timestamp lies *before* the wrap must
   be recognised as old media (wrap-aware comparison): it is discarded and
   the post-wrap pointer is not rewound. */
static void test_wraparound_late_attribution(void)
{
   const spx_uint32_t base = 0xFFFFFFF6u;
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret;

   make_packet(&in, base, STEP, 0);
   jitter_buffer_put(jb, &in);
   make_packet(&in, base + 10, STEP, 1);
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == base);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0u);
   CHECK(pointer_of(jb) == 10u);
   jitter_buffer_tick(jb);

   /* base-10 = 0xFFFFFFEC lies before the wrap and was already consumed. */
   make_packet(&in, base - 10, STEP, 2);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_INSERTION); /* late timing forces buffering up */
   CHECK(pointer_of(jb) == 10u);          /* no rewind across the wrap */
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* Insertion compensation: with a positive margin, on-time packets record
   negative timings, so a missing packet triggers JITTER_BUFFER_INSERTION
   (buffering increase) and the pointer does not advance. Once the timings
   are shifted back to non-negative, the next miss is a plain
   JITTER_BUFFER_MISSING and the pointer advances by one concealment unit. */
static void test_insertion_compensation(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   spx_int32_t margin = 20;
   int ret;

   jitter_buffer_ctl(jb, JITTER_BUFFER_SET_MARGIN, &margin);

   make_packet(&in, 0, STEP, 0);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0);
   jitter_buffer_tick(jb);

   /* Both packets arrive while next_stop == 10, so their recorded timings
      are timestamp - arrival - margin = -20 and -10. */
   make_packet(&in, 10, STEP, 1);
   jitter_buffer_put(jb, &in);
   make_packet(&in, 20, STEP, 2);
   jitter_buffer_put(jb, &in);

   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 10);
   jitter_buffer_tick(jb);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 20);
   CHECK(pointer_of(jb) == 30);
   jitter_buffer_tick(jb);

   /* Buffer empty: the negative timings make the optimal delay -20, so a
      20-unit insertion is requested and the pointer stays at 30. */
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_INSERTION);
   CHECK(out.timestamp == 30);
   CHECK(out.span == 20);
   CHECK(pointer_of(jb) == 30);
   jitter_buffer_tick(jb);

   /* The insertion shifted the timings to [0, 10]: the compensation boundary
      is exactly reached, so this miss is a normal loss (span 10) and the
      pointer moves forward again. */
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_MISSING);
   CHECK(out.timestamp == 30);
   CHECK(out.span == 10);
   CHECK(pointer_of(jb) == 40);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* Drop decision: 50 packets buffered well ahead of their playout time record
   large positive timings, so jitter_buffer_update_delay() decides to drop
   90 timestamp units and the pointer jumps forward by that amount. */
static void test_drop_decision(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret, i;

   make_packet(&in, 0, STEP, 0);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0);
   jitter_buffer_tick(jb); /* next_stop == 10 */

   /* 50 packets, all arriving while next_stop == 10: their timings will be
      timestamp - 10, i.e. 90, 100, ..., 580. */
   for (i = 0; i < 50; i++) {
      make_packet(&in, 100 + i*STEP, STEP, i + 1);
      jitter_buffer_put(jb, &in);
   }

   /* The playout pointer first walks across the gap [10,100): 9 stable
      consecutive misses, each advancing the pointer by exactly one step. */
   for (i = 0; i < 9; i++) {
      ret = get_packet(jb, &out, &off);
      CHECK(ret == JITTER_BUFFER_MISSING);
      CHECK(pointer_of(jb) == (spx_uint32_t)(20 + i*STEP));
      jitter_buffer_tick(jb);
   }
   CHECK(pointer_of(jb) == 100);

   for (i = 0; i < 50; i++) {
      ret = get_packet(jb, &out, &off);
      CHECK(ret == JITTER_BUFFER_OK);
      CHECK(out.timestamp == (spx_uint32_t)(100 + i*STEP));
      jitter_buffer_tick(jb);
   }
   CHECK(pointer_of(jb) == 600);

   /* Smallest recorded timing is 90: drop 90 units of buffered delay. */
   ret = jitter_buffer_update_delay(jb, NULL, NULL);
   CHECK(ret == 90);
   CHECK(pointer_of(jb) == 690);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

/* Consecutive loss: every miss advances the pointer by exactly one
   concealment unit. After more than 20 consecutive losses the next put()
   resynchronises the buffer on the new stream. */
static void test_consecutive_loss_and_resync(void)
{
   JitterBuffer *jb = new_jb();
   JitterBufferPacket in, out;
   spx_int32_t off;
   int ret, i;

   make_packet(&in, 0, STEP, 0);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK && out.timestamp == 0);
   jitter_buffer_tick(jb);

   /* 21 consecutive misses: deterministic MISSING results, pointer moves
      forward by 10 each time, never backwards. */
   for (i = 0; i < 21; i++) {
      spx_uint32_t before = pointer_of(jb);
      ret = get_packet(jb, &out, &off);
      CHECK(ret == JITTER_BUFFER_MISSING);
      CHECK(out.timestamp == before);
      CHECK(out.span == STEP);
      CHECK(pointer_of(jb) == before + STEP);
      jitter_buffer_tick(jb);
   }
   CHECK(pointer_of(jb) == 220);

   /* lost_count > 20: this put() resets the buffer and resynchronises on
      the new stream position instead of the stale pointer. */
   make_packet(&in, 5000, STEP, 100);
   jitter_buffer_put(jb, &in);
   ret = get_packet(jb, &out, &off);
   CHECK(ret == JITTER_BUFFER_OK);
   CHECK(out.timestamp == 5000);
   CHECK(pointer_of(jb) == 5010);
   jitter_buffer_tick(jb);
   jitter_buffer_destroy(jb);
}

static void run_all_tests(void)
{
   test_in_order_baseline();
   test_out_of_order_before_sync();
   test_out_of_order_after_sync();
   test_varying_spans();
   test_gap_and_start_offset();
   test_late_packet_no_replay_no_rewind();
   test_duplicate_timestamp_not_replayed();
   test_timestamp_wraparound();
   test_wraparound_late_attribution();
   test_insertion_compensation();
   test_drop_decision();
   test_consecutive_loss_and_resync();
}

int main(void)
{
   /* Run the whole suite twice: the results must be identical and must not
      depend on runtime state left behind by a previous round. */
   run_all_tests();
   run_all_tests();

   if (failed) {
      printf("jitter buffer unit tests: %d failure(s)\n", failed);
      return 1;
   }
   printf("jitter buffer unit tests: all passed\n");
   return 0;
}
