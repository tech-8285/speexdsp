/* Copyright (C) 2026 Jean-Marc Valin

   File: testresample_unit.c
   Unit tests for the resampler state machine:

   1. Feeding the same stream in different input/output block sizes
      (including blocks that are too small to consume/produce anything)
      must yield identical output, and the per-call consumed/produced
      sample counts must follow the nominal time relation
      out = (in - latency) * out_rate / in_rate.
   2. Interleaved and deinterleaved (per-channel) processing of the same
      multi-channel stream must produce identical results, independent of
      the order in which the channels are processed (no inter-channel
      misalignment).
   3. Changing the sampling rate or the quality while the filter holds
      history samples must not lose or duplicate any of those samples:
      the output phase must advance according to the same time relation
      before and after the switch.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

   1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "speex/speex_resampler.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* In fixed-point builds the float API takes and returns samples in the
   +/-32767 range (see WORD2INT in arch.h), so scale the test signals. */
#ifdef FIXED_POINT
#define SAMPLE_SCALE 32767.f
#else
#define SAMPLE_SCALE 1.f
#endif

static int failures = 0;

#define CHECK_MSG(cond, ...) do { \
      if (!(cond)) { \
         fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
         fprintf(stderr, __VA_ARGS__); \
         fprintf(stderr, "\n"); \
         failures++; \
      } \
   } while (0)

/* Greatest common divisor, used to predict the reduced ratio. */
static spx_uint32_t test_gcd(spx_uint32_t a, spx_uint32_t b)
{
   while (b != 0)
   {
      spx_uint32_t t = a;
      a = b;
      b = t % b;
   }
   return a;
}

/* Test 1: the output stream must not depend on how the input is chopped
   into blocks, nor on how much output room is offered on each call. */
static void test_block_size_invariance(void)
{
   const spx_uint32_t in_rate = 48000;
   const spx_uint32_t out_rate = 32000;
   const int quality = 5;
   const int n_in = 8000;
   const int n_flush = 500;
   const int out_cap = 2*(n_in+n_flush) + 4096;
   static const int ichunk_pat[] = {1, 2, 3, 5, 7, 11, 13, 160, 333, 517, 1000, 64};
   static const int ocap_pat[] = {1, 2, 17, 4096, 5, 100, 1, 3000};
   float *input, *ref_out, *tst_out;
   SpeexResamplerState *st;
   int err, i;
   spx_uint32_t pos, ref_n, tst_n, cum_in, cum_out;
   int latency_in, iter;
   double ratio, max_diff;

   fprintf(stderr, "test_block_size_invariance... ");

   input = (float*)malloc(n_in*sizeof(float));
   ref_out = (float*)malloc(out_cap*sizeof(float));
   tst_out = (float*)malloc(out_cap*sizeof(float));
   for (i=0;i<n_in;i++)
      input[i] = SAMPLE_SCALE*(0.6f*sinf((float)(2*M_PI*i/37.0)) + 0.3f*sinf((float)(2*M_PI*i/111.0)));

   /* Reference run: fixed 160-sample input blocks, always enough output room. */
   st = speex_resampler_init(1, in_rate, out_rate, quality, &err);
   CHECK_MSG(st != NULL && err == RESAMPLER_ERR_SUCCESS, "init failed (err=%d)", err);
   speex_resampler_skip_zeros(st);
   pos = 0;
   ref_n = 0;
   while (pos < (spx_uint32_t)(n_in+n_flush))
   {
      spx_uint32_t il, ol;
      int remaining = n_in+n_flush-(int)pos;
      il = remaining > 160 ? 160 : (spx_uint32_t)remaining;
      ol = 4096;
      /* Once the real input is exhausted, flush the filter tail with zeros. */
      speex_resampler_process_float(st, 0, pos < (spx_uint32_t)n_in ? input+pos : NULL, &il, ref_out+ref_n, &ol);
      pos += il;
      ref_n += ol;
   }
   speex_resampler_destroy(st);

   /* Test run: irregular input blocks and irregular (sometimes tiny)
      output room. Every call must keep the consumed/produced counts on
      the nominal time relation, and the final stream must be identical. */
   st = speex_resampler_init(1, in_rate, out_rate, quality, &err);
   CHECK_MSG(st != NULL && err == RESAMPLER_ERR_SUCCESS, "init failed (err=%d)", err);
   speex_resampler_skip_zeros(st);
   latency_in = speex_resampler_get_input_latency(st);
   ratio = (double)out_rate/in_rate;
   pos = 0;
   cum_in = 0;
   cum_out = 0;
   iter = 0;
   while (pos < (spx_uint32_t)(n_in+n_flush))
   {
      spx_uint32_t il, ol;
      int remaining = n_in+n_flush-(int)pos;
      double expected;
      il = (spx_uint32_t)ichunk_pat[iter%(int)(sizeof(ichunk_pat)/sizeof(ichunk_pat[0]))];
      if ((spx_uint32_t)remaining < il)
         il = (spx_uint32_t)remaining;
      /* Never let a single call straddle the real-input/zero-flush boundary. */
      if (pos < (spx_uint32_t)n_in && il > (spx_uint32_t)n_in-pos)
         il = (spx_uint32_t)n_in-pos;
      ol = (spx_uint32_t)ocap_pat[iter%(int)(sizeof(ocap_pat)/sizeof(ocap_pat[0]))];
      speex_resampler_process_float(st, 0, pos < (spx_uint32_t)n_in ? input+pos : NULL, &il, tst_out+cum_out, &ol);
      pos += il;
      cum_in += il;
      cum_out += ol;
      /* The number of output samples must track the number of consumed
         input samples according to the filter delay and the rate ratio. */
      expected = ((double)cum_in - latency_in)*ratio;
      if (expected < 0)
         expected = 0;
      CHECK_MSG(fabs((double)cum_out - expected) <= 1.6,
                "count drift at call %d: consumed=%u produced=%u expected=%.2f",
                iter, cum_in, cum_out, expected);
      iter++;
      CHECK_MSG(iter < 1000000, "resampler made no progress (pos=%u)", pos);
      if (iter >= 1000000)
         break;
   }
   tst_n = cum_out;
   speex_resampler_destroy(st);

   CHECK_MSG(tst_n == ref_n, "output length differs: %u vs %u", tst_n, ref_n);
   max_diff = 0;
   for (i=0;i<(int)(tst_n < ref_n ? tst_n : ref_n);i++)
   {
      double d = fabs((double)tst_out[i]-ref_out[i]);
      if (d > max_diff)
         max_diff = d;
   }
   CHECK_MSG(max_diff <= 1e-5*SAMPLE_SCALE, "output samples differ by up to %g", max_diff/SAMPLE_SCALE);

   if (failures == 0)
      fprintf(stderr, "OK\n");
   free(input);
   free(ref_out);
   free(tst_out);
}

/* Test 2: interleaved processing must match per-channel (deinterleaved)
   processing exactly, whatever order the channels are handled in, and
   both must match independent single-channel resamplers. */
static void test_interleaved_vs_deinterleaved(void)
{
   const spx_uint32_t in_rate = 48000;
   const spx_uint32_t out_rate = 24000;
   const int quality = 4;
   const int n = 4096;
   const int out_cap = 2*n + 4096;
   static const int ichunk_pat[] = {321, 1, 160, 77};
   static const int ocap_pat[] = {100, 4096, 7, 500};
   float *ch[2], *inter_in, *inter_out, *deint_out[2], *mono_out[2];
   spx_uint32_t count[4];
   SpeexResamplerState *st_inter, *st_deint, *st_mono[2];
   int err, i, c, iter;
   spx_uint32_t pos;
   double max_diff;

   fprintf(stderr, "test_interleaved_vs_deinterleaved... ");

   inter_in = (float*)malloc(2*n*sizeof(float));
   inter_out = (float*)malloc(2*out_cap*sizeof(float));
   for (c=0;c<2;c++)
   {
      ch[c] = (float*)malloc(n*sizeof(float));
      deint_out[c] = (float*)malloc(out_cap*sizeof(float));
      mono_out[c] = (float*)malloc(out_cap*sizeof(float));
   }
   for (i=0;i<n;i++)
   {
      ch[0][i] = SAMPLE_SCALE*0.7f*sinf((float)(2*M_PI*i/48.0));
      ch[1][i] = SAMPLE_SCALE*0.5f*sinf((float)(2*M_PI*i/80.0) + 0.3f);
      inter_in[2*i] = ch[0][i];
      inter_in[2*i+1] = ch[1][i];
   }

   st_inter = speex_resampler_init(2, in_rate, out_rate, quality, &err);
   st_deint = speex_resampler_init(2, in_rate, out_rate, quality, &err);
   st_mono[0] = speex_resampler_init(1, in_rate, out_rate, quality, &err);
   st_mono[1] = speex_resampler_init(1, in_rate, out_rate, quality, &err);
   CHECK_MSG(st_inter && st_deint && st_mono[0] && st_mono[1], "init failed");
   speex_resampler_skip_zeros(st_inter);
   speex_resampler_skip_zeros(st_deint);
   speex_resampler_skip_zeros(st_mono[0]);
   speex_resampler_skip_zeros(st_mono[1]);

   for (c=0;c<4;c++)
      count[c] = 0;
   pos = 0;
   iter = 0;
   while (pos < (spx_uint32_t)n)
   {
      spx_uint32_t offer, cap;
      spx_uint32_t il_inter, ol_inter, il_ch[2], ol_ch[2], il_mono[2], ol_mono[2];
      offer = (spx_uint32_t)ichunk_pat[iter%4];
      if (offer > (spx_uint32_t)n-pos)
         offer = (spx_uint32_t)n-pos;
      cap = (spx_uint32_t)ocap_pat[iter%4];

      il_inter = offer;
      ol_inter = cap;
      speex_resampler_process_interleaved_float(st_inter, inter_in+2*pos, &il_inter, inter_out+2*count[0], &ol_inter);

      /* Deinterleaved, deliberately processing channel 1 before channel 0:
         the processing order must not shift one channel against the other. */
      il_ch[1] = offer;
      ol_ch[1] = cap;
      speex_resampler_process_float(st_deint, 1, ch[1]+pos, &il_ch[1], deint_out[1]+count[1], &ol_ch[1]);
      il_ch[0] = offer;
      ol_ch[0] = cap;
      speex_resampler_process_float(st_deint, 0, ch[0]+pos, &il_ch[0], deint_out[0]+count[0], &ol_ch[0]);

      for (c=0;c<2;c++)
      {
         il_mono[c] = offer;
         ol_mono[c] = cap;
         speex_resampler_process_float(st_mono[c], 0, ch[c]+pos, &il_mono[c], mono_out[c]+count[2+c], &ol_mono[c]);
      }

      /* Every channel of every state must consume and produce the same
         number of samples for the same call. */
      CHECK_MSG(il_inter == il_ch[0] && il_inter == il_ch[1] && il_inter == il_mono[0] && il_inter == il_mono[1],
                "consumed count mismatch at call %d: %u %u %u %u %u",
                iter, il_inter, il_ch[0], il_ch[1], il_mono[0], il_mono[1]);
      CHECK_MSG(ol_inter == ol_ch[0] && ol_inter == ol_ch[1] && ol_inter == ol_mono[0] && ol_inter == ol_mono[1],
                "produced count mismatch at call %d: %u %u %u %u %u",
                iter, ol_inter, ol_ch[0], ol_ch[1], ol_mono[0], ol_mono[1]);

      pos += il_inter;
      count[0] += ol_inter;
      count[1] += ol_inter;
      count[2] += ol_inter;
      count[3] += ol_inter;
      iter++;
      CHECK_MSG(iter < 1000000, "resampler made no progress (pos=%u)", pos);
      if (iter >= 1000000)
         break;
   }

   CHECK_MSG(count[0] == count[1] && count[0] == count[2] && count[0] == count[3],
             "total produced counts differ: %u %u %u %u", count[0], count[1], count[2], count[3]);

   max_diff = 0;
   for (i=0;i<(int)count[0];i++)
   {
      double d0, d1;
      d0 = fabs((double)inter_out[2*i]-deint_out[0][i]);
      d1 = fabs((double)inter_out[2*i]-mono_out[0][i]);
      if (d0 > max_diff) max_diff = d0;
      if (d1 > max_diff) max_diff = d1;
      d0 = fabs((double)inter_out[2*i+1]-deint_out[1][i]);
      d1 = fabs((double)inter_out[2*i+1]-mono_out[1][i]);
      if (d0 > max_diff) max_diff = d0;
      if (d1 > max_diff) max_diff = d1;
   }
   CHECK_MSG(max_diff <= 1e-5*SAMPLE_SCALE, "interleaved/deinterleaved outputs differ by up to %g", max_diff/SAMPLE_SCALE);

   if (failures == 0)
      fprintf(stderr, "OK\n");
   speex_resampler_destroy(st_inter);
   speex_resampler_destroy(st_deint);
   speex_resampler_destroy(st_mono[0]);
   speex_resampler_destroy(st_mono[1]);
   free(inter_in);
   free(inter_out);
   for (c=0;c<2;c++)
   {
      free(ch[c]);
      free(deint_out[c]);
      free(mono_out[c]);
   }
}

/* Test 3: switching rate or quality in the middle of a stream. The counts
   must stay on the time relation (allowing for the latency change), and
   once the filter-length transient has passed, the output must be the same
   sine with the phase predicted from the input timeline -- a lost or
   duplicated history sample would show up as a permanent phase jump. */
static void test_rate_quality_switch(void)
{
   const int block = 160;
   const int n_blocks = 40;
   const int seg_in = block*n_blocks; /* 6400 input samples per segment */
   const int n_seg = 9;
   /* Segment table: rate/quality applied at the start of each segment.
      Segment 1 keeps filt_len (pure ratio change), segments 2-6 exercise
      the filter grow/shrink ("magic samples") paths of update_filter(),
      and segments 7-8 use a non-integer ratio (4:3) so that the
      fractional phase is non-trivial when the rate is switched. */
   static const spx_uint32_t seg_in_rate[9]  = {48000, 48000, 48000, 48000, 48000, 48000, 48000, 48000, 48000};
   static const spx_uint32_t seg_out_rate[9] = {48000, 96000, 24000, 24000, 24000, 48000, 48000, 36000, 24000};
   static const int          seg_quality[9]  = {4, 4, 4, 10, 1, 1, 4, 4, 4};
   /* Extra input samples fed at the end of a segment, on top of the
      regular blocks. Segment 7 gets an odd-sized tail so that the
      fractional phase (den_rate=3) is non-zero when the rate is switched
      to 48k->24k: this exercises the samp_frac_num rescaling. */
   static const int          seg_extra[9]    = {0, 0, 0, 0, 0, 0, 0, 162, 0};
   const int out_cap = 4*seg_in*n_seg + 65536;
   const int in_cap = seg_in*n_seg + 1024;
   float *input, *out;
   spx_uint32_t seg_out_start[9];
   int seg_latency[9];
   double seg_t0[9];
   SpeexResamplerState *st;
   int err, s, b, ret;
   spx_uint32_t cum_in, cum_out, base_in, base_out, in_off;
   spx_uint32_t i;
   int lat_old, lat_off;
   double cs, cc, phi, amp, max_resid, max_resid_strict;

   fprintf(stderr, "test_rate_quality_switch... ");

   input = (float*)malloc(in_cap*sizeof(float));
   out = (float*)malloc(out_cap*sizeof(float));
   for (i=0;i<(spx_uint32_t)in_cap;i++)
      input[i] = SAMPLE_SCALE*0.8f*sinf((float)(2*M_PI*(double)i/64.0));

   st = speex_resampler_init(1, seg_in_rate[0], seg_out_rate[0], seg_quality[0], &err);
   CHECK_MSG(st != NULL && err == RESAMPLER_ERR_SUCCESS, "init failed (err=%d)", err);
   speex_resampler_skip_zeros(st);

   cum_in = 0;
   cum_out = 0;
   base_in = 0;
   base_out = 0;
   in_off = 0;
   lat_off = speex_resampler_get_input_latency(st);
   lat_old = lat_off;

   for (s=0;s<n_seg;s++)
   {
      double ratio = (double)seg_out_rate[s]/seg_in_rate[s];
      spx_uint32_t want_num, want_den, got_num, got_den, fact;
      spx_uint32_t got_in_rate, got_out_rate;
      int got_quality;

      if (s > 0)
      {
         ret = speex_resampler_set_rate(st, seg_in_rate[s], seg_out_rate[s]);
         CHECK_MSG(ret == RESAMPLER_ERR_SUCCESS, "set_rate segment %d returned %d", s, ret);
         ret = speex_resampler_set_quality(st, seg_quality[s]);
         CHECK_MSG(ret == RESAMPLER_ERR_SUCCESS, "set_quality segment %d returned %d", s, ret);
         /* The change of filter delay is absorbed by the memory
            reshuffling; the counts may only drift by that amount. */
         lat_off = speex_resampler_get_input_latency(st) - lat_old;
         base_in = cum_in;
         base_out = cum_out;
      }
      lat_old = speex_resampler_get_input_latency(st);
      seg_latency[s] = lat_old;
      seg_out_start[s] = cum_out;
      /* Input time of the first output sample of this segment, propagated
         from the actual output counts: consecutive output samples are
         always in_rate/out_rate input samples apart, whatever switches
         happened in between. */
      if (s == 0)
         seg_t0[0] = 0;
      else
         seg_t0[s] = seg_t0[s-1] + (double)(cum_out - seg_out_start[s-1])
                     *seg_in_rate[s-1]/seg_out_rate[s-1];

      /* The getters must report back what was just configured. */
      speex_resampler_get_rate(st, &got_in_rate, &got_out_rate);
      CHECK_MSG(got_in_rate == seg_in_rate[s] && got_out_rate == seg_out_rate[s],
                "get_rate mismatch in segment %d: %u/%u", s, got_in_rate, got_out_rate);
      speex_resampler_get_quality(st, &got_quality);
      CHECK_MSG(got_quality == seg_quality[s], "get_quality mismatch in segment %d: %d", s, got_quality);
      fact = test_gcd(seg_in_rate[s], seg_out_rate[s]);
      want_num = seg_in_rate[s]/fact;
      want_den = seg_out_rate[s]/fact;
      speex_resampler_get_ratio(st, &got_num, &got_den);
      CHECK_MSG(got_num == want_num && got_den == want_den,
                "get_ratio mismatch in segment %d: %u/%u vs %u/%u",
                s, got_num, got_den, want_num, want_den);

      for (b=0;b<n_blocks+1;b++)
      {
         spx_uint32_t il, ol = 4096;
         double expected;
         if (b < n_blocks)
            il = block;
         else if (seg_extra[s] > 0)
            il = (spx_uint32_t)seg_extra[s];
         else
            break;
         ret = speex_resampler_process_float(st, 0, input+in_off, &il, out+cum_out, &ol);
         CHECK_MSG(ret == RESAMPLER_ERR_SUCCESS, "process returned %d in segment %d", ret, s);
         cum_in += il;
         cum_out += ol;
         in_off += il;
         /* Time relation: produced = (consumed - delay) * ratio, where the
            delay offset is allowed to change by the latency difference at
            each switch. The count can obviously never go below what was
            already produced before the switch. */
         expected = (double)base_out + ((double)cum_in - base_in - lat_off)*ratio;
         if (expected < (double)base_out)
            expected = (double)base_out;
         CHECK_MSG(fabs((double)cum_out - expected) <= 2.0,
                   "count drift in segment %d block %d: consumed=%u produced=%u expected=%.2f",
                   s, b, cum_in, cum_out, expected);
      }
      /* All offered input must have been consumed by the end of the segment. */
      CHECK_MSG(cum_in == in_off,
                "segment %d left input unconsumed: %u vs %u", s, cum_in, in_off);
   }
   CHECK_MSG(cum_out < (spx_uint32_t)out_cap, "output buffer overflow");

   /* Output must be finite and bounded. */
   for (i=0;i<cum_out;i++)
      CHECK_MSG(out[i] == out[i] && fabs((double)out[i]) < 1.2*SAMPLE_SCALE,
                "output sample %u out of range: %f", i, out[i]);

   /* Fit amplitude and phase of the sine on the steady part of segment 0
      (1:1 rate, so output sample k corresponds to input time k). */
   cs = 0;
   cc = 0;
   for (i=200;i<6000;i++)
   {
      cs += out[i]*sin(2*M_PI*(double)i/64.0);
      cc += out[i]*cos(2*M_PI*(double)i/64.0);
   }
   phi = atan2(cc, cs);
   amp = 2.0/5800.0*sqrt(cs*cs+cc*cc);
   CHECK_MSG(amp > 0.7*SAMPLE_SCALE && amp < 0.9*SAMPLE_SCALE, "unexpected fitted amplitude %f", amp/SAMPLE_SCALE);

   /* For every segment, once the filter-length transient is over, each
      output sample must match the sine extrapolated from the input
      timeline. A lost or duplicated history sample at a switch would
      shift the phase and blow the residual far past the tolerance. */
   max_resid = 0;
   max_resid_strict = 0;
   for (s=1;s<n_seg;s++)
   {
      double ratio = (double)seg_out_rate[s]/seg_in_rate[s];
      spx_uint32_t seg_out_end = (s+1 < n_seg) ? seg_out_start[s+1] : cum_out;
      /* Skip the transient: filt_len input samples of history were
         zero-padded when the filter grew. */
      spx_uint32_t skip = (spx_uint32_t)((2*seg_latency[s]+64)*ratio) + 2;
      spx_uint32_t k;
      for (k=seg_out_start[s]+skip;k<seg_out_end;k++)
      {
         double t = seg_t0[s] + (double)(k-seg_out_start[s])*seg_in_rate[s]/seg_out_rate[s];
         double expected = amp*sin(2*M_PI*t/64.0 + phi);
         double resid = fabs((double)out[k]-expected);
         if (resid > max_resid)
            max_resid = resid;
         if (s == 1 && resid > max_resid_strict)
            max_resid_strict = resid;
      }
   }
   /* Segment 1 only changes the ratio (filt_len and sinc table are
      unchanged), so the phase must be essentially exact there. */
   CHECK_MSG(max_resid_strict <= 0.02*SAMPLE_SCALE, "phase jump after pure ratio switch: residual %g", max_resid_strict/SAMPLE_SCALE);
   CHECK_MSG(max_resid <= 0.08*SAMPLE_SCALE, "phase discontinuity after rate/quality switch: residual %g", max_resid/SAMPLE_SCALE);

   if (failures == 0)
      fprintf(stderr, "OK (max phase residual %g)\n", max_resid/SAMPLE_SCALE);
   speex_resampler_destroy(st);
   free(input);
   free(out);
}

/* Test 4: input and output latency getters must describe the same delay
   on both sides of the conversion. */
static void test_latency_relation(void)
{
   static const spx_uint32_t rates[5][2] = {
      {48000, 48000}, {48000, 96000}, {48000, 24000}, {48000, 44100}, {44100, 48000}
   };
   int i, err;

   fprintf(stderr, "test_latency_relation... ");
   for (i=0;i<5;i++)
   {
      SpeexResamplerState *st = speex_resampler_init(1, rates[i][0], rates[i][1], 4, &err);
      int lat_in, lat_out;
      double expect;
      CHECK_MSG(st != NULL && err == RESAMPLER_ERR_SUCCESS, "init failed (err=%d)", err);
      lat_in = speex_resampler_get_input_latency(st);
      lat_out = speex_resampler_get_output_latency(st);
      expect = (double)lat_in*rates[i][1]/rates[i][0];
      CHECK_MSG(fabs(lat_out - expect) <= 1.0,
                "latency mismatch %u->%u: in=%d out=%d expected=%.2f",
                rates[i][0], rates[i][1], lat_in, lat_out, expect);
      speex_resampler_destroy(st);
   }
   if (failures == 0)
      fprintf(stderr, "OK\n");
}

int main()
{
   test_block_size_invariance();
   test_interleaved_vs_deinterleaved();
   test_rate_quality_switch();
   test_latency_relation();
   if (failures)
   {
      fprintf(stderr, "%d check(s) FAILED\n", failures);
      return 1;
   }
   fprintf(stderr, "all resampler unit tests PASSED\n");
   return 0;
}
