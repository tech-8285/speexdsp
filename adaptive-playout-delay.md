# SpeexDSP 自适应抖动缓冲：动态延迟下的播放连续性分析

分析对象：`libspeexdsp/jitter.c`（839 行）与公共契约 `include/speex/speex_jitter.h`。
参考用法：`libspeexdsp/testjitter.c`、`doc/manual.lyx`（"Jitter Buffer" / "Adaptive Jitter Buffer" 两节）。
本仓库内除 `testjitter.c` 外没有其他调用者，因此"调用者契约"只能依据头文件、手册和代码本身推断，文中明确区分**代码可判定的事实**与**无法从代码推断的行为**。

---

## 0. 结论速览（播放连续性边界的判定口径）

抖动缓冲只维护一条 **RTP 时间戳播放轴**（`pointer_timestamp`），它保证的是**时间戳层面的无缝**，不保证声学层面的无缝。在两次成功同步之间（即不在 reset 重同步点上），对任意连续两次 `jitter_buffer_get()` 调用，缓冲区保证：

1. 每次返回都带有 `timestamp` 与 `span`，且播放轴按本次返回内容严格推进（真实包推进 `span`，补偿包推进补偿量）；
2. 时间轴上不存在"既没有真实包、也没有补偿占位"的空洞——取不到真实包时必然返回 `JITTER_BUFFER_INSERTION` 或 `JITTER_BUFFER_MISSING` 之一来覆盖该区间；
3. 延迟增长通过"时间轴回退 + 插入占位"完成，返回时间戳序列保持单调不减、区间首尾相接（表现为重复区间，而非空洞）；
4. 延迟收缩通过"时间轴前跳、丢弃尚未播放的旧包"完成，会产生**有意的内容跳变**，这是契约允许的唯一一类时间戳不连续；
5. 抖动缓冲不持有任何墙钟（wall clock），不执行 PLC 解码，不知道也不判断包是否真的在网络上丢失——声学连续性、调用节奏、线程互斥全部是调用者责任。

可判定的边界条件与破坏条件见第 6 节。

---

## 1. 组件与责任边界：时序统计、延迟建议、播放推进

### 1.1 三个职责层的划分

| 职责层 | 所属代码 | 拥有的状态 | 产出 |
|---|---|---|---|
| 时序统计层 | `struct TimingBuffer`、`tb_init/tb_add`（jitter.c:81–131）、`update_timings`（333–351）、`shift_timings`（354–362） | `_tb[3]` 三个子窗口、每窗最多 40 条排序后的 `timing` | 一份带窗口老化的"相对到达时刻"直方图 |
| 延迟建议层 | `compute_opt_delay`（173–266） | 只读统计层 + `latency_tradeoff/auto_tradeoff/delay_step/window_size` | 一个 `spx_int16_t` 建议值 `opt`，**仅为建议，本身不改任何播放状态** |
| 建议执行 + 播放推进层 | `_jitter_buffer_update_delay`（712–732）、`jitter_buffer_get`（463–674）、`jitter_buffer_tick`（750–764）、`jitter_buffer_remaining_span`（766–775）、`jitter_buffer_put`（366–460） | `pointer_timestamp`、`next_stop`、`buffered`、`interp_requested`、`lost_count`、包数组 | 实际的时间轴移动、包返回/丢弃、补偿占位 |

关键边界：**`compute_opt_delay()` 是纯函数式建议**。它只读取 `TimingBuffer` 和配置，不触碰 `pointer_timestamp`、不移动包、不设置 `interp_requested`。真正改变播放契约的动作全部发生在 `_jitter_buffer_update_delay()` 和 `jitter_buffer_get()` 内。

### 1.2 建议值在什么时机被执行

建议值有两条执行路径，互斥由 `auto_adjust` 控制：

- **自动模式（默认，`auto_adjust=1`，jitter.c:286）**：每次 `jitter_buffer_tick()`（753–754）或 `jitter_buffer_remaining_span()`（769–770）内部调用 `_jitter_buffer_update_delay()`。手册要求 tick 是播放线程睡眠前的最后一个抖动缓冲调用。
- **手动模式**：调用者显式调用公共 `jitter_buffer_update_delay()`（735–742），该函数**第一件事就是把 `auto_adjust` 置 0**，此后 tick 不再自动调整。注意其 `packet` 与 `start_offset` 形参完全未被使用（712 行的内部函数同样忽略它们）。

此外，`jitter_buffer_get()` 在找不到任何可返回内容时，也会直接调用一次 `compute_opt_delay()`（640 行），用于在"普通丢包 concealment"与"被迫增加缓冲"之间做分支决策——这是建议层唯一一处嵌入播放推进层的旁路。

### 1.3 播放推进由谁驱动

抖动缓冲**没有内部定时器**（手册原文："designed not to use an explicit timer"）。播放推进的唯一驱动是调用者周期性调用 `get()` + `tick()`：

- `get()` 按 `desired_span` 消费播放轴上的下一段；
- `tick()` 依据上一次 `get()` 记账的 `buffered` 推算下一次取包的名义时刻 `next_stop`（756–763）。

调用者不调用 tick，`next_stop` 就不前进，迟到判定与统计全部停在旧时刻。这是调用者节奏，库无法自检。

---

## 2. 时序统计链：一个 timing 数字如何产生、老化、参与决策

### 2.1 timing 的定义与符号约定

timing 一律按"包时间戳 − 到达基准 − margin"计算，是 `spx_int32_t`，入窗时被夹到 `[-32767, 32767]`（jitter.c:335–338）：

- **迟到包路径**（`put`，392–395）：当 `LT32(packet->timestamp, next_stop)` 时，
  `timing = packet->timestamp - next_stop - buffer_margin`，结果为负，且该包的 `arrival[i]` 记为 0（453–454）。
- **正常取回路径**（`get`，584–587）：仅当 `arrival[i] != 0` 时，
  `timing = packet->timestamp - arrival[i] - buffer_margin`。包在其播放时刻之后才被消费时同样为负。

因此：**timing < 0 表示"晚到"，timing ≥ 0 表示"提前量"**。`buffer_margin`（默认 0，可经 `JITTER_BUFFER_SET_MARGIN` 设置）整体平移直方图，等价于要求额外的保底缓冲。

`arrival[]` 的语义有特殊约定（结构体注释 jitter.c:144）：0 表示该包是迟到包或 reset 后到达的包，同时 0 被刻意当作"无效到达时刻"，使这类包不在 `get()` 里二次统计——它们在 `put()` 时已经统计过一次。

### 2.2 排序、容量与窗口老化

- `tb_add`（95–131）维护**升序排序**（最负/最晚的在 `timing[0]`，结构体注释"latest first"即此义）。窗口满 40 条后，新值若比最旧值还"早"（`timing >= timing[filled-1]`），只递增 `curr_count` 后丢弃（99–103），即直方图只保留最极端的 40 个样本，但 `curr_count` 仍计入总观测数。
- `update_timings`（340–349）：当前子窗口计数达到 `subwindow_size` 时轮转，3 个 `TimingBuffer` 整体移位、最旧子窗清空复用。
- 窗口尺寸来自 `JITTER_BUFFER_SET_MAX_LATE_RATE`（819–823）：
  `window_size = 100*40/max_late_rate`，默认 `max_late_rate=4`（jitter.c:287–288）→ `window_size=1000`，`subwindow_size≈333`。即统计视野约为最近 1000 个包到达事件，分 3 个子窗老化。

### 2.3 决策函数 compute_opt_delay 的可判定语义（173–266）

**成本模型**（头注释 167–172 与代码 234）：

```
cost(d) = d + late_factor * (在延迟 d 下会变成迟到的帧数)
```

代码里候选 `d` 取观测到的 timing 值本身（`latest = ROUND_DOWN(timing, delay_step)`，230 行），`-latest` 即增加的缓冲量。

- `late_factor`：设置了 `latency_tradeoff`（`JITTER_BUFFER_SET_LATE_COST`）时为 `latency_tradeoff*100/tot_count`（198–199）；否则用自动值 `auto_tradeoff * window_size/tot_count`（201）。
- `auto_tradeoff` 每次计算后自适应更新为 `1 + (best-worst)/40`（257 行，初值 32000，reset 时恢复），即按观测到的到达时间散布自动标定"1% 迟到率值多少时间戳单位"。
- 只评估最"晚"的 `TOP_DELAY=40` 个样本（209–253），在三窗之间做归并选取。
- **迟滞（hysteresis）**：第一个非负候选出现时一次性给 `late` 加 4（248–252），使"维持现状/增加缓冲"相对"收缩缓冲"有 4 个迟到帧的成本优势，抑制增长↔收缩振荡。
- **冷启动保护**：总观测数 `tot_count < 40` 且算出 `opt>0`（建议收缩）时，强制返回 0（263–264）。样本不足时只允许加缓冲，不允许减缓冲。

**返回值符号（全库统一约定）**：

| `opt` 符号 | 含义 | 量化 |
|---|---|---|
| `opt < 0` | 需要**增长**缓冲，增长量 `-opt` | `ROUND_DOWN` 到 `delay_step` 的整数倍（74 行宏，对负数向负无穷取整） |
| `opt = 0` | 维持现状 | — |
| `opt > 0` | 建议**收缩**缓冲，收缩量 `opt` | 同上，正向量化为步长倍数 |

### 2.4 统计补偿：shift_timings 为何与时间轴移动配对

每次执行调整时都先调用 `shift_timings(jitter, -opt)`（719、726；get 旁路 648）：把所有历史 timing 加上 `-opt`。

- `opt<0`（加缓冲 `-opt` 量）：所有 timing 加上正的增长量——基准时间轴回退后，旧观测在新基准下显得"更早"，直方图不因自身的加缓冲动作而持续读出负 opt；
- `opt>0`（收缩 `opt` 量）：所有 timing 减去收缩量。

这保证**统计输入在缓冲调整前后保持不变量**：调整动作本身不会污染下一轮决策，这是动态延迟能稳定收敛而不自激的关键机制。

---

## 3. 建议值 vs 调用者实际推进：四类动作的精确语义

### 3.1 `_jitter_buffer_update_delay`（712–732）：建议如何落到时间轴

```c
if (opt < 0) {                          // 增长缓冲
    shift_timings(jitter, -opt);
    jitter->pointer_timestamp += opt;   // 时间轴回退 -opt（无符号回绕运算）
    jitter->interp_requested = -opt;    // 登记一笔"递延插入"
} else if (opt > 0) {                   // 收缩缓冲
    shift_timings(jitter, -opt);
    jitter->pointer_timestamp += opt;   // 时间轴前跳 opt
}
```

- **增长（opt<0）不立即吐包**：只回退 `pointer_timestamp` 并登记 `interp_requested`。下一次 `get()` 开头（501–516）优先兑现该递延插入：返回 `JITTER_BUFFER_INSERTION`、`len=0`、`span=interp_requested`，再把指针加回同样的量。净效果是播放轴在时间戳上**后退再前进，覆盖一段重复区间**，时间戳序列首尾相接、无空洞。
- **收缩（opt>0）没有补偿包**：指针直接前跳。前跳越过的、尚未播放的包会在后续 `put()` 的清理循环（378–387，条件 `timestamp+span <= pointer_timestamp`）或 `get()` 取回时被销毁/跳过。这是**主动丢数据**，语义上等价于加速播放。
- `opt=0`：零副作用。

### 3.2 `jitter_buffer_get` 的五条返回路径及其时间轴语义

| 路径 | 返回码 | 时间轴动作 | 音频责任 |
|---|---|---|---|
| reset 后首个包（473–496） | `OK` | 同步到最老包，`pointer_timestamp = next_stop = oldest` | 调用者直接播 |
| 递延插入兑现（501–516） | `INSERTION` | 指针 += `interp_requested`，`len=0` | 调用者必须自行 PLC |
| 命中真实包，四级匹配（521–629） | `OK` | 指针 = 包 `timestamp+span`；可能经 `start_offset` 报告偏移 | 调用者按 offset 对齐播放 |
| 无包且 `opt<0`（640–657） | `INSERTION` | **指针不动**，返回 `span=-opt` 的空包，同时已 shift 统计 | 调用者 PLC；下次 get 仍在同一时间点重试真实包 |
| 无包且 `opt≥0`（659–670） | `MISSING` | 指针 += `ROUND_DOWN(desired_span, concealment_size)` | 调用者 PLC，时间轴按 concealment 粒度推进 |

真实包的四级匹配是递进放宽的（521–573）：①时间戳精确且覆盖整段 → ②更老但完整覆盖 → ③更老且部分覆盖 → ④起点落在当前 chunk 内的最早包。找不到才进入丢包分支。这意味着**迟到但仍与当前播放区间有交集的包会被降级使用而不是直接判死**，offset 通过 `start_offset` 输出（609–613）；调用者传 `NULL` 时非零 offset 只会触发 warning 并被丢弃。

### 3.3 补偿（INSERTION）与丢包判定（MISSING）的语义差别

两者都返回 `len=0`，都要求调用者做 PLC，但契约不同：

- **INSERTION 是缓冲策略动作**：`get()` 旁路（643–657）表示"统计认为缓冲不足，此刻不推进时间轴，用占位换取等待迟到包的机会"；update_delay 路径（501–516）表示"上一轮已决定增长缓冲"。两种情况下播放轴都不产生空洞。
- **MISSING 是消费动作**：统计认为当前延迟已足够（`opt≥0`），不再等待，时间轴按 `concealment_size` 粒度前进。注意 `desired_span` 会被 `ROUND_DOWN` 取整——若取整结果为 0，指针不前进，调用者若原样重试可能空转，这一行为库内不防护。
- **库不区分"真丢包"和"迟到未到"**：`get()` 时刻只看包里有没有可匹配的内容。`sequence` 字段仅存取与原样回传（451、619、700 行），**完全不参与任何丢失/乱序判定**。真正的网络丢包检测是调用者（RTP 层）的事。

### 3.4 丢弃与迟到判定的三个独立闸门

1. **迟到统计闸门**（put，392）：`LT32(packet->timestamp, next_stop)` → 记一条负 timing，标 `arrival=0`。包**仍会入缓冲**。
2. **彻底无用丢弃闸门**（put，408）：仅当 reset 状态，或 `GE32(packet->timestamp+packet->span+delay_step, pointer_timestamp)` 才入缓冲。即包的覆盖区间末端比当前播放指针还早超过一个 `delay_step` 时，静默丢弃（无返回值、无计数）。
3. **播放后清理闸门**（put，375–387）：每次 put 时清理所有 `timestamp+span <= pointer_timestamp` 的未播放旧包。注释明确指出用 `<=` 而非更严格条件，是为了保留"刚刚迟到"的包供下一轮插值后使用。
4. **容量溢出**（put，419–437）：固定 200 槽（`SPEEX_JITTER_MAX_BUFFER_SIZE`），满时静默淘汰时间戳最早的包，无反馈。
5. **失联重同步**（put，402–405）：连续 `lost_count>20` 时下一次 put 直接 `jitter_buffer_reset()`，时间轴作废、在新包上重新对齐。`lost_count` 在每次成功取到真实包时清零（581），每次 miss 递增（636）。

补充：结构体中的 `late_cutoff` 字段（152 行，初值 50，283 行赋值）**全库再无读取点，是死字段**。"多晚才算彻底无用"实际由闸门 2 的 `delay_step` 决定，不能由 `late_cutoff` 推断。

### 3.5 32 位回绕安全与时间戳契约

所有比较经 `GT32/GE32/LT32/LE32`（69–72）以 `spx_int32_t` 差值实现，`TSUB` 同理——时间戳按无符号 32 位自然回绕处理，只要单个比较的两端相差不超过 2³¹，回绕点附近判定仍然正确。`pointer_timestamp` 本身是 `spx_uint32_t`，加负数（回退）在回绕运算下定义明确。

---

## 4. 抖动、丢包、迟到包并存时，延迟增长/收缩对连续语音区间的影响

定义**连续语音区间**：从一次成功同步（reset 后首个 `OK`，或 `lost_count>20` 触发的重同步）起，到下一次重同步止，时间戳轴上由真实包与 PLC 占位首尾拼接而成的播放区间。以下三个场景均可由代码路径直接判定。

### 场景 A：突发抖动 + 部分丢包 → 延迟增长（opt<0）

1. 若干包晚于 `next_stop` 到达：put 记负 timing、标 `arrival=0` 但保留入缓冲；
2. 消费者按节奏 `get()`，目标时间点无包 → `lost_count++`，调 `compute_opt_delay`；
3. 负 timing 样本在成本函数中占优且越过迟滞阈值 → `opt<0`：返回 `INSERTION`、`span=-opt`、**指针不动**，直方图同步 shift；
4. 调用者 PLC 覆盖该区间；下一轮 tick 后 `get()` 在**同一时间点**重试：
   - 迟到包在此期间到达且与当前区间有交集 → 四级匹配兜底命中，返回 `OK` 与 `start_offset`，`lost_count` 清零，语音区间无缝衔接到真实数据；
   - 仍未到 → 继续 INSERTION 或转入 MISSING 推进。
5. 若调整发生在 tick 内（update_delay 路径），则表现为先回退指针、下一次 get 兑现递延插入——时间戳序列出现一段**重复覆盖区**而非空洞。

**对连续性的净效果**：时间戳连续性保持；声学连续性取决于调用者 PLC 质量，库不保证。增长是分步长进行的，且迟滞机制保证不会因单个迟到包就频繁加延迟。

### 场景 B：真丢包与迟到无法区分 → MISSING 推进

当统计认为缓冲已足够（`opt≥0`）时，缺包一律走 MISSING：指针按 `ROUND_DOWN(desired_span, concealment_size)` 前进。若包在指针越过之后才到：

- 其末端仍在指针前方一个 `delay_step` 内 → 入缓冲，靠第 2/3/4 级匹配在后续 chunk 中被部分利用（可能带非零 `start_offset`）；
- 否则被闸门 2 静默丢弃，只在 put 时留下一条负 timing 供后续决策。

即**迟到包对当前语音区间的补救窗口 = 播放指针到 `pointer_timestamp - delay_step` 之间**；超出该窗口的迟到包只影响统计、不再影响播放。连续 MISSING 超过 20 次且期间无任何真实包命中，下一次 put 触发 reset，当前连续语音区间终止。

### 场景 C：网络恢复、包持续早到 → 延迟收缩（opt>0）

1. 直方图中正 timing 占主导，成本函数在冷启动保护（样本≥40）与迟滞惩罚之外找到正 opt；
2. update_delay 执行：`pointer_timestamp += opt`，shift 统计，**无占位包**；
3. 被指针跳过的旧包在下一次 put 清理或 get 时销毁；
4. 此后 `get()` 取回的是新指针位置的包，播放内容发生一次**有意跳变**（跳过 opt 量媒体），但后续时间轴重新首尾相接。

**对连续性的净效果**：收缩是唯一一类库主动制造的时间戳不连续，目的是把已增长的延迟还回去。因为有迟滞（+4 帧惩罚）和冷启动保护，收缩方向的调整显著比增长保守；testjitter 的 "frozen sender" 用例验证的是另一种情形——发送方长时间冻结后，靠 `lost_count>20` reset 在新流上重同步，而不是靠收缩追赶。

### 场景 D：三种损伤同时存在时的优先级

一次 `get()` 无包时的决策顺序是固定的：先查 `interp_requested`（递延增长优先兑现）→ 四级真实包匹配（迟到包兜底）→ `compute_opt_delay` 决定 INSERTION（继续等）还是 MISSING（放弃等）。因此同一时刻三类损伤并存时：**已承诺的增长优先于新决策，任何可用真实包优先于补偿，统计建议只在两者都落空时生效**。这条优先级保证了状态变化（建议值翻转）不会打断尚未兑现的播放契约。

---

## 5. 状态变化时仍然成立的契约，与不能从代码推断的行为

### 5.1 任何状态变化下都保持的时间戳/播放契约

1. **播放轴单调不减（回绕运算意义下）**：真实包推进 `span`；MISSING 推进非负的取整量；INSERTION 要么不动指针（get 旁路），要么回退后由紧随的递延插入等量加回（update_delay 路径），对外返回的时间戳序列不会倒退。
2. **无空洞契约**：每个名义播放区间都由 OK / INSERTION / MISSING 之一覆盖，不存在"无返回内容又不推进指针"的路径（`ROUND_DOWN` 取整为 0 的边界除外，见 5.2）。
3. **每次返回自带坐标**：`packet.timestamp + span` 与 `start_offset` 足以让调用者独立完成时间对齐；调用者无需理解内部缓冲结构。
4. **调整与统计解耦的不变量**：`shift_timings` 保证缓冲增减不污染后续 `compute_opt_delay` 输入，建议值翻转不会自激。
5. **回绕安全**：全部时间戳比较在 2³¹ 差值内对 32 位回绕成立。
6. **重同步后的新基线明确**：reset 后第一个可用包令 `pointer_timestamp=next_stop=oldest`，新连续区间从该包时间戳起算，旧包全部释放，状态可追溯。
7. **`buffered` 记账契约**：每次 `get()` 的三条出口都把 `buffered` 置为 `packet.span - desired_span (+start_offset)`（513、624–627、655、668），tick 据此把"应用层仍持有的未播放数据"折算进 `next_stop`；`remaining_span` 允许调用者直接上报该余量。

### 5.2 不能从本库推断、必须由调用者负责或外部保证的行为

1. **声学连续性不可推断**：INSERTION/MISSING 都只给 `len=0` 的空包，PLC 算法、补偿时长的听感、重复区间（增长时）的时长拉伸处理完全在库外。
2. **调用节奏不可推断**：库无墙钟、无内部线程。tick/remaining_span 的调用周期、get 的 `desired_span` 是否与声卡帧长一致，决定 `next_stop` 是否反映真实时间；漏调 tick 会使迟到判定停摆，库不报错（源码 TODO 列出的 "warn if update_delay not called between get() and tick()" 尚未实现，jitter.c:40）。
3. **真丢包 vs 极端迟到不可推断**：判定只基于缓冲内有无匹配包；`sequence` 不参与逻辑。RTP 序列号丢包检测是调用者职责。
4. **手动/自动模式的混用后果无防护**：一旦显式调用公共 `jitter_buffer_update_delay()`，`auto_adjust` 永久关闭（无重新开启的 ctl）；若在递延插入尚未兑现时再次调整，`interp_requested` 会被新值覆盖，库不检查这种调用序列。
5. **线程安全不可推断**：手册明确要求 put/get 跨线程时调用者加互斥锁；库内无锁。
6. **丢弃不可观测**：彻底迟到丢弃（408）、容量淘汰（419–437）、播放后清理（378–387）均无返回码、无计数器。应用层无法从 API 获得丢包/迟到统计（源码 TODO 的 Statistics 一节未实现，45–51）。
7. **时钟漂移不建模**：统计只反映包时间戳与到达基准的相对差，发送/接收采样率漂移要靠调用者用重采样器（同库 `resample.c`）修正；`get_another`（676–709）只解决同时间戳的分片媒体，不处理漂移。
8. **配置有效性不校验**：`max_late_rate` 设为 0 会在 ctl 中触发除零（821–822）；`desired_span` 小于 `concealment_size` 时 MISSING 的取整 span 为 0，可能造成调用者侧空转；这些都依赖调用者约束。
9. **`late_cutoff` 不可作为行为依据**：该字段为死字段，任何基于它推断迟到阈值的分析都不成立。
10. **缓冲上限的时延含义不可推断**：200 包上限对应的实际毫秒延迟取决于包 span，库不提供时延查询接口；只有 `JITTER_BUFFER_GET_AVAILABLE_COUNT`（790–799）给出当前就绪包数。

---

## 6. 可判定的播放连续性边界（最终判定口径）

设调用者满足前置条件：① 按播放周期严格调用 `get()` 且每个播放周期后调用一次 `tick()` 或 `remaining_span()`；② 对 INSERTION/MISSING 返回执行等价时长的 PLC；③ 跨线程访问加锁；④ 时间戳差值在 2³¹ 内。则以下命题可由代码直接判定：

### 6.1 保证成立的边界

- **B1（时间轴无缝）**：同一连续语音区间内，相邻两次 `get()` 返回的 `[timestamp, timestamp+span)` 区间首尾相接；INSERTION 路径产生的是重叠/重复区间，MISSING 路径产生的是等长占位，均无未覆盖空洞。
- **B2（单调性）**：对外返回的 `timestamp` 单调不减；唯一的指针回退发生在 update_delay 内部，且由下一次 get 的递延插入对冲，不暴露为倒退的返回时间戳。
- **B3（迟到包补救窗口）**：迟到真实包只要满足 `timestamp+span+delay_step >= pointer_timestamp`（put 闸门，jitter.c:408）且与某待取 chunk 有交集，就仍可能经四级匹配返回 `OK`；补救窗口的宽度显式等于一个 `delay_step` 加上当前 chunk 的匹配容差。
- **B4（调整有界且量化）**：每次延迟增减量都是 `delay_step` 的整数倍；单步增长不超过直方图中观测到的最坏晚到量（候选仅取自实际 timing，jitter.c:229–230）；收缩在样本数 <40 时被禁止（263–264），且恒承担 +4 帧迟滞成本。
- **B5（故障封闭）**：连续取不到真实包超过 20 次后，下一次 put 必然 reset 重同步（402–405），失控状态不会无限延续；reset 点是契约内唯一显式的区间断裂点，且断裂后新基线由到达包的时间戳确定性给出。
- **B6（建议与推进解耦）**：`compute_opt_delay` 的任何输出在被 `_jitter_buffer_update_delay` 或 get 旁路执行前不改变播放契约；已登记的 `interp_requested` 在 get 入口优先兑现，不受新一轮建议翻转影响。

### 6.2 连续性被破坏或不再被保证的判定条件

- **D1（有意跳变）**：`opt>0` 收缩一旦执行，`pointer_timestamp` 前跳 opt，被跳过的媒体不可恢复——时间戳连续性在此刻断裂，这是策略允许的断裂而非缺陷。
- **D2（重同步断裂）**：reset（含失联 20 次触发、主动 reset、销毁重建）之后，新旧连续区间之间不存在任何时间戳衔接承诺。
- **D3（库外失效）**：调用者未对 INSERTION/MISSING 做 PLC、漏调 tick、`desired_span` 与设备周期失配、或跨线程未加锁时，B1–B2 在声学层面失效，且库无任何告警可定位（仅负 `buffered` 有一种 warning，jitter.c:761）。
- **D4（容量丢弃）**：缓冲占用达 200 包时新包淘汰最老包（419–437），被淘汰包对应的播放区间只能以 MISSING 占位覆盖；该事件对调用者不可见。
- **D5（取整边界）**：MISSING 路径 `ROUND_DOWN(desired_span, concealment_size)=0` 时指针不推进，B1 的"必然推进"依赖调用者保证 `desired_span >= concealment_size`。
- **D6（统计视野外的损伤）**：直方图只保留每子窗 40 条最极端样本、总视野约 1000 个到达事件（默认配置）；超出该窗口的抖动历史不影响当前建议，短时突发在窗口老化后可能导致延迟回落、再次遭遇同类突发时重新走 INSERTION 等待——连续性仍由 B1 维持，但补偿占比会周期性升高。

### 6.3 一句话判定

> 只要调用者履行周期驱动与 PLC 责任，SpeexDSP 抖动缓冲在两次重同步之间保证**时间戳轴无缝、单调、每段有主**：延迟增长以重复区间吸收抖动且不产生空洞，延迟收缩以一次受控跳变归还时延，迟到包在一个 `delay_step` 的显式窗口内可被救回，超过 20 次连续取包失败则确定性重同步；库保证的全部连续性到此为止，补偿音频的听感、丢包/迟到的区分与计数、线程安全与调用时序均在边界之外。
