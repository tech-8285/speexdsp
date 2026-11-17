# 回声残差频谱如何改变预处理器的频域抑制边界

分析对象：本仓库 SpeexDSP 1.2.1（`configure.ac`），核心代码
`libspeexdsp/preprocess.c`（预处理器，Ephraim–Malah 降噪 + 残余回声抑制）与
`libspeexdsp/mdf.c`（MDF 声学回声消除器 AEC）。
所有结论均标注证据类型：**【源码可证】** 表示由代码直接推出的处理契约；
**【实测】** 表示本次用 ASan/UBSan 与行为对照实验验证（实验工程位于仓库外
`/tmp/speex-asan-check/`，未改动仓库源码）；**【待音质证据】** 表示涉及听感/保真度，
静态代码与无崩溃运行都无法确认的结论。

---

## 0. 术语与模块边界

| 模块 | 入口 | 职责 |
|---|---|---|
| AEC（`mdf.c`） | `speex_echo_cancellation()` | 用自适应块频域滤波器估计线性回声并从麦克信号中减去，产出"近场+残余"误差信号 `out`；同时维护泄漏率 `leak_estimate`、自适应状态 `adapted` 等 |
| 预处理器（`preprocess.c`） | `speex_preprocess_run()` | 对 AEC 输出做 WOLA 频域处理：噪声估计、Ephraim–Malah 增益、**残余回声抑制（RES）**、可选 AGC/VAD |

标准接线（`libspeexdsp/testecho.c:34-45`）：两个状态以**相同 frame_size** 创建，
通过 `SPEEX_PREPROCESS_SET_ECHO_STATE` 绑定，每帧先 `speex_echo_cancellation()` 后
`speex_preprocess_run()`。头文件明确要求 frame_size 必须一致，
"for residual echo cancellation to work"（`include/speex/speex_preprocess.h:59-62`）。

---

## 1. 残差频谱如何进入预处理器

### 1.1 数据通路（按帧执行顺序）

1. **AEC 侧生成残余回声功率谱**——`mdf.c:1190-1221` `speex_echo_get_residual()`：
   - 取 AEC 内部缓存的"被减去的回声估计" `st->last_y`（长度 `window_size = 2*frame_size`，`mdf.c:419`）；
   - 加 Hanning 窗（`mdf.c:1199-1200`），做 `spx_fft`，再由 `power_spectrum()` 取**半谱**（`mdf.c:228-237`），输出 `window_size/2+1 = frame_size+1` 个功率字；
   - 乘以泄漏系数 `leak2 = clamp(2*leak_estimate)`（浮点：`mdf.c:1212-1215`；定点：`1207-1210`）。
2. **预处理器每帧拉取**——`preprocess.c:744-761`：仅当 `st->echo_state != NULL` 时调用
   `speex_echo_get_residual(st->echo_state, st->residual_echo, N)`（`N = ps_size = frame_size`）。
   注意传入的 `len=N` 在 AEC 内部**被忽略**，实际长度由 AEC 的 `window_size` 决定（`mdf.c:1196`）。
3. **浮点健壮性闸门**——`preprocess.c:748-754`：仅 `#ifndef FIXED_POINT` 存在；
   若 DC bin 为 NaN 或 ≥ `N*1e9`，整段残余谱清零。**定点构建无此保护。**
4. **时间维最大保持平滑**——`preprocess.c:755-756`：
   `echo_noise[i] = max(0.6 * echo_noise[i], residual_echo[i])`。
   这是只增不减的 max-hold 加指数松弛，保证残余估计不会瞬时塌缩，但也意味着远端停止后
   `echo_noise` 以每帧 ×0.6 的速率缓慢衰减。
5. **映射到 24 个 Bark 子带**——`filterbank_compute_bank32()`（`preprocess.c:757`，
   实现见 `filterbank.c:134-154`）。滤波器组在 `speex_preprocess_state_init` 中按采样率与
   `NB_BANDS=24`（`preprocess.c:75`）以 Bark 尺度构建，每个 FFT bin 以线性权重分到相邻两带。
6. **进入总噪声与增益下限两条路径**（详见 1.3）。

`last_y` 的内容受 AEC 自适应门控（`mdf.c:1176-1185`）：仅当 `st->adapted` 为真时才填入
`in - out`（实际被减掉的回声）；未收敛时该缓存保持上一帧平移后的旧值，初始为零
（`mdf.c:548-551`）。`adapted` 的置位条件是累计自适应量足够且泄漏能量显著（`mdf.c:1124-1127`）。

### 1.2 回声状态缺失或变化时，哪些频带量仍然有效

| 情形 | 代码行为 | 仍然有效的量 | 失效/退化的量 |
|---|---|---|---|
| **未绑定/运行时置 NULL**（`preprocess.c:758-761`） | `echo_noise[0..N+M-1]` 每帧全部清零 | 麦克风功率谱 `ps`、噪声估计 `noise`、先验/后验 SNR（`post`/`prior`）、平滑先验 `zeta`、帧语音概率 `Pframe`、EM 增益 `gain/gain2`、VAD、AGC——全部继续工作 | 残余回声抑制完全失效；增益下限退化为纯噪声下限（见 1.3 公式，`echo=0` 时） |
| **AEC 尚未收敛**（`adapted=0`） | `last_y` 为零/旧值，`leak_estimate` 初值 0（`mdf.c:436`）且被 `MIN_LEAK=.005` 托底（`mdf.c:1085`） | 预处理器自身的全部降噪链路 | 残余谱近似 0，RES 实际不动作；这是"无残差输入"而非故障 |
| **运行时切换到另一个 AEC 指针**（`preprocess.c:1164-1166` 仅裸指针赋值） | 下一帧立即从新 AEC 拉取；旧 `echo_noise` 平滑值保留并参与 max-hold | 降噪链路不受影响 | 无任何尺寸/状态校验（见第 3 节兼容性判据与实测缺陷） |
| **AEC 内部复位**（`screwed_up>=50` 触发 `speex_echo_state_reset`，`mdf.c:1032-1037`） | `last_y`、滤波器、`leak_estimate`、`adapted` 全部归零 | 预处理器 `echo_noise` **不会被通知复位**，仍保留复位前的 max-hold 残余估计并按 ×0.6 衰减 | 复位后若干帧内 RES 依据陈旧回声谱抑制——方向安全（偏保守），但可能误伤近场语音 |
| **饱和/远端异常**（`saturated`，`mdf.c:731-763, 973-977`） | AEC 冻结自适应若干帧 | 预处理器降噪链路 | 残余估计质量下降，但接口不暴露该状态 |
| **输入谱出现 NaN/荒谬值（仅浮点）** | DC 检查命中后整段残余清零（`preprocess.c:749-753`） | 降噪链路 | RES 当帧静默关闭 |

### 1.3 残余谱改变频域抑制边界的两个作用点

**作用点 A：抬高总噪声，压低先验/后验 SNR**（`preprocess.c:788-805`）

```c
tot_noise = 1 + (noise[i] >> NOISE_SHIFT) + echo_noise[i] + reverb_estimate[i];
post[i]  = ps[i]/tot_noise - 1;        // 后验 SNR，封顶 100
prior[i] = γ·max(0,post) + (1-γ)·old_ps/tot_noise;  // 决策导向先验 SNR
```

- 回声能量越大，`tot_noise` 越大，SNR 越低，后续 Ephraim–Malah 增益越小。
- `reverb_estimate` 在本版本恒为 0：去混响的更新语句被注释（`preprocess.c:909-910`），
  ctl setter 是空 FIXME（`preprocess.c:1112-1128`），仅 `estimate_update` 里做衰减
  （`preprocess.c:1040`）而 `run()` 从不增加它。**【源码可证】当前版本实际只有
  "背景噪声 + 残余回声"两项参与总噪声。**

**作用点 B：增益下限（gain floor）按噪声/回声能量加权混合**（`preprocess.c:823-825, 376-388`）

```c
effective_echo_suppress = (1-Pframe)·echo_suppress + Pframe·echo_suppress_active;
// 默认 echo_suppress=-40 dB, echo_suppress_active=-15 dB, noise_suppress=-15 dB
gain_floor = sqrt(noise_floor·noise + echo_floor·echo) / sqrt(1 + noise + echo);
```

- 默认值（`preprocess.c:79-81`）：纯噪声/双讲时最多压 −15 dB；判定无近场语音时
  残余回声最多压 −40 dB。`Pframe` 是由 24 带平均先验 SNR 经 `qcurve` 映射的帧级语音概率
  （`preprocess.c:817-823`），在两个抑制深度间连续插值。
- 最终每 bin 增益先与 Bark 带增益互相约束（`preprocess.c:897-900`，线性增益不得低于
  Bark 增益的 1/3），再应用下限（`:906-907`），最后按语音存在概率 `p` 在下限与 EM 增益间
  做响度域混合（`:912-915`）。残余回声因此**同时**通过 SNR（软决策）和 gain floor
  （硬下限）改变每个频点的抑制边界。

### 1.4 去噪与残差抑制的责任边界

- **AEC 的责任**：线性回声建模与相减；它只向预处理器提供"回声功率谱 + 泄漏率"，
  不提供相位、不提供远端/双讲显式标志。预处理器拿到的始终是**功率量**，无法做相位对消。
- **预处理器降噪的责任**：`noise[]` 的更新只看 `update_prob`（Cohen 最小统计量，
  `preprocess.c:660-711`）与当前 `ps`（`:775-779`），**不直接读 `echo_noise`**。
  即噪声估计器本身不把残余回声当噪声学习——回声只在增益计算阶段汇入 `tot_noise` 与下限。
- **RES 的责任**：完全寄生在降噪增益流水线内，没有独立输出路径。
  `denoise_enabled=0` 时 `gain2` 被强制为 1（`preprocess.c:933-937`），RES 与降噪一起被旁路。
  **【实测】** 四组对照（回声绑定/NULL × 降噪开/关，各 400 帧、跳过 20 帧 WOLA 预热）：
  降噪关时绑定 AEC 与 NULL 的最大样本差完全相同；做一帧延迟对齐后，降噪关+无回声的
  输入输出最大差为 **0.0000**，证明此时残余路径对波形零影响，仅存在 WOLA 的一帧算法延迟。
- **VAD 是同一套估计的消费者而非独立裁判**：`Pframe` 同时驱动 RES 深度插值（`:823`）与
  VAD 判决（`:992-1003`），代码自述 "This VAD is a kludge"（`:991`），
  ctl 中也有 "replaced by a hack pending a complete rewrite" 警告（`:1096`）。

---

## 2. 频域估计对语音保真的约束与实际取舍

### 2.1 【源码可证】结构性约束（处理契约，不依赖主观听感）

1. **抑制只作用于幅度，不触碰相位**：增益 `gain2` 是实数，分别乘 FFT 的实/虚部
   （`preprocess.c:940-946`），相位保持不变。残余回声与近场语音在同一频带重叠时，
   该带整体被压，无法只压回声保留语音——双讲误伤是算法结构决定的。
2. **硬下限保证最大衰减有界**：任何频点增益不会低于按能量加权的 `gain_floor`
   （`:906-907`），默认 −15 dB（双讲）/−40 dB（纯回声）。这意味着：
   - 保真方向：语音不会被彻底挖空（最多衰减到下限）；
   - 残留方向：下限以下的回声/噪声必然残留，−40 dB 是设计上限而非测量结果。
3. **帧级语音概率连续插值抑制深度**（`:823`）：双讲判定不是硬开关，`Pframe` 估计偏差会
   直接平移整个频带的抑制边界；而 `Pframe` 来自递归平滑的先验 SNR（`zeta`，时间常数 0.7，
   `:810-815`），天然滞后于突发双讲。
4. **Bark 分辨率约束**：24 带 Bark 滤波器组（`:440-442`）决定抑制边界的频率粒度；
   线性 FFT 增益还被 Bark 增益 1/3 规则钳制（`:897-900`），窄带语音/回声无法独立处理。
5. **max-hold 残余平滑（×0.6 松弛，`:755-756`）**：远端骤停后回声下限需多帧才消退，
   期间持续压低对应频带——以"残留风险换语音流畅"的保守取舍，参数为硬编码、不可通过 ctl 配置。
6. **WOLA 一帧延迟与窗形**：分析/合成窗 + 50% 重叠相加（`:614-658, 977-989`）引入固定
   一帧算法延迟；**【实测】** 单位增益下延迟对齐后波形逐样本一致（max diff = 0），
   即窗函数本身不引入稳态失真，但瞬态帧的边界效应属于结构特性。
7. **SNR 估计的封顶与递归**：`post`/`prior` 均封顶 100（`:797, 804`），`old_ps` 按
   0.2 旧值 + 0.8·增益²·当前功率更新（`:903`），强音后的频谱拖尾会影响下一帧判决。
8. **定点构建的额外约束**：无残余谱 NaN 闸门（`:747` 条件编译）；`NOISE_SHIFT=7`
   （`:156`）改变噪声量的定标；AGC 明确不支持定点（`:563, 948, 960` 三处 FIXME）。

### 2.2 【待音质证据】无法由源码或"无崩溃"确认的结论

以下各项代码只能证明"机制存在"，是否构成可接受的音质必须用客观指标
（PESQ/POLQA/ERLE/双讲语音谱失真）与主观听测确认：

- **默认 −40/−15 dB 抑制深度的实际听感平衡**：代码证明了衰减界限，证明不了该界限下
  双讲语音的自然度与回声残留的可接受度。
- **max-hold 平滑在远端骤停后的"闷/水声"程度**：衰减速率 0.6/帧可证，听感后果不可证。
- **Bark 带间插值与 1/3 钳制对音乐噪声（musical noise）的影响**：EM 增益的时频抖动
  是否产生可闻金属音，需要频谱图与听测。
- **`Pframe` 滞后导致的双讲开头吞字**：滞后机制可证，吞字时长/可懂度损失需证据。
- **AEC 复位后陈旧 `echo_noise` 误伤近场的持续时长与严重度**（1.2 节第 4 行）。
- **定点版本相对浮点版本的音质差距**：定标、Q15 超几何查表（`:292-308`）、
  无 NaN 闸门等差异的音质影响需要 A/B 实测。
- **泄漏率 `leak_estimate` 被夹在 [0.005, 1]（`mdf.c:1085-1088`）对 RES 激进度的实际影响。**

> 明确立场：本次实验中所有配置均无崩溃、ASan 在合法配置下无告警，但**无崩溃只证明内存与
> 控制流安全，不构成任何音质合格证据**。代码自带的三处自认缺陷（VAD kludge、去混响停用、
> 定点 AGC FIXME）也说明该实现的部分路径本身就是未完成状态。

---

## 3. 状态变化与输入不完整时的兼容判据

### 3.1 【源码可证 + 实测】判据清单

| # | 场景 | 兼容判据（满足即可安全工作） | 证据 |
|---|---|---|---|
| C1 | 绑定 AEC | 预处理器与 AEC 的 `frame_size` **必须相等**；相等时 AEC 写 `frame_size+1` 个半谱字，预处理器 `residual_echo` 容量为 `frame_size+24` 字（`preprocess.c:451`），余量 23 字，不会越界 | 头文件要求 `speex_preprocess.h:59-62`；【实测】`frame_size=128` 时哨兵实验确认最高写入下标 128，容量 152，ASan 无告警 |
| C2 | frame_size 不匹配 | **无任何运行时校验**：`SET_ECHO_STATE` 只存指针（`preprocess.c:1164-1166`），`get_residual` 忽略传入 len 而用 AEC 自己的 `window_size`（`mdf.c:1196`）。AEC frame 大于预处理器 frame 时发生堆越界写 | 【实测】AEC=256 / PP=128 时 ASan 报 `heap-buffer-overflow WRITE of size 4 ... mdf.c:1204`，写入点正是 `preprocess.c:451` 分配的 608 字节块之后；调用栈 `speex_echo_get_residual → speex_preprocess_run:746` |
| C3 | 未绑定 / 置 NULL | 合法且每帧显式清零 `echo_noise`（`:758-761`）；RES 静默停用，降噪功能完整 | 【源码可证】；【实测】NULL 配置 400 帧行为正常 |
| C4 | 运行时切换 AEC 指针 | 不崩溃、下一帧生效；但调用方必须自行保证 C1 的尺寸约束，否则等同于 C2 | 【源码可证】无校验代码路径 |
| C5 | AEC 未收敛 | 兼容：残余谱≈0，RES 空转，降噪正常；无需上层特殊处理 | `mdf.c:1124-1127, 1176-1185` |
| C6 | AEC 运行中复位 | 不崩溃；但预处理器不知情，陈旧 `echo_noise` 按 0.6/帧自然衰减，期间偏保守抑制 | `mdf.c:1032-1037` + `preprocess.c:755-756` |
| C7 | 残余谱 NaN/越界（浮点） | 单帧自愈：DC 检查命中即整段清零（`:749-753`）；定点构建无此机制，上层需自行保证不注入异常定点状态 | 条件编译 `:747` |
| C8 | 降噪关闭 | RES 同步旁路（`:933-937`）；此时绑定 AEC 无意义但无害；WOLA 一帧延迟仍在 | 【实测】延迟对齐 max diff=0 |
| C9 | 抑制参数被改成正数 | ctl 强制取负（`-ABS()`，`:1147, 1153, 1159`），不会产生正增益下限 | 【源码可证】 |
| C10 | 采样率 | AEC 与预处理器分别保存采样率；滤波器组按预处理器采样率构建（`:442`），AEC 泄漏/陷波器按自身采样率配置（`mdf.c:1231-1246`）。代码不校验两者一致，不一致时频带对应关系失真但不崩溃——应视为**调用方契约**而非受支持配置 | 【源码可证】 |
| C11 | 只调 `speex_preprocess_estimate_update()` | 该路径**完全不读 echo_state**（`:1009-1041`），只更新噪声/统计量；与 `run()` 交替使用时 `echo_noise` 停在最后一次 `run()` 的值 | 【源码可证】 |
| C12 | 废弃 API `speex_preprocess(st,x,echo)` | `echo` 参数被忽略，直接转 `speex_preprocess_run`（`:717-720`）；`speex_echo_cancel` 末参同理被忽略（`mdf.c:680-683`）。旧式显式传残余谱的用法在本版本已无效，RES 只能经 `SET_ECHO_STATE` 获得数据 | 【源码可证】 |

### 3.2 证据缺口（本报告未能闭合、需补充工作的部分）

1. **音质证据全面缺失**：仓库内无测试音频（`regressions/` 无 `.sw/.raw/.pcm`），
   本次实验使用合成信号，只能验证内存安全与数值通路，无法给出 ERLE、PESQ/POLQA、
   双讲可懂度、音乐噪声等任何质量指标。第 2.2 节全部条目待真实语料实测。
2. **定点构建未实测**：受限于环境，ASan/行为实验仅覆盖浮点（`FLOATING_POINT +
   USE_SMALLFT`）。定点路径的 NaN 无保护（C7）、Q15 查表与 `NOISE_SHIFT=7` 定标的行为
   仅做了静态分析，未运行验证。
3. **C2 越界的真实触发面未在完整应用中复现**：已用最小复现确证越界写存在，
   但实际工程中 frame_size 通常按头文件约定保持一致；该缺陷的触发前提（错误接线）
   在生产代码中的出现概率需要调用方代码审计才能评估。
4. **AEC 复位（C6）与饱和冻结期对 RES 的量化影响**：机制可证，持续帧数与误伤频带范围
   未做插桩测量。
5. **非 2 的幂 / 非常规 frame_size 的行为**：`ps_size` 直接等于 `frame_size`
   （`:418`，旧的取整逻辑被 `#if 0` 停用），FFT 表按 `2*N` 初始化（`:522`）；
   非常规尺寸下 FFT 与滤波器组的正确性未测试。
6. **多通道（`speex_echo_state_init_mc`，C/K>1）与预处理器单通道模型的组合**：
   `get_residual` 与 `last_y` 的多通道路径在 mdf.c 中标注 FIXME（`:1173`），未验证。
7. **长时间运行的数值漂移**：实验最长 400 帧；`nb_adapt` 在 20000 帧封顶
   （`:736-737`）、`min_count` 窗口随适应阶段扩大（`:678-685`）等长时行为的稳态影响未测。

---

## 4. 结论摘要

- 残余回声以**功率谱（无相位）**形式经 `speex_echo_get_residual → max-hold 平滑 →
  24 Bark 带`进入预处理器，通过"抬高总噪声压低 SNR"与"能量加权增益下限"两条路径
  改写每个频点的抑制边界（`preprocess.c:744-825, 897-915`）。
- 回声状态缺失时预处理器的全部降噪/VAD/AGC 量仍然有效，仅 RES 静默停用；
  AEC 未收敛、浮点 NaN 等不完整输入均有确定性退化路径，但 AEC 复位会留下未通知的
  陈旧残余估计。
- 去噪与 RES 共享同一条增益流水线、由同一个 `Pframe` 驱动，且 RES 无独立开关
  （随 `denoise_enabled` 一起旁路）；责任边界是"AEC 管线性相减，预处理器管幅度域
  概率抑制"，双讲重叠频带的语音误伤是该结构的固有取舍。
- 已实测确证一个真实健壮性缺陷：**frame_size 不匹配时 `mdf.c:1204` 堆越界写，
  且 `SET_ECHO_STATE` 无任何校验**（C2）。合法配置下内存安全，但合法配置的无崩溃运行
  不提供任何音质保证；第 2.2 节的保真度结论与第 3.2 节的七项证据缺口必须以真实语料、
   objective 指标和（资源允许时）定点构建实测来闭合。
