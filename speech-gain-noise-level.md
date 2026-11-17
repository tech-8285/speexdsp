# SpeexDSP 自动增益（AGC）与降噪同时启用时的输出电平分析

基于本仓库 `libspeexdsp/preprocess.c`（1215 行）与 `include/speex/speex_preprocess.h` 的实际代码。
所有结论均来自静态代码阅读；**未做任何实际输出电平测量**，这一限制对结论性质的影响见文末。

## 1. 单帧处理链总览（`speex_preprocess_run`，preprocess.c:722）

一帧数据依次经过：

1. **分析**（`preprocess_analysis`，preprocess.c:614）：输入与 `inbuf` 拼接成 2N 点帧，乘分析/合成共用的 `window`（`conj_window`），定点构建下按峰值做 `frame_shift` 块浮点移位，然后 FFT，计算功率谱 `ps`，并经 `filterbank_compute_bank32` 得到 24 个 Bark 带的带域功率（`ps[N..N+M)`）。
2. **噪声估计**（`update_noise_prob` + preprocess.c:775）：Cohen 式最小值跟踪（`S`/`Smin`/`Stmp`/`update_prob`）决定哪些频bin允许更新噪声估计 `noise`，自适应速率 `beta = max(0.03, 1/nb_adapt)`（preprocess.c:740）。
3. **SNR 与语音概率**：由 `noise`+`echo_noise`+`reverb_estimate` 算后验 SNR `post`、先验 SNR `prior`（均钳到 100），平滑得 `zeta`；整帧语音概率 `Pframe = 0.1 + 0.899*qcurve(mean(zeta[N..N+M)))`（preprocess.c:821）。
4. **降噪增益**：Bark 域 Ephraim-Malah 增益 + 语音存在概率 `gain2`，经 `filterbank_compute_psd16` 插回线性频率；线性域再算一次 EM 增益，受 Bark 增益 3 倍约束、受 `gain_floor`（由 `noise_suppress`/`echo_suppress` 决定）托底，最后 `gain2 = [p*sqrt(gain) + (1-p)*sqrt(gain_floor)]^2`（preprocess.c:914-915）。若 `denoise_enabled==0`，`gain2` 全部置 1（preprocess.c:933-937）。
5. **增益应用**：`gain2` 逐bin乘到频谱 `ft` 上（preprocess.c:940-946）。
6. **AGC**（仅浮点，`speex_compute_agc`，preprocess.c:565）：见第 2 节。
7. **IFFT**（1/N 缩放），定点下按 `frame_shift` 移回。
8. **峰值限制**（仅浮点且 AGC 开启，preprocess.c:961-975）：若帧内最大样本超过 28000，整帧乘 `28000/max_sample`。
9. **重叠合成（WOLA）**：再乘同一 `window`，与 `outbuf` 重叠相加写出 `frame_size` 个样本，余下 `N3 = 2N - frame_size` 个样本存入 `outbuf` 留给下一帧（preprocess.c:977-989）。

**降噪后的频谱如何进入增益处理**：`gain2` 直接乘在 `ft` 上，AGC 紧接着在同一个 `ft` 上再乘 `agc_gain`（preprocess.c:608-609），即 AGC 作用于**已降噪的频谱**。但注意：AGC 的响度估计循环用的是 `st->ps[i]`（preprocess.c:573-576），而 `ps` 是分析阶段算出的**降噪前**输入功率谱，之后并未被 `gain2` 改写。因此 AGC 看到的响度包含残余噪声能量，降噪深度会间接影响 AGC——降噪越狠，输出实际电平相对 AGC 预期越低，因为 AGC 按输入响度（含噪）计算目标增益，却施加在已衰减的频谱上。

## 2. AGC 的目标电平为什么不等于每帧电平

`speex_compute_agc`（preprocess.c:565-611）的核心公式：

```
target_gain = AMP_SCALE * agc_level * (loudness / (1e-4 + loudness_accum))^(-1/LOUDNESS_EXP)
```

其中 `AMP_SCALE=0.001`，`agc_level` 默认 8000，`LOUDNESS_EXP=5`。`loudness` 是感知加权（`loudness_weight`，3.8 kHz 附近加权的响度曲线）功率谱开根号后的**历史平滑值**，不是当前帧值。目标电平 8000 只是统计稳态目标，实际增益被以下机制层层约束：

### 2.1 语音概率（Pframe）的门控

- **响度历史更新门控**（preprocess.c:580）：只有 `Pframe > 0.3` 时，`st->loudness` 才以 `rate = 0.03*Pframe^2` 向当前帧响度靠拢，`loudness_accum` 同步累积。静音/噪声帧不更新历史——历史只记录"像语音"的响度。
- **增益上调门控**（preprocess.c:593）：只有 `(Pframe > 0.5 && nb_adapt > 20)` 或 `target_gain < agc_gain` 时才允许改增益。即**增益上调必须语音概率过半且已度过启动期；增益下调随时允许**（保护性不对称）。

### 2.2 响度历史的惯性

`rate = 0.03*Pframe^2` 最大约 0.03/帧，响度历史是时间常数约 33 帧（8 kHz/160 样本帧时约 0.66 s）的指数滑动平均。`loudness` 初始为 `1e-15`、`loudness_accum` 初始为 0（`speex_alloc` 即 calloc，os_support.h:56），所以刚启动时 `loudness/(1e-4+loudness_accum)` 极小、`target_gain` 极大——这正是 `init_max` 存在的原因。

### 2.3 静音 → 弱语音 → 强语音的时序推演

- **静音段**：`Pframe` 低（qcurve 输出接近 0.1），响度历史冻结，增益不变。
- **弱语音出现**：若 `Pframe` 在 0.3~0.5 之间，响度历史开始缓慢更新，但增益**不允许上调**（Pframe≤0.5 门控），输出电平维持现状；只有 `Pframe>0.5` 且 `nb_adapt>20` 后增益才以每帧最多 `max_increase_step`（默认 +12 dB/s，preprocess.c:515）爬升，同时受 `init_max` 压制（见 2.4）。因此弱语音开始后的若干帧内，输出电平**显著低于**目标 8000。
- **强语音出现**：当前帧响度 `loudness` 突增，但历史 `st->loudness` 仍停留在弱语音水平 → 公式给出的 `target_gain` 偏大，增益只能以 +12 dB/s 缓慢上调；同时历史以 `rate≈0.03` 被拉高，`target_gain` 逐帧回落。若响度突增超过 10 倍（`loudness >= 10*prev_loudness`），preprocess.c:597 的降速钳制**不生效**，允许增益一步降到位——这是对突发强音的快速保护；而缓变时降速被钳在 -40 dB/s（默认，preprocess.c:516）。无论哪条路径，强语音的头若干帧输出电平都**不等于**目标电平，只是被速率限制和峰值限制框在边界内。

### 2.4 增益上限、变化速率、峰值限制冲突时的输出边界

增益在每帧内按以下顺序被钳制（preprocess.c:595-602）：

1. `target_gain ≤ max_increase_step * agc_gain`（升速限制，默认 +12 dB/s）；
2. `target_gain ≥ max_decrease_step * agc_gain`，**但仅当** `loudness < 10*prev_loudness`（降速限制，默认 -40 dB/s；突强音豁免）；
3. `target_gain ≤ max_gain`（默认 30 倍 ≈ +29.5 dB，preprocess.c:514）；
4. `target_gain ≤ init_max`（启动期增益上限）。

`init_max` 初始为 1（即启动时根本不允许放大），仅当 `Pframe>0.3` 且 `nb_adapt>20` 时按 `init_max *= 1 + 0.1*Pframe^2` 每帧增长（preprocess.c:586-587），封顶 `max_gain`。冲突时的优先级：**`init_max` 最后钳制，启动期它是最紧的约束**；`max_gain` 是稳态上限；速率限制决定多快能碰到这些上限。

最后的硬边界是**峰值限制**（preprocess.c:968-973）：AGC 之后、合成窗之前，若时域峰值超过 28000（约 -1.3 dBFS），整帧等比压缩到 28000。因此即使 `agc_gain` 与信号幅度的乘积理论上会削波，输出样本也被压在 ±28000 以内。这是一个"数值保护"边界，它可能让输出电平**低于** AGC 目标——峰值限制与目标电平冲突时，峰值限制赢。

综合起来，任何单帧的输出电平上界为：

```
min(agc_gain_本帧允许值, 信号自身的峰值限制 28000/峰值) × 降噪后信号
```

下界没有显式保护（增益可以降到公式值，弱信号不会被强行抬高到目标）。

## 3. 启动阶段与设置切换后残留的历史

### 3.1 启动阶段（`speex_preprocess_state_init`，preprocess.c:391）

- 噪声估计 `noise[i]` 初始化为 1.0（`QCONST32(1, NOISE_SHIFT)`），`old_ps=1`，`gain=1`，`post=prior=1`，`update_prob=1`；`Smin/Stmp` 在第一帧清零（preprocess.c:672-676）。
- `beta = max(0.03, 1/nb_adapt)`：前约 33 帧噪声估计自适应更快，之后固定在 0.03。
- 最小值跟踪窗口 `min_range` 随 `nb_adapt` 分级增长：15 / 50 / 150 / 300 帧（preprocess.c:678-685），即启动初期噪声底更新更频繁。
- AGC 侧：`loudness=1e-15`、`loudness_accum=0`（calloc 清零）、`agc_gain=1`、`init_max=1`、`prev_loudness=1`。前 20 帧（`nb_adapt≤20`）增益完全不能上调，`init_max` 之后也要数十帧才能爬到 `max_gain`。
- `nb_adapt` 封顶 20000（preprocess.c:736-737），`beta` 因此不会小于约 0.03。

### 3.2 设置切换后仍然存在的 history

`speex_preprocess_ctl`（preprocess.c:1044）只改参数，**不清任何历史**：

- **关/开降噪**（`SET_DENOISE`）：关闭期间 `gain2` 被强制为 1，但噪声估计、`old_ps`、`zeta`、`prior`、最小值跟踪全部照常更新。重新开启时这些历史直接生效，不会重新收敛。
- **关/开 AGC**（`SET_AGC`）：`speex_compute_agc` 整个不被调用，`loudness`/`loudness_accum`/`agc_gain`/`init_max`/`prev_loudness` 全部冻结在关闭前的值。重新开启后从旧历史继续——若关闭期间输入电平变化很大，开启瞬间的 `target_gain` 会基于过时历史计算，需要重新收敛。
- **改 `NOISE_SUPPRESS`/`ECHO_SUPPRESS`**：只影响 `compute_gain_floor` 的参数，下一帧即生效，但噪声谱估计本身不变。
- **改 `AGC_LEVEL`/`AGC_TARGET`**：只改目标，响度历史保留，增益按速率限制向新目标迁移。
- **`speex_preprocess_estimate_update`**（preprocess.c:1009）：只更新噪声估计、`old_ps`、`outbuf`，`min_count++` 但 **`nb_adapt` 不增加**，也不触碰 AGC 历史。

## 4. 浮点与定点构建的支持差异、旧设置兼容

- **AGC 整体仅浮点可用**：`agc_*` 字段、`speex_compute_agc`、28000 峰值限制、以及 `SET_AGC`/`SET_AGC_INCREMENT`/`SET_AGC_DECREMENT`/`SET_AGC_MAX_GAIN`/`GET_AGC_LOUDNESS`/`GET_AGC_GAIN`/`SET_AGC_TARGET` 等全部 ctl 都在 `#ifndef FIXED_POINT` 内（preprocess.c:227-240, 563-612, 949-975, 1057-1094, 1170-1204）。代码中明确注释 "FIXME: The AGC doesn't work yet with fixed-point"（preprocess.c:563）。**定点构建下调用这些 ctl 会落到 default 分支返回 -1**，且没有任何峰值限制——定点下输出仅依赖 `gain2≤1` 的降噪增益和 `frame_shift` 块浮点保护。
- **`DISABLE_FLOAT_API`**（configure.ac:171）：`SET_AGC_LEVEL`/`GET_AGC_LEVEL`（float 参数版本，preprocess.c:1065-1075）被裁掉；`SET_AGC_TARGET`/`GET_AGC_TARGET`（int32 版本，preprocess.c:1194-1203）仍可用。**旧代码兼容条件**：使用 float 版 `SET_AGC_LEVEL` 的应用在 `DISABLE_FLOAT_API` 构建下会收到 -1，必须改用 `SET_AGC_TARGET`；两者都写 `agc_level` 并同样钳到 [1, 32768]，语义等价，仅参数类型不同。
- 定点构建另有自身的数值保护：`DIV32_16_Q8/Q15` 饱和返回 32767（preprocess.c:92-142）、`post/prior` 钳到 100、`hypergeom_gain` 查表插值（preprocess.c:292-308）。这些保护的是**数值范围**，与输出响度是否达标无关。

## 5. 结论：数值保护 vs 听感目标，以及测量限制

可以从代码直接断言的（**数值范围受保护**类结论）：

1. 浮点 + AGC 开启时，输出样本峰值被硬限制在 ±28000（约 -1.3 dBFS），不会削波；
2. AGC 增益逐帧变化率受 `max_increase_step`/`max_decrease_step` 限制（突强音豁免降速限制），增益幅值受 `max_gain` 与启动期 `init_max` 限制；
3. 定点构建下除法饱和、SNR 钳幅、`frame_shift` 块浮点保证中间量不溢出；
4. `agc_level` 被钳在 [1, 32768]。

**不能**从代码断言的（**语音听感达到目标**类结论）：

1. 目标电平 `agc_level`（默认 8000）只是稳态统计目标。由于 Pframe 门控、响度历史惯性、速率限制、`init_max` 启动压制和 28000 峰值限制，**任意给定帧的输出电平一般不等于目标电平**；静音→弱语音→强语音过渡期内偏差尤其大。
2. AGC 的响度估计基于降噪前的输入功率谱，而增益施加在降噪后的频谱上，噪声抑制深度会系统性影响实际输出电平相对目标的偏移方向与大小，其量值取决于实际噪声谱，无法仅从代码确定。
3. 本分析**未运行任何音频、未取得实际输出电平测量**。因此凡涉及"输出电平达到多少""听感是否达到目标响度"的论断，本文一律不给出；上述第 2 节的所有推演只是代码路径的上界/下界分析。要得到听感类结论，必须用真实或合成信号实测输出电平（例如经 `SPEEX_PREPROCESS_GET_AGC_GAIN`/`GET_AGC_LOUDNESS` 读取运行时状态并记录输出 RMS），这超出本次静态分析的范围。
