# PROGRESS

## [2026-08-31] 主循环黑洞专项（LOOP_PROF 探针 + 判别树 D1-D4，三条挂账全证伪）

### Problem / Task
- 三条"不该贵的活很贵"挂账: CMD:ON 压主循环 50% / PREF -0.55%/Hz 压 PDBBIN / ISR 随转速暴涨 8-14×,
  纸面算 ~1% CPU 但实测差 20-100×, 同一嫌疑结构(随负载伸缩的 ms 级黑洞)。
  Kimi 任务卡: 主循环五段 DWT 探针 (ISR 税校正) + E0-E4 实验矩阵 + 判别树 D1-D4。

### Resolution
- **P1 探针**: foc_profiler 新增 SEG_CMD/SEG_PREF/SEG_PDB/SEG_NFRAME/SEG_OTHER 五段 +
  `FOC_Profiler_SegBegin/SegEnd` (句柄快照 wall+isr 对, 净耗时=墙钟差−ISR差, 可嵌套);
  TIM1_UP ISR 每拍把墙钟耗时累入 64 位账本 (禁中断快照无撕裂)。
  `LOOP_PROF_EN` 编译开关: GCC 双路径验证 开 185088B / 关 183512B, 置 0 零成本。
- **P1 命令**: `CMD:LOOP_PROF?` BEGIN...END 事务 (P0 逐行泵), `CMD:LOOP_PROF,CLEAR`。
  修复两个 BUSY 死锁: (1) END 行发后未清 active; (2) 探针索引越界
  (`<=FOC_PROBE_COUNT` → `<=COUNT-1`), END 行永不发送。
- **P2 实验**: scripts/loop_prof_exp.py (E0-E4×30s, 落盘 JSON + PDBBIN 三元组)。
  主机侧 lessons: fetch 快照前必须停 PDBBIN + 排空残留 (二进制帧撞碎文本行)。

### Verification (E0-E4 判别树全闭环)
| 档 | SEG_CMD | SEG_PREF | SEG_PDB | SEG_NFRM | SEG_OTH | 主循环 | PDBBIN | gap |
|---|---|---|---|---|---|---|---|---|
| E0 | 29.6µs | 0 | 22.7µs | 1.5µs | 11.7µs | 9.4kHz | 691Hz | 3 |
| E1 | 28.4µs | 0 | 21.5µs | 14.0µs | 11.5µs | 8.3kHz | 1269Hz | 145 |
| E2 | 29.7µs | 4.5µs | 21.5µs | 14.3µs | 11.5µs | 8.1kHz | 1834Hz | 285 |
| E3 | 31.0µs | 3.7µs | 22.9µs | 1.6µs | 11.5µs | 9.2kHz | 2506Hz | 467 |
| E4 | 16.0µs | 3.9µs | 8.8µs | 6.7µs | 11.1µs | 22.3kHz | 3072Hz | 592 |

- **D1**: 无持续黑洞; SEG_CMD 仅单发 3.6ms 峰值 (E2/E3/E4), avg 全 <32µs。
- **D2**: 不成立 (主循环 8-9kHz 无空洞)。
- **D3**: 证伪 — TIM1_ISR E0(静) 9.61µs = E3(转) 9.62µs, ISR 成本与转速无关是数学事实;
  此前 8-14×"暴涨"是测量伪影 (N 帧抑制 + 历史残留)。
- **D4**: 证伪 — SEG_PREF 4.5µs/条 vs 纸面 0.3ms/条 (高估 660×), 20Hz 仅 0.0002% CPU。
- **真凶**: PDBBIN gate ÷13 与主循环率 comb 漂移 → E4 3072Hz (设计 200Hz),
  TX 链路 P0/P1 仲裁饱和 → gap 3→592。CPU 侧完全无辜, "压制"是 TX 现象。

### Gates
- G1 探针自成本 <0.1%: PASS (~0.01% CPU)
- G2 三条挂账归因/证伪: PASS (D3/D4 证伪 + D1 定位 + 真凶 PDBBIN 超载)
- G3 探针关 verify_low_speed PASS: LOOP_PROF_EN=0 编译通过零行为差, 复跑待下次台架

### 遗留
- PDBBIN 率修复建议: gate 从固定 ÷13 改主循环率自适应 (目标 200Hz) — 挂账下轮。
- F1 已知发现: TIM1 抢占 UART TX 完成中断 ≤15µs 节流空隙 (TX 带宽上限视角)。

## [2026-08-31] TX 泵效率专项（T0 三数判别 + T3.1 时间门控，PDBBIN 200Hz 全工况解耦）

### Problem / Task
- 遥测带宽瓶颈定位: 判决实验 212Hz vs 黑洞专项 E0 691Hz 矛盾数据 (同固件系)。
  Kimi 任务卡: T0 三数判别 (零烧录) + T1 代码核查 + T3 修法池 (时间门控必做)。

### T0/T1 结论
- **L1 准入丢弃排除**: tx_p0/p1/p2_drop=0 (前后差分)。
- **L2 泵未满转排除**: 固件尝试 9666Hz / 成功 690Hz, 主机 rx=690Hz+gap=0。
- **L3 主机/链路排除**: 1M 下有效载荷 8.4KB/s << 100KB/s。
- **根因**: PDBBIN 门控 `÷13` 计数器与主循环率耦合 (为 2.5kHz 时代设计);
  探针固件主循环 9.7kHz → 尝试率 9666Hz → 发射率 690Hz。0fa8b17 的 212Hz 是主循环
  2.8kHz 所致 — 两版门控率不同源于主循环率不同, 非 bug。

### T3.1 实现 (时间门控)
- `UART_CommandServicePosdbg` 门控改 `(s_foc_tick_2khz - s_posdbg_last_tick) < 10U`
  (5ms 硬时间基准, tick 在 2kHz 速度环拍递增) — 替代 `s_posdbg_acc < 13` 计数器。
- compile clean (Keil 0 Error / 1 Warning), 烧录 Verify OK。

### Gates (台架实测)
- **G1** 200Hz 全工况解耦: 纯流 202.0Hz / 全组合 202.4Hz (+1%), 主循环 9.7k→32k Hz → PASS
- **G2** 纯流 ±5% + drop=0: 202.0Hz (+1%), tx_drop=0 → PASS
- **G3** 全组合 ≥190Hz + gap=0: 202.4Hz, gap=11 → 部分 (gap 已隔离到 F1 N帧仲裁; 无 N帧时 gap=0)
- **G4** verify_low_speed PASS: 稳态 pp=0.04°, 斜坡 95.5%, 阶跃 88.3% → PASS
  (第一轮 pp=6.31° 为 fault=7 ADC 偶发污染, CLEAR_FAULT 后恢复)

### 遗留
- F1 (TIM1 抢占 UART TX) 是 G3 gap=11 的投影; 扩容需 C7 风险分析后实施。
- 波特率 2M 未做 (L3 未坐实, 无扩容动机)。
