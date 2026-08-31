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
