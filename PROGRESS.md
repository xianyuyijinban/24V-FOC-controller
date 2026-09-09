# PROGRESS

## [2026-09-09] AI 链路协议增量 ②: EVT 事件帧 — 完成闭环（含 ①纯流脚本补账）

- 落码 08126aa: 固件 type 0x22 (DBG_TYPE_EVT) payload 13B = tick4(2kHz 同 PDBBIN
  基准) + code1 + payload8; 帧 18B。事件表: 0x01 state 迁移 / 0x02 fault set/clear /
  0x03 ESC 触发退出 / 0x04 AW 模式切换 / 0x05 tx_p1_drop 变化。同类 100ms 限速
  (200 拍), 被丢次数饱和 255 回填下一同类帧 payload[7]。P0 优先级, 帧边界原子准入。
- 接线架构 (铁律 1): 0x01/02/03/05 走 2kHz 拍尾部采样对比 (EVT_SampleControlState,
  it.c), 事件源全是 g_foc_app 已有字段, 不穿透 foc_app.c 分层; 0x04 命令驱动
  (POS_AW_MODE 处); ESC 触发帧带最近位置环 err (滞后 ≤5ms)。
- 解析端: foclink TYPE_EVT=0x22 分流, EvtEvent.decoded() 语义解码; 单测
  test_foclink_evt 4/4 (5 类解码/溢出槽/PDB 共存 seq 连续/CRC+len 防护)。
- 台架验收 (板=08126aa→ee6cacd, evt_bench_20260909_194441.json 原文数值):
  - 0x04 风暴 (12 连发 @20ms): 发出 3 帧 (storm_sent=3), 溢出丢 8
    (storm_overflow_sum=8), 生效切换 11 ≤ 上限 11 — 限速守恒;
    storm_events overflow = [0, 4, 4]。
  - 0x01: state_events=[{tick=1315046, old=3(READY), new=4(RUNNING)}] —
    ENABLE→RUNNING 迁移帧到。
  - 0x03: esc_events 2 帧 — trigger tick=1334881 err_rad=-0.06921 (-3.97°,
    过 3° 线), exit tick=1337481 err_rad=-0.02281 (-1.31°, 落 1.5° 回差内);
    卡滞点在 126.43° 起点 6° 阶跃 (角度依赖, 与 ②⑦ 同源)。
  - C/E: 事件 tick 单调 (evt_monotonic=true) 且落在 PDB tick 范围
    (1330251..1354885) 内 — 同基准可对齐; PDB rx=1264 crc_err=0 seq_gap=1
    (≤2 容忍带), EVT rx=7 crc_err=0; 全窗 tx_p1_drop_delta=0 (gap 归因齐)。
- ①收尾补账 (用户指令): 纯流脚本升级 pdbv2_pure_stream.py (scripts/low_speed/)
  加 tx_p1_drop_delta (scope 头尾差, 无符号回绕安全; 尾查询关流后测 —
  220Hz 流下单发 P0 查询会丢, 实证 3 连丢); 重跑 60s 全绿
  (pdbv2_pure_stream_20260909_193507.json 原文数值): frames=12005 rate_hz=200.1
  crc_err=0 seq_gap=0 tx_p1_drop_delta=0 tick_bad=0; v1 段 frames=1000
  crc_err=0 seq_gap=0 tx_p1_drop_delta=0。
- ①镜像缺陷修复 ee6cacd (②验收时暴露): 静止帧 ff_coulomb 残留 -0.022
  (上次锁存值) 而 ff_total≈0 — coulomb_dir=0 拍 Coulomb 分支不执行, 镜像缺
  归零路径; 修复后纯流重跑 12005 帧全 0 自洽。
- 执行发现两笔: (a) 大角度起点 (130-132°) 6° 阶跃不卡滞 — ESC 触发验收
  须回位 126° 卡滞点, 角度依赖再次实证; (b) 风暴 12 连发 228B 贴 RX 环
  256B 上限, 个别命令可被环覆盖丢失 (UART_RX? err=0 不计环覆盖 — 盲区),
  守恒口径放宽为 ≤11 上限。

## [2026-09-09] AI 链路协议增量 ①: PDBBIN v2 帧 — 完成闭环

- 落码 70aeb70: 固件 type 0x21 (DBG_TYPE_PDB2V2) payload 49B = v1 37B 前缀逐比特一致
  + 尾部 3×float (ff_coulomb/ff_cogging/pos_integral); `CMD:PDBBIN,2` 开 v2, `,1` v1
  逐比特不变, `,?` 报版本。解析端 foclink TYPE_PDB2V2 分流, ladder/verify 增
  `--pdbver` (默认 1), meta 增 `pdbbin_ver`, validator 容忍新旧。
- 取数口径 (核实后): ff_coulomb = Coulomb 段 coulomb×smooth 生效值 (FFDiag.coulomb_iq
  最小只读镜像, 赋值两处不动逻辑); ff_cogging = ff_diag.cogging_iq (已有);
  pos_integral = pos_ki_out_prev 饱和后 ki_out (已有, 任务卡"static 局部变量"担忧不成立)。
- 单测: test_foclink_pdbv2 5/5 (v2 解析/边界值/负值 + v1 回归 + 混合流 + CRC/len 防护);
  validator 19/19 (增 pdbbin_ver=2 过/ver=3 FAIL/缺省过 3 例)。
- 台架验收 (板=70aeb70, 断言式烧录, FW_INFO 1.5.0 alive):
  - 纯流 60s (scripts/low_speed/pdbv2_pure_stream_20260909_170414.json): 12004 帧 200.1Hz,
    CRC=0, seq gap=0, tick 间隔 9/10/11 全在 drain 抖动带内; v1 5s 回归流照常。
  - v2 运动轮 (verify_lowspeed_20260909_170937.json, --pdbver 2, validator OK):
    ff_coulomb_absmax=0.022 (comp 定版值精确复现), ff_cogging=0 (COG 关照实),
    pos_integral_absmax=0.0134A (阶跃充电量级合理); health gap=0 CRC=0 delta=0。
  - v1 回归 (verify_lowspeed_20260909_171210.json, --pdbver 1, validator OK):
    阶跃 2.8s=100.8% (98-101% 历史带内) — "逐比特不变"非嘴上说的。
- 文档: docs/UART_COMMANDS.md PDBBIN v2 帧格式 + 命令表同步。

## [2026-09-09] 挂账：门控死区诊断任务卡 + AI 链路协议增量设计案

### 门控死区（2°/s 斜坡过冲 125-143%）— 诊断先行，规格待定
- 两贡献者未分账: 8e796ea latch 反号释放（滑跳段 comp 掉零→滞后→补课冲, 主贡献）+
  死区满额 comp（2°/s=0.035rad/s 落 0.06 死区, 该吃动摩擦 0.2× 却吃静满额, 次贡献 +5~10 点）。
- 任务卡（同一 B 序列: 2°/s 斜坡 + 6° 阶跃, 三臂对照）:
  1. 臂A 现状基线（定版参数不动, comp 0.022 + 死区 0.06）
  2. 臂B 临时 comp 减半（CMD:FRIC_COMP,0.011）→ track 大降 ⇒ 满额 comp 主因
  3. 臂C 临时死区旁路烧一档（FOC_FRIC_VDEAD_RADPS=0）→ track 回落 124-133% 区间 ⇒ 死区贡献定量
- 顺手项: G3 分段标定——诊断序列多记几档"交付-破壁"对, 摩擦账本从单点升级分段。
- 判据出口: 分账完成后定指令运动门控规格（ref 步进锁存"指令在动"标志, 动则绕过死区
  恢复 Stribeck 衰减; hold 窗 50tick > 0.2s 步进间隔可复用）——锁定区改动, 规格齐后
  需岳翔宇确认再落码。
- 执行前提: 修后脚本 + validator OK 为每臂有效前置（同 ②验证链标准）。

### AI 链路协议增量设计案（docs/plans/2026-09-09-ai-link-protocol-increments.md）
- 结论: 不重写协议, 三件增量 —— ①PDBBIN v2 帧（ff 分解+积分通道, 49B）②EVT 事件帧
  （state 迁移/fault/ESC, 14B）③TRIG 故障触发 ring buffer（2kHz 原速率验尸, 28KB RAM）。
- 不提频（CH340N 1Mbps 不动, 主机 RX 抖动证据链）; 不删文本命令通道（fail-closed 身份链）。
- 状态: 设计案待批, DeepSeek 未落码; 三件各自独立可单件批准。

## [2026-09-09 补] 版本更替 v1.4.0 → v1.5.0（②POS_AW_ESC 结案里程碑）

- 触发: ②验证链全绿（⑦ ESC ON 验收 + ⑧ 电流回归 esc_count=0 ×10 reps）,
  POS_AW_ESC 以默认 OFF 定版入库, 功能里程碑升 minor 版本。
- 内容: FOC_FW_VERSION "1.4.0" → "1.5.0" (foc_app.h:165); 断言式烧录
  (子树断言 + UV4 Verify OK + FW_INFO alive 报 1.5.0); tag
  `v1.5.0-POS_AW_ESC_CASE_CLOSED`。
- 过程证据 JSON 随本笔入库 (14 份, 规则: 被台账引用或过程判别材料):
  9/6 电流回归 3 份 (214027/215057/215350, ①关门依据) + ladder 2 份
  (221808/222629, 昨晚参考轮) + 9/7 G3 判别 2 份 (145220 首跑 7 条 V1
  实证 / 151646 修复后重跑) + 9/8 gap 判别 3 份 (090952/091503 ESC ON
  两连 abort / 092103 no-ESC 判别 + 177° 锚定异常) + 9/9 ESC 首击与
  watch 口径演进 2 份 (012849 两轮全胜被 r2 gap 杀 / A 尝试 3 连
  014852/015635/020003 gap 环境噪声 → delta 规格出台的直接动因)。
- 注: 1.5.0 为**仪器与观测链强化版**——身份链 (raw/ack 证据)、gap 归因
  (tx_p1_drop 直接测量)、ESC 观测链 (flags bit16/count/active) 均为
  本版本新能力。

## [2026-09-06 → 09-09] 候选② POS_AW_ESC 全链闭环：锁定区落码 → ⑦ ESC ON 验收 → ⑧ 电流回归关门

### ②固件落码（65bb4f1，岳翔宇批准锁定区改动）
- 僵持积分逃逸状态机 (foc_app.c:1713)：触发 |err|>3° 同号持续 2s (200Hz tick) →
  暂停 AW1 回拉 + 放开积分门 (ki_out ±0.10A 限幅不动, pd_sat 冻结保留)；退出
  三条件 |err|<1.5° 回差 / err 翻号立即 / 10s 强制。默认 OFF，
  `CMD:POS_AW_ESC,1` (清零计数) / `,0` (立即退逃逸)；查询
  `POS_AW_ESC,OK,en=,active=,count=`；JDIAG v7 增 esc=/esc_n=；
  PDBBIN flags bit16 = esc_active 逐帧可见 (stm32h7xx_it.c:659)。
- 台账措辞（Kimi 定）：逃逸机制 = **积分驱动微幅棘轮慢爬 + 振动助破**，与 G3
  "坡道微动助破"同族；**不是"积分余量碾压摩擦线"**（破壁 iq 峰 0.044-0.048A
  指令口径 < G3 标定摩擦线 0.070A 交付口径）。

### ⑦ G2@126°×3 ESC ON 验收 — 通过（s2_gain_ladder_20260909_013624.json, validator OK）
- t95 从"永不"（G2 卡滞档历史全部 N/A）→ **4.43/4.53/4.48s**（8s 窗内破壁）；
  esc_count 0→1→2→3（每轮独立触发恰好一次）；esc_active 112-115 帧 (~1.5s)；
  僵持末 +0.98-1.45s（2s 计时含在阶跃头）；破壁后 max|err| 1.19-1.80°
  （G3 预演对照 0.94-0.98° 同档, 无 lurch）；极限环 pp 0.03°；回程 track
  81.7/123.6/127.5%（破壁后追赶形态, 锁存反号释放压住无振荡）。
- 执行链插曲（全数据带回）：092103 锚定 177.4°（转子被人为转动, 回位脚本
  /tmp/move_to_126.py 分段 ≤12° 回 126.5°）→ UnboundLocalError cog_ack
  （init 首试丢响应时重试打印崩, 修）→ 三连 seq_gap abort（见下）。

### gap 口径改朝换代：tx_p1_drop 直接测量（Kimi A 裁决, 07b4274 + a1e1e32）
- 原口径"容忍≤2 (F1 N 帧仲裁投影候选)"被 **CMD:UART_RX? 的 tx_p1_drop=0 证伪**：
  静默/纯流/运动负载下固件 TX 环 P1 丢帧始终为 0 → gap 全在**主机侧 RX 抖动**
  （CH340, PollingPeriod 已=0 无可修）, 数据本体 CRC=0 逐帧无损。
  0906 背景 0 帧 / 0909 背景 2-3 帧 per 40s 窗 — 环境噪声, "等环境好转"=
  把仪器有效性寄托给天气。
- 判定升级为测量：脚本每轮 scope 首尾取 tx_p1_drop 差值（无符号回绕安全）进
  health.tx_p1_drop_delta；validator 三分支（gap>2 时）：delta 缺失=FAIL（无证据
  fail-closed）/ **delta=0=WARN（主机侧 RX 损耗, 归因齐）** / delta>0=FAIL（固件
  真丢）。gap≤2 现状不动（loci 容忍）。单测 16/16。
- watch 守卫两修：run 级累计改逐轮 scope_loci（010815 实证三轮各 1 帧→累计 3
  被误杀）→ 再改 gap 不当场杀、判定完全交 validator（012849 实证：同相成簇与
  跨轮散布在 watch 层不可分, 强判=口径发明）。

### ⑧ 电流回归 ESC ON — 关门（惰性证明 esc_count=0 × 10 reps）
- **A (20°/0.5°/s ×2)**: verify_lowspeed_20260909_084215.json — track 90.7/90.5%
  （历史带 92.8-97.2% 下缘, 0909 背景 gap 4/6 帧 delta=0×2 归因齐 WARN）、
  step@2.8s 98.0/98.3%、稳态 pp 0.09/0.10°、esc_count=0×2 → **validator OK**。
- **B (6°/2°/s ×2)**: verify_lowspeed_20260909_084549.json — **validator OK 零
  WARN**：step@2.8s 101.4/98.6%、稳态 pp 0.02-0.03°、gap 1/0、esc_count=0×2。
- ②定版姿态（⑧关门后生效, Kimi 裁决）：**默认值维持 OFF**。定版电流模式不需要
  它（10 reps esc_count=0 证惰性且不需要）；S2 电压模式仍是实验态, 逃逸是它的
  诊断工具不默认绑死。

### 仪器链修一笔账（9/6-9/9 累计）
- f7122e1 ladder meta raw/ack 证据链（9/5 起 8 份 ladder JSON 身份链不完整,
  G3 首跑 7 条 V1 实证; validator COG_CFG 前缀 OK→gain= 对齐固件 uart_upload.c:1743）
- 6c47b3f flash_deploy `${Commit}` PS 5.1 语法（F 笔血缘）
- d6bc952 ESC 观测链（ladder --esc 断言 count>0 / verify --esc 断言 count==0
  方向相反; flags bit16 逐帧; finally 兜底 POS_AW_ESC,0）
- a679d8c verify 补 parse_status_fields（ESC count 解析 NameError）
- watch 预检重试：verify FW_INFO/JDIAG/CH_CFG 带重试（上 run 遗留 PDB 流活跃时
  单发被挤掉, 台架两连 FAIL 实证）

### 挂账不变（三件）
- 门控死区（死区 2°/s 过冲副作用 133-142%, 指令速率门控候选, 锁定区）
- 18:43 环死案
- AI 协议增量三件

## [2026-09-06 凌晨] 电流回归首轮（仪器四修前, 参考值不作关门依据）+ 死区 2°/s 副作用

### 首轮数据 (verify_lowspeed_20260906_005105.json / 005248.json, 旧脚本产物)
- **A (20°/0.5°/s ×2)**: 斜坡终点 99.0/100.0%、跟踪率 97.5/95.6% — 贴定版 98.8% 区间;
  稳态真值 pp 0.09/0.02° resid 0.19/-0.003° (gate TIMEOUT 是窗语义 bug, 真值由 collect 窗算出)
- **B (6°/2°/s ×2)**: 阶跃 2.8s 到位 101.6/100.4% ✓; **斜坡跟踪率 144.2/162.4% —
  死区 2°/s 过冲副作用 (对比今晨 8e796ea 无死区 133/124%)**。
  机制: 2°/s=0.035 rad/s 落进 0.06 死区 → 全程满额静摩擦 comp, 真实慢动该吃
  Stribeck 地板 0.2×。"静止微振"(≤0.03) 与"真实慢动"(0.035) 频段重叠, 任何
  死区数值都分不开。固件候选: 指令速率门控死区 (|dpos_ref/dt|>阈值→绕过死区),
  锁定区改动, Phase 2 数据齐后议。
- **validator 抓 4 类**: schema 假失败 (validator 自身 bug, dispatch 顺序)、
  seq_gap=1 (A rep1 单帧, 位置待查)、sample_rate 21-24Hz (N 共存, 阈值 mode-aware)、
  gate TIMEOUT ×4 (窗语义实现偏离计划原文)。
- **仪器四修 (Kimi 裁决)**: ① validator schema-aware dispatch + 真实 JSON fixture
  ② rate floor 双档 (N 共存 ≥18Hz / 纯流 ≥150Hz, meta.n_coexist) ③ wait_stable
  回滑动样本语义 (span≥gate_window 硬判 + 大摆滑出) ④ seq_gap 容忍前提=位置日志
  (gap_loci 进 JSON, validator 强制校验) — 归因材料先于容忍度。
- **①未关门**: 修后仪器重跑 A×2+B×2 → validator 全 OK (含 gap_loci) 才关门。
- 台架顺序不变: F 笔断言式烧录 → 重跑 → ①关门 → ② S2 G2 @126°+160°。

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
- **P2 实验**: scripts/low_speed/loop_prof_exp.py (E0-E4×30s, 落盘 JSON + PDBBIN 三元组)。
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
- **L2 泵未满转排除**: 主机 rx=690Hz+gap=0; 主循环迭代率 9666Hz, 固件成功发射率 ≈690Hz。
- **L3 主机/链路排除**: 1M 下有效载荷 8.4KB/s << 100KB/s。
- **根因**: PDBBIN 门控 `÷13` 计数器与主循环率耦合 (为 2.5kHz 时代设计);
  探针固件主循环迭代率 9666Hz → ÷13 ≈ 743Hz 尝试率 ≈ E0 实测 690Hz。
  0fa8b17 的 212Hz 是其主循环 2.8kHz ÷13 ≈ 215Hz 所致 — 两版门控率不同源于
  主循环率不同, 非 bug。两版主循环率差 3.4× 的具体机制未查(时间门控解耦后已无害)。

### T3.1 实现 (时间门控)
- `UART_CommandServicePosdbg` 门控改 `(s_foc_tick_2khz - s_posdbg_last_tick) < 10U`
  (5ms 硬时间基准, tick 在 2kHz 速度环拍递增) — 替代 `s_posdbg_acc < 13` 计数器。
- compile clean (Keil 0 Error / 1 Warning), 烧录 Verify OK。

### Gates (台架实测)
- **G1** 200Hz 全工况解耦: 纯流 202.0Hz / 全组合 202.4Hz (+1%), 主循环 9.7k→32k Hz → PASS
- **G2** 纯流 ±5% + drop=0: 202.0Hz (+1%), tx_drop=0 → PASS
- **G3** 全组合 ≥190Hz + gap=0: 202.4Hz, gap=11 → 部分 (gap 已隔离到 F1 N帧仲裁; 无 N帧时 gap=0)
- **G4** verify_low_speed PASS (判据): 稳态 pp=0.04°, 斜坡 95.5%, 阶跃 88.3%。
  **判据声明**: 检查点与历史同判据 (阶跃 0.5/1.0/2.0s, 斜坡 ramp+0.5/1.5/2.5s, 50Hz N帧)。
  **数据对照 (阶跃 2.0s 到位%)**: 08-30 11:24 = 97.0%, 08-30 17:15 = 97.8%, 08-31 21:31 = 101.7%,
  08-31 21:33 = 88.3%, 09-01 = 113.0% — **同固件五值散布 ±12pp, 为批间方差, 非 9pp 系统性回退**
  (2026-09-01 修正: 原"同判据 9+pp 真实回退"声明取消; 候选: 摩擦切入/出切批间状态差, 未隔离)。
  **稳态 pp 全表 (09-01 新立)**: 08-30 定版 0.02/0.02/0.03/0.03/0.03° (≤0.04° 干净态),
  但 08-30 15:40=3.53°, 17:15=5.62° (≥3° 漂移态), 08-31 21:31=6.31°/21:33=0.04°, 09-01=6.24° —
  **间歇性双态 (漂移态≥3° vs 干净态≤0.04°), 同固件同日内切换** (08-30 五连 0.02/3.53/0.03/0.03/5.62;
  verify_lowspeed_*.json 21 轮全表)。**漂移先于 0fa8b17/1d898ce 两笔提交存在** (08-30 17:15 烧 afb2eb3,
  更早 08-20 有 3-5° 高发 4.92/4.18/2.99)。**双态触发条件比漂移幅值更值得查** —
  候选: 位置触发 (齿槽/摩擦平衡点, cog_phase=60° 偏最优 9°), 摩擦切入/出切批间状态差, 温度。
- **G3 尾差 (LOOP_PROF_EN 实际=1)**: 仍开着 (板载 probe_en=1, 未在 =0 构建上跑 verify)。

### 遗留
- F1 (TIM1 抢占 UART TX) 是 G3 gap=11 的投影; **C7 风险分析已交付 (7.5 附录): 2M 波特率推荐/提级否/时机不急**。
- 波特率 2M 未做 (L3 未坐实, 无扩容动机; C7 判定等 N帧+PDBBIN 双开满速需求再动)。

## [2026-09-01] 尾差复跑 (任务1) + 观测器/Ke 调查 (任务2)

### 任务1: verify_low_speed 复跑 (同固件 1d898ce, 第三轮)
- **预检**: JDIAG 审计全绿 (compiled LUT min=-0.0093/max=0.0133, enc=-1, cog_gain=0.0, J=5.294e-5)。
  注: **板载固件实际 LOOP_PROF_EN=1** (CMD:LOOP_PROF? probe_en=1), 任务卡"当前构建 LOOP_PROF_EN=0"前提与实况不符;
  G3"探针关闭 verify 闭环"因此未真正验证过——6.24° 与探针无关 (探针是诊断探针), 且 6° 级仅 08-31 后出现
  (08-30 定版是 0.02/0.03°, 见稳态全表; "08-30 无探针固件时已出现"说法无凭据)。
- **结果**: 阶跃 2.0s=113.0% 到位 (pp=0.13°), 斜坡 5.5s=105.0% (pp=0.05°), **稳态 2s pp=6.24°**。
- **"88.3% 回归"叙事修正**: 同固件阶跃三值 88.3/101.7/113.0 → 批间 ±12pp 离散, 非 9pp 系统回退。
  修正在 PROGRESS 下 TX 泵 G4 声明 (候选: 摩擦切入/出切批间状态差, 未隔离)。
- **稳态 6.24° → 间歇性双态 (修正)**: 三次复跑 6.31°/0.04°/6.24° — **不是"稳态保持失败"是间歇性/双态稳态漂移**。
  **历史全表 (21 轮)**: 08-30 定版全 ≤0.04°, 但 15:40=3.53°/17:15=5.62° (3° 级 08-30 首次出现),
  08-31 21:31=6.31°/21:33=0.04°, 09-01=6.24° — **双态同固件同日内切换**。
  **08-30 漂移先于 0fa8b17/1d898ce 存在** (17:15 烧 afb2eb3, 更早 08-20 有 3-5° 高发 4.92/4.18/2.99)。
  **双态触发条件比漂移幅值更值得查** (候选: 位置触发 齿槽/摩擦平衡点 cog_phase=60° 偏最优 9°,
  摩擦切入/出切批间状态差, 温度)。
- 台架已回安全态 (OFF/STOP/CLEAR_FAULT/POS_DIRECT,0), JDIAG 确认无故障。

### 任务2: 观测器/Ke 调查 (零烧录, 代码+数据盘点)
- **速度反馈量化分析 (保留但降优先级)**: 当前走**编码器差分** (foc_app.c:878-902), 0.1°/s 时 δθ≈0.056 LSB/拍,
  量化抖动真实存在 (0.1°/s 跟随早达标 err_std 0.01-0.06°, 量化非首要问题)。
  **——勿偷换目标: 当前最严重新问题 = 位置环稳态漂移 (间歇性双态 6° 级), 不是速度环量化噪声。**
  量化分析降为低速平滑候选, 优先级放位置环双态之后; 观测器接入 (obs_use_speed=1) 仍当低速平滑备选。
- **Ke 使用路径**: BEMF 前馈 (foc_core.c:656 vq_bemf=ωe·(Ld·Id+Ke)) **默认关闭** (bemf_user_enable=0);
  P4 扰动观测器 FOC_FF_ENABLE_OBSERVER=0 编译关闭; ESO 未接。
  **→ Ke 不在低速路径上, 低速爬行与 Ke 无关。** Ke 只影响高速 BEMF 前馈 (若开启)。
- **Ke 离线拟合: 数据源不成立** — speed_sweep JSON 无 Vd/Vq (只存 PDBBIN 聚合窗),
  N 帧有 Vq (uart_upload.c:410 p[21]) 但**无脚本存过原始 N 帧行**; bemf_regression (6月) 是低速静态非高速匀速点。
  **需补一次高速段采集 (N帧原始行 或 PWM_DIAG 高速扫描) 才能测 Ke。** 计划清单新增真实数据缺口。
- 报告: docs/observer_ke_investigation_20260901.md

### 下一步建议 (非本任务, 09-01 修正)
1. **位置环稳态双态调查 (新最高优先)**: 双态触发条件比漂移幅值更值得查。
   零烧录判决: 固定多角度各自测稳态 pp (verify_settle_angles.py, 角度扫描→是否位置触发)。
   若角度相关 → cog_phase=60° 偏最优 9° 嫌疑坐实 → 触发齿槽相位校准。
   若角度无关 → 转向摩擦切入/出切批间状态差 / 温度。
2. **观测器接入实验** (低速平滑主攻, 次优先): CMD:OBS_CFG,<w0>,use_d,use_speed 开启 obs_use_speed=1。
3. **Ke 精准测量** (高速 BEMF 前驱前置): 需一次高速段采集 (当前数据缺口)。
4. **A/B 回退 0fa8b17 vs 1d898ce**: 08-30 17:15 烧 afb2eb3 时漂移已存在 → 两笔非根因, 只在双态调查无果后跑。

## [2026-09-01 下午] 稳态漂移判别矩阵 E1-E4 (Kimi 任务卡, 18 点全判决)

### E1 角度矩阵 8 点 (PDBBIN 位置线, 修复后)
- **8/8 全到位零拒动; "165° 拒动" 撤销** (修复前 drain bug 假象: 陈旧帧 pp 虚高 30-45°)
- 200/240° "STUCK" 是脚本解环绕 bug (实际到位 199.99/240.01°)
- 8 点 pp 全 ≤0.187°, **无一 ≥3° 漂移态** — 本次扫描无一复现 6° 漂移
- **位置触发假设直接证伪** (8 角全干净, 240° 干净 0.055°), 与上午 240.69° 6.24° 冲突 → 非角度决定

### E2 AW 三模 + E3 COG A/B (240.7°/165°)
- **AW 模式不是漂移态触发器 (静止工况证伪, 序列工况未测)**: 三模全 CLEAN/MID; AW0 旧律 165° err=-2.26° 更差但非漂移触发器。
  **嫌疑缩小未出局**: 漂移态历史样本全在 verify 阶跃大摆→回位序列后, E2 静止 settle 工况未测序列历史;
  摩擦 position_friction_active 退出同理未测。
- **COG LUT 不是漂移态触发器** (ON/OFF 全 CLEAN 0.055-0.066°; 165° COG_ON 疑点重测确认时序假象, 实际到位 165.06°)
- **总量 18 点 (E1 8+E2/E3 10), max pp=0.220°, 漂移态 0 点**

### 核心发现
- **6° 级漂移仅出现在 verify_low_speed 稳态段 (回位+5s 后 2s 窗) —— E1-E4 纯 PREF 静止 (settle 5.5s+窗 2s) 无一复现**
- **残留假设**: 漂移态可能是 verify 特定序列 (阶跃大摆+回位+积分器残留) 触发, 而非静止预置位置触发;
  或极低概率事件 (温度/摩擦随机切换)。E4 层定无法完成 (当前无漂移复现, iq_cmd=0 无数据可判)。
- **待验证 (下次台架)**: 复现 verify 完整流程 (阶跃+回位+5s+测 2s) 在 240.69° 跑 4 轮看双态概率;
  若不复现 → 极低概率事件, 降优先级。
- 报告: docs/settle_discrim_matrix_20260901.md; 数据: settle_mat_*.json / settle_abc_*.json

## [2026-09-01 晚] 双态漂移收官判据 (12轮全净, 降级低概率事件, 主攻线关闭)

### 12 轮 verify 序列全净 (Kimi 收官判据: 4轮内复现→归因; 4轮全净→降级)
- **3 批 × 4 轮 = 12 轮完整 verify 序列 (阶跃+回位+5s+测2s) 全部 CLEAN (pp 0.033-0.077°), 无一漂移态**
- 序列与历史 6° 级漂移 (08-31 21:31=6.31°, 09-01 11:33=6.24°) 同序列同位置同配置
- 累计 38 数据点 (12轮 + E1-E4 18点 + E1 8点) 无一 ≥3°

### 状态字对照 (第二刀)
- DIR? 独立验证正常: dir=0/hold=0/integral=0.0374/iq_cmd=0.0156/fric=0.022 (无漂移态正常值)
- 12 轮主运行内 DIR? 没抓到 (PDBBIN 重开时 P0 被 P1 抢) — 但无漂移态可抓, 不影响降级判定
- 漂移态现形瞬间抓状态字未完成 (漂移态从不出现)

### 终审: 降级低概率事件
- 12 轮全净 × 38 点全净, 漂移态概率粗估 <10%/轮 (12轮全净 p=5%→54%, p=10%→28%)
- 但历史 2 次 6° 级证明概率 >0; 触发条件在现有命令面/单次台架内无法确定
- **判定: 低概率事件, 主攻线 (稳态漂移探因) 关闭** — 下次台架顺手记稳态 pp 就行
- 复现时动作: 抓 DIR? 状态字 (已证能抓) → 三分法归因 (振荡=AW阈值极限环/单向慢爬=推力不足/静止偏移=死区平衡)
- 报告: docs/drift_probe_final_20260901.md; 数据: drift_probe_*.json (3批12轮)

## [2026-09-04] 双态漂移案终审定案 — 积压帧混入测量窗的脚本伪影家族 (推翻 09-01 低概率结案)

### 定案 (Kimi 终审, 铁证亲自核过)
- **电机/控制链/观测链/磁体全部无罪** — "双态漂移"= 串口积压帧混入测量窗的伪影
- 定量否决: 轮0 PDBBIN 轨迹 theta 极值跨度 5.89° < 报告 pp 6.22° — pp 超轨迹上限, 窗外帧混入
- 机制: g4_anatomy3 sleep(2.0)+sleep(1.5) 期间无人排空 nframe_q, 测量窗第一批 pop 出 3.5s 前保持段/回位摆动旧帧
- 历史同构: verify_low_speed.py:226-230 (回位 PREF 后 sleep(5.0) 不排空) — 08-30 3.53/5.62°, 08-31 6.31°, A/B 5.03/6.22°, seq 8.77/5.39°, settle_gate 5.78-6.11° 全部同族
- "双态"随机性 = OS 串口缓冲区边界抽签, 不是电机状态

### 记录在案的错误 (两轮)
- 9/3 "0/20 全净决定性反转" — 那轮环死 18 轮, 根本没测到电机
- 9/4 "印证收敛抽签+热→Rs↑→电流弱" — 读的是积压帧; 且电流环调节电流本身, Rs 升不削弱 iq, 热力学论证方向不成立
- 9/3 稳定门 spec 漏 p2/p8 列 — 同记录

### 真实控制行为 (9/4 轮0 真正产出, 非伪影)
- 斜坡段: 粘滑爬行仍在 (滞后 -0.48° 滑跳 +1.23°/0.26s 过冲 +0.55°, 控制在 ±0.5° 级)
- 保持段: err +0.08° iq -0.011A — 好
- 回位段: 快摆 ~19°/s 到位, 积分受限慢爬 2.3s 到 err 0.29° 僵持, **最后 0.3° 还要 ~7s (真实稳态收敛 ~10s 级)** — 定版 trade-off 代价, 记文档暂不算 bug

### 环死案 (独立真实未决, 最高优先级)
- 18:43 轮0-1 活, 轮2-19 死: pos_err bit-exact 0.0 × 3420 帧 + iq_cmd 冻结 0.01154 + theta 冻结 — PositionLoop 体中段停走
- 审计: stall gate 排除 (STALL_MODE 从未发, 已识别); haptic 排除 (RAW); state/fault 无法排除 (当轮未记录) — **最可能 state 掉 RUNNING 或 control_mode 变化, 1636 gate 静默跳过**
- 当前固件 20 轮 state=4/fault=0 全程环活 → 无硬回归, 间歇事件
- **修复**: PDBBIN flags 接 (state<<8)|fault_code — 下次环死不再瞎 (已改, 待编译验证)

### 观测家族第五条 (进 skill): 积压帧混入测量窗
- 修法: foclink.MeasureWindow helper — 窗前排空+积压计数 / 按帧时间戳(host_rx)过滤 / 内置稳定门 (已实现)
- 所有低速脚本改用它; 判读增益阶梯/任何稳态数据必须用修复后测量窗, 否则又是一轮垃圾数据

## [2026-09-04] S2 增益阶梯 9 轮 + 环死代码审计 + 提交拆分 (Kimi 任务卡)

### 增益阶梯数据 (LADDER G1/G2/G3 × 3 轮, s2_gain_ladder_20260904_132438.json)
- 口径定义: 电压口径 V/rad = 电流口径 A/rad × Rs_phase(Rs/2≈4.4)。G1=0.245A/rad(1.016V/rad),
  G2=0.490(2.033), G3=0.735(3.050)。
- **Kimi 三处口径纠错**: ① G1 不是"卡死 1/3 轮"——3/3 轮斜坡全程没跟 (回位目标 a0 距僵持
  点 <1°, "回位"是语义幻觉); ② steady_resid_deg 基准 bug: 测量窗在回位 PREF a0 之后跑,
  目标=a0, 脚本却按 a0+6 减 → 9 轮全 -5.5~-6.0° 是设计输出不是故障 (已修); ③ t95 全军
  None 是协议退化: 阶跃从斜坡末尾发 + pos_err 从未进 ±0.1° → 该指标这轮等于没测。
- **序列修复** (s2_gain_ladder.py): 静止阶跃+6° (t95 有效) → 斜坡回程 12s @0.5°/s →
  稳定门 → 测量窗 (目标=a0, resid=mean(angle−a0))。
- **机理锁定**: G1 僵持 iq_cmd=-0.022A = kp 0.245 × 5.23° 精确; P+comp 0.044A 仍不动 →
  持矩位置相关 (0.022A 常数 comp 是平均), **非电压模式失败** (电流模式 kp=0.245 同点僵持)。
  P 死区预测命中 (5.15° 预测 vs 5.2-5.8° 实测; G2/G3 不僵持因 2.57°/1.72° 边界超持矩)。
  斜坡超前 142-169% 真实: 粘滑释放冲量 + comp 方向锁存保持窗 (err 过零后仍推老方向)。
  电流模式 kp=0.49: 死区边 2.57° vs 积分门 2° → 0.57° "P 不够、积分不许"死带 =
  10s 慢尾巴结构根源。
- **S2 对 60A 中期结论**: 电压伺服可行 (无环死、无 PWM 量化), 但低速率品质当前劣于
  电流模式定版。改善候选: ①comp 锁存 err 反号快速释放 (已改 8e796ea) ②持续僵持积分
  逃逸 (排队) ③COG LUT 精确标定启用 (一次只改一个)。

### 环死代码审计 (零台架)
- **stall 排除**: stall_open_loop_active 唯一置位路径 = FOC_App_Enable 的
  requires_stall_mode (!identified || !encoder && stall_mode_armed), 脚本从未发
  CMD:STALL_MODE → stall_mode_armed 恒 0。stall_open_loop_active 绝不置位。
- **haptic 排除**: app_mode=RAW → IsHapticMode=0。
- **最可能: state 掉 RUNNING 或 control_mode 变** — PositionLoop 门 (foc_app.c:1636)
  是唯一匹配 18:43 签名的路径: pos_err 不写 (读 0 冻结) + iq_cmd/theta 冻结。
  18:43 无 state/fault 记录 = 观测链缺陷 (当时没字段可查) → **修复 bdb7fb0**:
  PDBBIN flags 接 (state<<8)|fault_code, 下次环死首帧即自证。

### 提交拆分 (4 笔, 每笔独立可编译)
- **bdb7fb0** fix: VOLT_OFF 收拢 + PDBBIN flags 进帧 + Rs 口径注释 + N帧电压模式 Vq 修复
- **7e65b5a** fix: 速度环初始化块 + FOC_App_VoltageOff 收拢共用
- **eec5852** feat: S2 电压闭环位置伺服 — 电压模式位置环直通控制律
- **09d68da** data+scripts: 双态漂移案定案笔记 + MeasureWindow + 增益阶梯/解剖/环死脚本

### 遗留
- G2/G3 全到位 (resid 0.19-0.26°) 但斜坡超前待修 (8e796ea 已改未台架验证);
  极限环 ∝ kp (G1 0.03-0.10°, G2 0.04-0.42°, G3 0.02-0.45°)。

## [2026-09-05] 补偿锁存优先级重排 (8e796ea) + 脚本迁移 + Kimi 审查两 bug

### 8e796ea (foc_app.c, 1 文件 8+/3-)
- **根因**: 斜坡中 ref 每 0.2s 步进反复刷新 hold(50 tick=0.25s), 原④反号清
  (err·dir<0 → 清 0) 排在 hold 递减之后 → **整个斜坡期间④永不触发**。粘滑脱扣电机
  冲过 ref (err 反号) 后 comp 仍按老方向推满 hold 窗 → 喂过冲 = 142-169% 超前根因。
- **新优先级**: ①ref 步进刷新 → ②(err·dir)<0 → 立即释放 + hold=0 → ③hold 递减 → ④到位清 0。
- **风险论证 (Kimi)**: 反号持续时 comp 占空降为每步进 1 tick≈实质关闭; 粘住段 err 同号
  顶死区边 (G1 5.2°/G2 2.57°) ②永不触发 → 行为逐比特不变; 近零悬停偶发提前释放有界
  (≤0.2s 内重新锁存)。5Hz 间歇病不复活。
- Keil 编译 0 Error 0 Warning, foc_app.o/axf 重编验证。
- **台架验收 (门槛)**: ①G2 电压斜坡复测 track 142-169% → ~100±15%, 极限环不变
  ②G1 对照逐比特不变 ③**电流模式定版回归强制** (0.5°/s 40s 斜坡 + 6° 阶跃, 98.8% 级
  不退) — 不过回滚 8e796ea 重烧, 不过夜。

### verify_low_speed.py 迁移 (Kimi 定案: 回归只认迁移后数据)
- 稳态窗 → foclink.MeasureWindow (begin 排压 + collect 窗内帧 + wait_stable 门);
  斜坡/阶跃采集 → PDBBIN (200Hz 浮点 + seq/CRC8); watch_health 逐帧检查
  fault/state 掉 RUNNING/流静默 (18:43 案教训, flags 修复后可用); 阶跃检查点加 2.8s
  (对表"2.8s 内到 5.8° 级"); --reps 参数; 预清理加 VOLT_OFF; 轨迹列 (theta/poserr/
  ff_iq/iq_act)。
- s2_gain_ladder.py: --gains G1,G2 档位过滤 (默认全跑, 单测过)。

### Kimi 审查: 两必修 + 一非阻塞
- **Bug1 致命**: FOC_STATE_RUNNING=3 应为 **4** (枚举 foc_app.h:169-175:
  IDLE=0/INIT=1/PARAM_IDENTIFY=2/READY=3/RUNNING=4/FAULT=5; 3 是 READY)。
  it.c 只是打包工, 查错文件 — 不修第一帧就炸, 回归死路上。已修+注释写全值列表。
- **Bug2 安全**: finally 三条命令 (PDBBIN,0/STOP/CLEAR_FAULT) 缺 \n → 固件按行
  解析永远不收 → 脚本退出后电机保持使能 + 流继续。已修 (全脚本 17 处 write 扫过)。
- **非阻塞记档**: debug_stream.h:32 注释"高 8 位=state"应为**次 8 位 (bits 15:8)** —
  脚本按 it.c 解是对的, 头文件注释误导 (本次顺手改)。
- 通过项: ladder --gains diff 干净; 迁移主体全核过 (MeasureWindow 三板斧/PDBBIN 单源/
  检查点 2.8s/pre-clean VOLT_OFF/N帧 32 字段格式相容)。

### 台架执行规格 (钉死)
1. 烧 8e796ea (UV4 -b→-f → pyocd reset)
2. 回归 A: verify_low_speed.py --step 20 --ramp-deg-s 0.5 --reps 2 (98.8% 基准线)
3. 回归 B: verify_low_speed.py --reps 2 (6° 阶跃 2.8s→5.8° 判据)
4. A+B 全过 → s2_gain_ladder.py --gains G1,G2 → JSON 给 Kimi 裁决
5. 任意一步掉链 → 回滚 8e796ea, 本轮到此为止。

### 判读口径 (本轮特有)
- 反号释放生效后斜坡滑跳段 comp 主动掉零 = 设计行为 (ff_total 掉零只在 err 反号
  滑跳瞬间); **ff_total 在 err 同号时掉零成簇 = 逻辑 bug, 立刻停**。
- 电流模式回归若出现新 comp 间歇 (爬行): PDBBIN ff_total 轨迹会显示斜坡中段掉零。

## [2026-09-05 午后] 卡滞案定性 — 静态突破权限缺口（主障碍）+ 候选②僵持积分逃逸规格（待批）

### 卡滞案定性：权限缺口，常驻且固件无关
- **现象**: 静止发 6° 阶跃电机冲 1.2° 即卡，回落也卡，resid ~0.5-0.9° 僵持；
  ladder track G2 2-11%（9/4 晚改协议后才暴露：阶跃改从静止发）。
- **权限夹逼 (Kimi)**: 电压模式 G2 满额交付 ≈0.048A < 静摩擦 ≈0.056A（电流
  模式能走）。G3（1.5×增益）部分突破（126° 3-37% / 160° 24-54%）证明权限敏感。
- **三固件同卡**: 9/4 旧固件 / 09d68da 二分（113454 JSON）/ 8e796ea —— 8e796ea
  双证无罪（僵持 err 同号不触发反号释放 + ff 全程同向推注）。9/4 的 169%"到位"
  是旧协议（阶跃在斜坡末尾发、电机已在动）从没测过静止突破——缺口一直在。
- **协议混杂案认领**: 9/4 晚阶梯协议被改过（阶跃从斜坡末尾发→从静止发），判别
  实验设计时没锁住，Kimi 与 DeepSeek 各背一半。教训：判别实验必须声明协议版本。

### 判别实验链（三个假设被数据推翻）
- 位置相关假设：126° 也卡（track 2-5%）→ 推翻，且 9/4"126° 干净"基准不成立。
- 8e796ea 涉案假设：09d68da 二分（还原 8e796ea hunk）同卡 → 推翻。
- dtcomp 是墙假设：DT,1 @G2 160° track 6-11% 仍卡，轮1 gate 10.6s 张弛振荡，
  轮2 TIMEOUT 15s → 证伪。**后续所有 run 一律 DT OFF**（dtcomp 固件默认关，
  脚本不再发 CMD:DT,1）。

### comp 塌地板根因 + Stribeck 死区修复（半生效，未提交）
- **根因**: FOC_FRIC_STRIBECK_KINEMATIC=0.20 / VS=0.01（foc_app.h:84-85）；静止
  微振 v_mech mean 0.011-0.016、P95 0.027-0.048 rad/s > vs → smooth 钉 0.2 地板
  → comp 0.022→0.0044。不是观测器冻住，是真实微振/量化噪声过 Stribeck 膝点。
- **死区修复**: FOC_FRIC_VDEAD_RADPS=0.06（foc_app.h:86），v_eff 死区内=0（满额
  comp）出死区重锚连续（foc_app.c:1260-1262）。comp 恢复满额，但满额踢出死区→
  塌→回摆 = **边界张弛振荡**（121059 JSON：满额帧 v=0.053 / 地板帧 v=0.024）。
- 死区加宽 0.15 排队（②验证后单独烧，不混入②那笔）；风险：观测器游走幅度可能
  比差分速度大 2×，0.15 未必盖住。

### 交付损耗 72% = Rs 热漂（结构性，今日只记录）
- Rs 冷态 4.149Ω → 热态 ≈5.4Ω：0.19V 交付只能出 0.035A（实测 iq_act 0.033 吻合）。
- 电压开环无电流反馈校正。长期方向：在线 Rs 估计，或接受电压模式低速权限天花板。

### 候选②僵持积分逃逸规格（Kimi 已定，待岳翔宇确认锁定区后落码）
- **锁定区声明**: 动位置环积分/AW 结构（AGENTS.md 锁定区第 1 条）；防护 = 运行时
  开关 CMD:POS_AW_ESC **默认 0**，定版行为逐比特不变。先例：POS_AW_MODE、8e796ea。
- **触发**: |pos_error| > 3°（FOC_POS_AW_ESC_TRIG_RAD=0.0524）同号持续 2.0s
  （200Hz=400 tick）；timer=0 时锚定 err 符号，翻号或 |err|<3° → 清零。
  驳回"积分符号"触发：僵持时 AW1 以 3%/tick 抽积分（τ≈0.17s）钉在 0 附近，
  无可观测量。驳回 2°/1.5s：2°=AW 门本身，慢收敛期误触发。
- **动作**（foc_app.c:1711 前判状态，逃逸生效时）: 暂停整个 AW 回拉分支
  （1712-1731 跳过，aw_decay=0）+ 放开 1732 积分门（条件加 || esc_active；
  **pd_sat 冻结保留**）；ki_out 硬限幅 ±FOC_POS_INTEGRAL_LIMIT_A 不动。
  估算 err 5° 充电率 4.4e-4 rad·s/tick → 到限幅 ≈3.1s。
- **退出（任一）**: |err|<1.5°（回差 FOC_POS_AW_ESC_EXIT_RAD≈0.0262）/ err 翻号
  （立即退，错号积分交 AW1 回拉）/ 10s 强制（真卡死不许绕限幅常驻）。
- **可观测（必须带）**: pos_aw_esc_active_diag + pos_aw_esc_count_diag 进
  POSDBG/JDIAG；ladder 脚本断言 esc_count>0，否则"到位"无法归因（测试纪律 §3）。
- **skip_integral 核查**: 唯一置位 it.c:2556-2562（PREF 手动即时拍，随拍随清），
  僵持态恒 0，不干扰逃逸。
- **验证链（按序）**: ①提交脏树三笔（死区修复/脚本/文档，各可独立编译）→
  ②落②（一笔）→ ③烧录 + ladder G1-G3 @126°+@160° ESC ON（警戒：破壁后带电荷
  积分松手 lurch/过冲、极限环 pp、9/4 式斜坡过冲回潮——锁存反号释放应压住，
  压不住就停）→ ④电流回归 A×2+B×2 **ESC ON** 证惰性（预期 esc_count=0，离触发
  线 4 倍裕量）→ ⑤死区 0.15 单独烧 → ⑥电压模式正式 A/B 门。

### 流程挂账
- **阶段 1 已收口 (2026-09-06)**: A 笔 `7d7e97b` (foc_app.c/h + debug_stream.h:
  Stribeck 死区 + flags 注释纠正) + B 笔 `9fb9a74` (foclink/ladder/verify/validator
  + tests/×2 + uart_fw_info.py: 恢复规划阶段 1 全部)。板载 "Stribeckfix" 此前无
  SHA 身份 → 已由 7d7e97b 钉成可引用 SHA, **Phase 2 基线 = 7d7e97b** (**A 笔落定后
  重烧核对板上固件身份 = 7d7e97b 源码产物**)。C 笔 (文档) 在 9fb9a74 后落。
- 本地领先 origin **11 笔未推送**（origin 在 2b40950），建议近期推一次备份。
- 18:43 环死案：被动陷阱已布（flags=state<<8|fault 进帧 + watch_health 当场爆），
  等再现不主动追。

## [2026-09-05] S2 卡滞恢复规划 — 先收紧证据链，再决定锁定区改动

### Problem / Task
- S2 电压模式静止 6° 阶跃的当前结论被测试脚本缺陷削弱：最新 G2 数据
  `s2_gain_ladder_20260905_125522.json` 的 round 0 报 `t95=0.02s`，但同一 PDB
  轨迹随后从 `-0.01°` 走到 `-5.13°`；该首帧发生在阶跃命令真正生效前，不能作到位证据。
- `s2_gain_ladder.py` 仍发 `CMD:DT,1`，而 09-05 判别已经证伪 DT 能突破静摩擦，后续
  统一要求 DT OFF。脚本同时只打印 JDIAG、不 fail-closed，且没有逐帧 state/fault/
  PDBBIN 推进检查；产出的 run 身份也不完整。
- `foclink.MeasureWindow.wait_stable()` 目前也没有实现声明的“连续 2s 稳定”：它只保留
  最近 2s 的样本，但在累计 5 帧且 pp 合格时即返回，没有要求第一帧到最后一帧覆盖
  `gate_window`。PDBBIN 约 50Hz 时约 0.1s 即可放行；G2 round 0 的 `gate=0.10s`
  是该缺陷的实证，故现有稳态门也不构成有效稳态证据。

### Resolution
- 规划结论：S2 的已知主问题是静止突破权限不足，非 6° 稳态漂移。G2 的约 `0.19V`
  在热态相电阻下实测只能交付约 `0.033-0.035A`，低于约 `0.056A` 静摩擦门槛；G3
  在部分角度更高的跟踪率支持该物理解释。电流模式回归仍可作为产品基线。
- 阶段 1（仅测试脚本，DeepSeek 负责）：
  - 强制 `CMD:DT,0` 并在 JSON 记录确认响应；预检必须采集/解析 `SYS:FW_INFO?`、
    `JDIAG`、`CH_CFG`，记录固件、LUT min/max、cog_save、增益、模式和 AW 状态。任何
    必需项缺失即中止，不落有效结果。
  - 每帧检查 PDBBIN 序号/样本率推进、`flags=(state<<8)|fault`，要求
    `state=RUNNING`、`fault=0`；流静默、掉帧阈值越界或稳定门超时均标记该 run `invalid`
    并使进程失败，不能混入汇总。
  - 修复稳定门：判定 pp 前，滑动样本的 `last_rx - first_rx` 必须达到 `gate_window`；
    还须记录该覆盖时长、帧数、pp 和门前积压数。单测必须覆盖“5 帧但仅 0.1s”的假稳态。
  - 重定义阶跃指标：先检测误差确实离开初始窗，再判重新进入目标窗并保持一段 dwell。
    `t95` 使用 `|error| <= 5% * step`（6° 时 0.3°）；若保留 `±0.1°` 指标，命名为
    `t_settle_0p1_deg`，不得继续标为 t95。记录命令发送、阶跃可见、离开、到位的时刻。
  - 提供离线 JSON validator 和合成轨迹测试，覆盖旧帧、未发生阶跃、一次瞬时越界、
    state/fault、静默流、稳定门超时；在这些输入上必须 fail-closed。
- 阶段 2（单变量验证，脚本验收后）：冻结当前 Stribeck 死区候选
  `FOC_FRIC_VDEAD_RADPS=0.06` 的精确源码/构建身份，先跑电流模式回归，再以 DT OFF
  跑 S2 G2 静止阶跃，126° 与 160° 各多轮。每轮保留 PDB 原始/抽稀轨迹及
  `ff_total`、`iq_cmd`、`iq_act`、`v_mech`、state/fault；不同时改增益、LUT、死区或 AW。
- 阶段 3（仅当阶段 2 数据有效且仍需 S2 静止突破）：候选 `POS_AW_ESC` 会更改位置环
  积分/AW 结构，属于 AGENTS.md 锁定区。实现前需用户确认；实现应默认关闭、带 active/
  count 诊断，且先证明电流模式 `count=0` 无回归，再测试 S2 G2/G3。
- 产品方向：若目标是可靠的低速静止定位，当前证据支持电流模式作为正式路径；S2 保持
  实验模式，直至其在热态和多角度下证明具备足够突破权限和可接受的极限环。

### Prevention / Follow-up
- 后续只承认带完整 run 身份、协议版本、有效性状态和原始轨迹的台架结果。不得使用现有
  `t95=0.02s` 或任何 DT ON ladder 结果裁决控制算法。
- 固件候选落地必须先从当前脏工作树中拆出可回溯的单目的提交；当前板上
  "Stribeckfix" 没有可验证 SHA，不能作为 A/B 基线。

### Verification
- 静态审查：`scripts/low_speed/s2_gain_ladder.py` 确认仍有 `CMD:DT,1`、JDIAG 仅打印、
  t95 未经过离开初始窗判定且无 PDB health guard。
- 静态审查：`scripts/foclink.py:397` 确认稳定门未要求样本时间跨度达到
  `gate_window`；该问题解释 `s2_gain_ladder_20260905_125522.json` G2 round 0 的
  `gate_wait_s=0.10`。
- 数据核对：`s2_gain_ladder_20260905_125522.json` 的 G2 round 0 首个位置误差
  `-0.01°`，轨迹范围 `-5.13°..+0.61°`，与 `t95=0.02s` 相矛盾，故该指标无效。
- 已运行：`python -m py_compile scripts/foclink.py scripts/low_speed/s2_gain_ladder.py scripts/low_speed/verify_low_speed.py`；通过。
- 已运行：`git diff --check`；通过（仅 CRLF 提示）。未进行编译、烧录或台架测试。

### Commit
- Branch: `foc-public-0817`
- Commit: `b5de07ab4a4780720943cedfcb8556cc0a0060d3`
- Status: working tree changes not committed yet
- Files:
  - `PROGRESS.md`
  - `MDK-ARM/code/debug_stream.h`
  - `MDK-ARM/code/foc_app.c`
  - `MDK-ARM/code/foc_app.h`
  - `README.md`
  - `Project_Architecture.md`
  - `scripts/low_speed/s2_gain_ladder.py`
  - `scripts/low_speed/verify_low_speed.py`
