# PROGRESS

已并入 [PROCESS.md](PROCESS.md)。

从 `2026-04-05 20:10` 起，`PROCESS.md` 是唯一主日志；本文件仅保留为兼容入口，避免后续台架调试继续分叉记录。

## [2026-06-11] 前馈系统全面补齐（P0-P4）

- **范围**: 基于 `FOC_前馈分析_20260610.md`，补齐5条缺失前馈路径
- **实施项**:
  - **P1 BEMF解耦**: `foc_core.c` 电流环新增 `Vd_ff=-ωLq·Iq, Vq_ff=+ω(Ld·Id+Ke)`，`FOC_Handle_t` 新增 bemf 字段
  - **P2 惯量前馈**: `foc_app.c` SpeedLoop 新增 `Iq_ff=J·α/Kt`；`MI_IdentifyJ()` 从 stub 重写为恒电流加速+滑行法（同时识别 J 和 B）
  - **P3 库仑+粘滞摩擦**: `MotorParam_t` 新增 `Tc` 字段；SpeedLoop 新增 `sign(ω)·Tc/Kt + B·ω/Kt` 前馈；`PARAM_VERSION` → `0x00010009`
  - **P0 齿槽LUT**: 新增 `FOC_CoggingLUT_t`（264 float）、运行时线性插值查表、`MI_STATE_COGGING_IDENTIFY` 识别步骤（开环慢速拖动+分bin记录）、Flash LUT 存储（CRC32+magic 校验）
  - **P4 负载转矩观测器**: Gopinath 型降维观测器，默认关闭（`FOC_FF_ENABLE_OBSERVER=0`），待调参后开启
- **编译验证**: `build.ps1` GCC 通过，0 错误 0 警告
- **关键文件变更**: `foc_core.h/c`, `foc_app.h/c`, `motor_identify.h/c`, `param_storage.h/c`
- **参考**: `C:\Users\xiangyu\Desktop\FOC_前馈分析_20260610.md`
- **计划文件**: `C:\Users\xiangyu\.claude\plans\c-users-xiangyu-desktop-foc-20260610-md-playful-bachman.md`

## [2026-04-16 00:53] 位置环PD改造兼容记录
- Problem: 本仓库的正式日志已迁到 `PROCESS.md`，但这次位置环从 `PI` 语义切到显式 `PD` 属于重要控制架构变更，需要在兼容入口保留可追溯记录，避免后续只看 `PROGRESS.md` 的人漏掉这次调整。
- Resolution: 详细变更已写入 `PROCESS.md`，内容涵盖固件位置环 `PD` 公式、`CMD:PD_POS`、Host GUI 与文档同步更新，以及回归测试与构建验证结果；本条用于在兼容入口标记该事件和对应主提交。
- Prevention: 继续以 `PROCESS.md` 为唯一主日志，但遇到控制架构级变更时，在 `PROGRESS.md` 至少追加一条镜像记录，确保兼容入口不会静默失真。
- Commit: 09ad81d961e1e3b808360e38a6456b38fee74f95
- Recurrence policy: Not allowed to happen again.
