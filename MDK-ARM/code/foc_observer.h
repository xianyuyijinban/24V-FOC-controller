#ifndef FOC_SPEED_OBSERVER_H
#define FOC_SPEED_OBSERVER_H

#include <stdint.h>

/* 低速速度观测器 (线性 ESO / 增广 Luenberger)
 *
 * 目标: 解决 0.1°/s 级极低速速度估计的编码器量化噪声问题。
 * 直接位置差分: 15-bit 编码器 0.011°/LSB, 速度环 2kHz 下每周期仅移动
 * 0.00005°(0.005 LSB), 差分原始噪声 ±22°/s, 20Hz LPF 后仍 ±0.5°/s,
 * 淹没 0.1°/s 信号。观测器用动力学模型积分速度, 位置仅慢速校正。
 *
 * 状态: [theta_hat, omega_hat, T_hat]   (全部机械帧)
 * 模型: d(theta)/dt = omega
 *       d(omega)/dt = (Kt*Iq_mech - B*omega - T)/J
 *       d(T)/dt     = 0        (慢变聚合扰动/摩擦, 阻碍运动为正)
 * 测量: theta_mech; 校正增益按三重极点 -w0 配置:
 *       L1 = 3w0 - B/J
 *       L2 = 3w0^2 - L1*B/J
 *       L3 = -J*w0^3
 *
 * 低速时位置校正每周期 <<1 LSB, omega_hat 由模型积分主导,
 * 位置仅在跨越 LSB 时轻校正 → 平滑速度。w0 为核心调参:
 * 上限受量化噪声 (sigma_omega ≈ sigma_q * w0), 下限须覆盖位置更新率
 * (0.1°/s -> 0.11s/LSB -> w0 ≳ 10 rad/s)。建议 10~15, 初值 12。
 */
typedef struct {
    /* 模型参数 */
    float J_hat;      /* 估计惯量 kg·m^2 (须用辨识值) */
    float B_hat;      /* 估计粘滞 N·m·s/rad */
    float Kt;         /* 转矩常数 N·m/A (机械侧, = Ke) */
    float w0;         /* 观测器带宽 rad/s */

    /* 状态 (机械帧) */
    float theta_hat;  /* 估计位置 rad */
    float omega_hat;  /* 估计速度 rad/s (原始, 诊断用) */
    float omega_lpf;  /* 估计速度 rad/s (低通平滑, 控制用) */
    float omega_prev; /* 上一拍 omega_lpf (DOB 加速度计算) */
    float T_hat;      /* 估计扰动/摩擦 N·m */
    float T_hat_lpf;  /* 扰动平滑输出 (前馈用) N·m */

    /* T 学习增益 (L3 放大倍数). 极点配置的 L3=-J·w0³·Ts 极小, 低速位置量化
     * 残差零均值随机游走 → T_hat 学不到摩擦(恒 0). 放大 t_gain 让 T 快速
     * 响应位置误差的系统性分量, 配 T_hat LPF 滤量化噪声. */
    float t_gain;

    /* 扰动限幅 (防发散/抖动) */
    float T_max;      /* N·m */
    float T_rate;     /* N·m/s 限速率 */

    uint8_t valid;    /* 已初始化 */
} FOC_SpeedObserver_t;

void FOC_SpeedObserver_Init(FOC_SpeedObserver_t *obs,
                            float J, float B, float Kt, float w0, float Ts);
void FOC_SpeedObserver_Reset(FOC_SpeedObserver_t *obs, float theta_mech);
void FOC_SpeedObserver_Update(FOC_SpeedObserver_t *obs,
                              float theta_mech, float iq_mech, float Ts);

#endif /* FOC_SPEED_OBSERVER_H */
