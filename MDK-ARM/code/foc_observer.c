#include "foc_observer.h"
#include "foc_core.h"   /* FOC_PI */

/* 扰动估计默认限幅: 摩擦启动电流 ~0.09A * Kt(0.129) ≈ 0.012 N·m,
 * 齿槽 ~0.005 N·m, 余量取 2.5x。 */
#ifndef FOC_OBSERVER_T_MAX_NM
#define FOC_OBSERVER_T_MAX_NM     0.03f
#endif
#ifndef FOC_OBSERVER_T_RATE_NMPS
#define FOC_OBSERVER_T_RATE_NMPS  0.3f   /* 扰动估计限速率 N·m/s (低速摩擦切换要跟得上) */
#endif
/* omega_hat 输出平滑 LPF 截止频率 Hz (滤位置量化校正尖峰; 太高滞后大) */
#ifndef FOC_OBSERVER_OUTPUT_LPF_HZ
#define FOC_OBSERVER_OUTPUT_LPF_HZ 5.0f
#endif
/* T_hat 输出平滑 LPF 截止频率 Hz (滤 t_gain 放大的位置量化噪声) */
#ifndef FOC_OBSERVER_T_LPF_HZ
#define FOC_OBSERVER_T_LPF_HZ 2.0f
#endif
/* 模型参数 fallback (观测器不应被无效参数击穿) */
#ifndef FOC_OBSERVER_J_FALLBACK
#define FOC_OBSERVER_J_FALLBACK   1e-4f
#endif
#ifndef FOC_OBSERVER_KT_FALLBACK
#define FOC_OBSERVER_KT_FALLBACK  0.129f
#endif

void FOC_SpeedObserver_Init(FOC_SpeedObserver_t *obs,
                            float J, float B, float Kt, float w0, float Ts)
{
    (void)Ts; /* Ts 通过 Update 传入, Init 不依赖 */

    if (obs == NULL) {
        return;
    }

    obs->J_hat = (J > 1e-6f) ? J : FOC_OBSERVER_J_FALLBACK;
    obs->B_hat = (B >= 0.0f) ? B : 0.0f;
    obs->Kt    = (Kt > 1e-6f) ? Kt : FOC_OBSERVER_KT_FALLBACK;
    obs->w0    = (w0 > 1.0f) ? w0 : 12.0f;

    obs->theta_hat = 0.0f;
    obs->omega_hat = 0.0f;
    obs->omega_lpf = 0.0f;
    obs->omega_prev = 0.0f;
    obs->T_hat     = 0.0f;
    obs->T_hat_lpf = 0.0f;
    obs->t_gain    = 1.0f;

    obs->T_max  = FOC_OBSERVER_T_MAX_NM;
    obs->T_rate = FOC_OBSERVER_T_RATE_NMPS;
    obs->valid  = 0U; /* 首次 Update 对齐位置 */
}

void FOC_SpeedObserver_Reset(FOC_SpeedObserver_t *obs, float theta_mech)
{
    if (obs == NULL) {
        return;
    }
    obs->theta_hat = theta_mech;
    obs->omega_hat = 0.0f;
    obs->omega_lpf = 0.0f;
    obs->omega_prev = 0.0f;
    obs->T_hat     = 0.0f;
    obs->T_hat_lpf = 0.0f;
    obs->valid     = 1U;
}

void FOC_SpeedObserver_Update(FOC_SpeedObserver_t *obs,
                              float theta_mech, float iq_mech, float Ts)
{
    if (obs == NULL) {
        return;
    }

    /* 首拍对齐位置, 避免历史未知位置产生大残差冲击 */
    if (obs->valid == 0U) {
        obs->theta_hat = theta_mech;
        obs->omega_hat = 0.0f;
        obs->T_hat     = 0.0f;
        obs->valid     = 1U;
        return;
    }

    const float w0 = obs->w0;
    const float J  = obs->J_hat;
    const float B  = obs->B_hat;
    const float Kt = obs->Kt;

    /* 预测 (forward Euler). 模型保留 B 阻尼项(物理正确), 但增益配置忽略
     * a=B/J: 低速电机 J 小(~1e-5)而 B 相对大, a 巨大会把 L2=3w0²-L1·a 推到
     * 天文数字致观测器发散(台架实测 omega_hat 爆到 1e5°/s)。低速 B 项
     * 本身可忽略(静摩擦由 T_hat 吸收)。 */
    float th_pred = obs->theta_hat + Ts * obs->omega_hat;
    float om_pred = obs->omega_hat
                  + Ts * ((Kt * iq_mech) - (B * obs->omega_hat) - obs->T_hat) / J;

    /* 位置残差 (最短路径) */
    float err = theta_mech - th_pred;
    while (err > FOC_PI) {
        err -= 2.0f * FOC_PI;
    }
    while (err < -FOC_PI) {
        err += 2.0f * FOC_PI;
    }

    /* 三重极点 -w0 离散校正增益 (前向欧拉).
     * 必须乘 Ts: 连续增益 L1=3w0 直接用于离散会大 ~1/Ts 倍,
     * 使离散闭环特征值 |λ|>>1 发散(台架实测 omega_hat 爆 3e5°/s)。
     * 离散极点 z0=e^{-w0·Ts}, α=1-z0≈w0·Ts, 增益 L1=3α, L2=3α²/Ts,
     * L3=-J·α³/Ts², 即 3w0Ts / 3w0²Ts / -J·w0³Ts。 */
    float L1 = 3.0f * w0 * Ts;
    float L2 = 3.0f * w0 * w0 * Ts;

    obs->theta_hat = th_pred + L1 * err;
    obs->omega_hat = om_pred + L2 * err;

    /* omega_hat 输出平滑 LPF (控制用): 滤位置量化校正尖峰。
     * 内部预测保持原始 omega_hat(快), 输出 omega_lpf 平滑(慢), 不双重滞后。 */
    {
        const float wc = 2.0f * FOC_PI * FOC_OBSERVER_OUTPUT_LPF_HZ;
        const float alpha_l = (wc * Ts) / (1.0f + wc * Ts);
        obs->omega_lpf += alpha_l * (obs->omega_hat - obs->omega_lpf);
    }

    /* T 估计: DOB 从动力学反解 T = Kt*Iq - J*dω/dt - B*ω。
     * 不用位置残差学习: 位置量化残差零均值, 对恒定摩擦不可观, L3 放大只会
     * 振荡(台架实测 t_gain=50 → omega 振荡、T 仍 0)。低速 acc≈0, T≈Kt*Iq
     * (即摩擦力矩)。omega_hat 原始高频, 用 omega_lpf 微分。 */
    {
        float acc = (obs->omega_lpf - obs->omega_prev) / Ts;
        float T_dob = (Kt * iq_mech) - (J * acc) - (B * obs->omega_lpf);
        if (T_dob > obs->T_max) {
            T_dob = obs->T_max;
        } else if (T_dob < -obs->T_max) {
            T_dob = -obs->T_max;
        }
        const float wc_t = 2.0f * FOC_PI * FOC_OBSERVER_T_LPF_HZ;
        const float alpha_t = (wc_t * Ts) / (1.0f + wc_t * Ts);
        obs->T_hat += alpha_t * (T_dob - obs->T_hat);
        obs->T_hat_lpf = obs->T_hat;
    }
    obs->omega_prev = obs->omega_lpf;
}
