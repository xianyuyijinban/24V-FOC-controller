/**
 * @file    foc_dtcomp.c
 * @brief   逆变器死区补偿实现（方案书 §5.3）
 * @note    死区效应：相电流 > 0 时下管续流，实际相电压比指令低 Vdt；
 *          电流 < 0 时高管续流，实际相电压比指令高 Vdt。
 *          补偿 = 按电流符号反向叠加 Vdt。
 */

#include "foc_dtcomp.h"
#include <math.h>
#include <string.h>

void FOC_DtComp_Init(FOC_DtComp_t *dt)
{
    if (dt == NULL) return;
    memset(dt, 0, sizeof(FOC_DtComp_t));
    dt->enabled = 0U;
    dt->amplitude_v = FOC_DTCOMP_DEFAULT_AMPLITUDE_V;
}

/* 带滞环的符号判断：超过 +hyst 锁正，低于 -hyst 锁负，环内不更新 */
static uint8_t DtComp_SignWithHyst(const float current, uint8_t prev_sign, const float hyst)
{
    if (current > hyst)  return 1U;
    if (current < -hyst) return 2U;
    return prev_sign;   /* 环内保持 */
}

void FOC_DtComp_Apply(FOC_DtComp_t *dt, float ia, float ib, float ic, float vabc[3])
{
    if (dt == NULL) return;
    if (dt->enabled == 0U) {
        dt->comp_a = dt->comp_b = dt->comp_c = 0.0f;
        return;
    }

    dt->sign_a = DtComp_SignWithHyst(ia, dt->sign_a, FOC_DTCOMP_SIGN_HYST_A);
    dt->sign_b = DtComp_SignWithHyst(ib, dt->sign_b, FOC_DTCOMP_SIGN_HYST_A);
    dt->sign_c = DtComp_SignWithHyst(ic, dt->sign_c, FOC_DTCOMP_SIGN_HYST_A);

    /* i > 0: 实际电压偏低 → 补 +Vdt；i < 0: 实际偏高 → 补 −Vdt
     * sign==0（滞环内且从未越环）不补偿 */
    dt->comp_a = (dt->sign_a == 1U) ? dt->amplitude_v
               : (dt->sign_a == 2U) ? -dt->amplitude_v : 0.0f;
    dt->comp_b = (dt->sign_b == 1U) ? dt->amplitude_v
               : (dt->sign_b == 2U) ? -dt->amplitude_v : 0.0f;
    dt->comp_c = (dt->sign_c == 1U) ? dt->amplitude_v
               : (dt->sign_c == 2U) ? -dt->amplitude_v : 0.0f;

    vabc[0] += dt->comp_a;
    vabc[1] += dt->comp_b;
    vabc[2] += dt->comp_c;
}
