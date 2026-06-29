#include "demo_button_control.h"

#include "main.h"
#include <math.h>

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState stable_level;
    GPIO_PinState last_sample;
    uint32_t last_change_ms;
} DemoButtonDebounce_t;

typedef enum {
    DEMO_BUTTON_MODE_NONE = 0,
    DEMO_BUTTON_MODE_SPEED_10DPS,
    DEMO_BUTTON_MODE_ZERO_SPRING,
} DemoButtonMode_t;

static const float DEMO_BUTTON_SPEED_REF_RAD_S = 0.174533f;
static const float DEMO_BUTTON_ZERO_SAT_RAD = 2.0943951f;
static const float DEMO_BUTTON_ZERO_SPRING_IQ_MAX = 1.5f;

static FOC_AppHandle_t *s_app = NULL;
static DemoButtonMode_t s_mode = DEMO_BUTTON_MODE_NONE;
static uint8_t s_zero_spring_wait_active = 0U;
static uint32_t s_zero_spring_wait_seq = 0U;
static DemoButtonDebounce_t s_mod1_button;
static DemoButtonDebounce_t s_mod2_button;

static void DemoButtonControl_ArmZeroSpringWait(void)
{
    if (s_app == NULL) {
        return;
    }

    s_zero_spring_wait_active = 1U;
    s_zero_spring_wait_seq = s_app->theta_sample_seq;
}

static void DemoButtonControl_InitButton(
    DemoButtonDebounce_t *button,
    GPIO_TypeDef *port,
    uint16_t pin)
{
    GPIO_PinState raw_level;

    raw_level = HAL_GPIO_ReadPin(port, pin);
    button->port = port;
    button->pin = pin;
    button->stable_level = raw_level;
    button->last_sample = raw_level;
    button->last_change_ms = HAL_GetTick();
}

static uint8_t DemoButtonControl_UpdateButton(DemoButtonDebounce_t *button)
{
    uint32_t now_ms;
    GPIO_PinState raw_level;
    GPIO_PinState previous_stable_level;

    raw_level = HAL_GPIO_ReadPin(button->port, button->pin);
    now_ms = HAL_GetTick();

    if (raw_level != button->last_sample) {
        button->last_sample = raw_level;
        button->last_change_ms = now_ms;
    }

    if ((now_ms - button->last_change_ms) < 25U) {
        return 0U;
    }

    if (raw_level == button->stable_level) {
        return 0U;
    }

    previous_stable_level = button->stable_level;
    button->stable_level = raw_level;

    return ((previous_stable_level != GPIO_PIN_RESET) && (raw_level == GPIO_PIN_RESET)) ? 1U : 0U;
}

static float DemoButtonControl_WrapPmPi(float angle_rad)
{
    while (angle_rad > FOC_PI) {
        angle_rad -= 2.0f * FOC_PI;
    }

    while (angle_rad < -FOC_PI) {
        angle_rad += 2.0f * FOC_PI;
    }

    return angle_rad;
}

static void DemoButtonControl_UpdateZeroSpringCurrent(void)
{
    float angle_error;
    float iq_mag;
    float iq_ref;

    if ((s_app == NULL) ||
        (s_mode != DEMO_BUTTON_MODE_ZERO_SPRING) ||
        (s_app->state != FOC_STATE_RUNNING) ||
        (s_app->control_mode != FOC_MODE_TORQUE)) {
        return;
    }

    if (s_zero_spring_wait_active != 0U) {
        if (s_app->theta_sample_seq == s_zero_spring_wait_seq) {
            return;
        }
        s_zero_spring_wait_active = 0U;
    }

    angle_error = DemoButtonControl_WrapPmPi(
        s_app->motor_param.theta_mech_zero - s_app->theta_mech);
    iq_mag = fabsf(angle_error) / DEMO_BUTTON_ZERO_SAT_RAD;
    if (iq_mag > 1.0f) {
        iq_mag = 1.0f;
    }

    iq_ref = copysignf(iq_mag * DEMO_BUTTON_ZERO_SPRING_IQ_MAX, angle_error);
    FOC_App_SetCurrentRef(s_app, 0.0f, iq_ref);
}

static void DemoButtonControl_HandleMod1Press(void)
{
    if ((s_app == NULL) ||
        (s_app->power_unlocked == 0U) ||
        (s_app->state == FOC_STATE_PARAM_IDENTIFY)) {
        return;
    }

    FOC_App_ResetMotionState(s_app);

    if (s_mode != DEMO_BUTTON_MODE_SPEED_10DPS) {
        s_mode = DEMO_BUTTON_MODE_SPEED_10DPS;
        s_zero_spring_wait_active = 0U;
        FOC_App_SetControlMode(s_app, FOC_MODE_SPEED);
        FOC_App_SetSpeedRef(s_app, DEMO_BUTTON_SPEED_REF_RAD_S);
    } else {
        s_mode = DEMO_BUTTON_MODE_ZERO_SPRING;
        DemoButtonControl_ArmZeroSpringWait();
        FOC_App_SetControlMode(s_app, FOC_MODE_TORQUE);
    }

    if (s_app->state == FOC_STATE_READY) {
        FOC_App_Enable(s_app);
    }

    DemoButtonControl_UpdateZeroSpringCurrent();
}

static void DemoButtonControl_HandleMod2Press(void)
{
    if ((s_app == NULL) || (s_app->power_unlocked == 0U)) {
        return;
    }

    if (s_app->state == FOC_STATE_PARAM_IDENTIFY) {
        FOC_App_StopIdentify(s_app);
        return;
    }

    FOC_App_Disable(s_app);
    FOC_App_ResetMotionState(s_app);
    FOC_App_StartIdentify(s_app);
}

void DemoButtonControl_Init(FOC_AppHandle_t *app)
{
    s_app = app;
    s_mode = DEMO_BUTTON_MODE_NONE;
    s_zero_spring_wait_active = 0U;
    s_zero_spring_wait_seq = 0U;
    DemoButtonControl_InitButton(&s_mod1_button, MOD1_GPIO_Port, MOD1_Pin);
    DemoButtonControl_InitButton(&s_mod2_button, MOD2_GPIO_Port, MOD2_Pin);
}

void DemoButtonControl_Service(void)
{
    uint8_t mod1_pressed;
    uint8_t mod2_pressed;

    if (s_app == NULL) {
        return;
    }

    if ((s_mode == DEMO_BUTTON_MODE_ZERO_SPRING) &&
        (s_app->state != FOC_STATE_RUNNING)) {
        DemoButtonControl_ArmZeroSpringWait();
    }

    mod1_pressed = DemoButtonControl_UpdateButton(&s_mod1_button);
    mod2_pressed = DemoButtonControl_UpdateButton(&s_mod2_button);

    if (mod1_pressed) {
        DemoButtonControl_HandleMod1Press();
    }

    if (mod2_pressed) {
        DemoButtonControl_HandleMod2Press();
    }

    DemoButtonControl_UpdateZeroSpringCurrent();
}
