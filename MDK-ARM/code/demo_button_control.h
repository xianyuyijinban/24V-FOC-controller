#ifndef __DEMO_BUTTON_CONTROL_H
#define __DEMO_BUTTON_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_app.h"

void DemoButtonControl_Init(FOC_AppHandle_t *app);
void DemoButtonControl_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEMO_BUTTON_CONTROL_H */
