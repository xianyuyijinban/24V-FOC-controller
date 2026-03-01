#ifndef __HEAD_H
#define __HEAD_H

#include "main.h"
#include "drv8350s.h"
#include "tle5012.h"
#include "adc_sampling.h"
#include "foc_core.h"
#include "motor_identify.h"
#include "param_storage.h"
#include "foc_app.h"
#include "uart_upload.h"

	extern volatile uint16_t adc_data[8];
	extern volatile uint16_t urT_data[8];
	extern volatile uint8_t urR_data[128];
		
	extern DRV8350S_Handle_t drv8350s;
	extern UART_HandleTypeDef huart1;
	
	/* FOC应用层句柄 */
	extern FOC_AppHandle_t g_foc_app;

	void UART_Command_ProcessPending(void);
#endif
