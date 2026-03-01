/**
 * @file    tle5012.h
 * @brief   TLE5012B磁编码器驱动
 * 
 * 【修复记录】
 * v1.1 - 添加故障回调和数据有效性检查 (修复TLE5012-001)
 */

#ifndef __TLE5012_H
#define __TLE5012_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

// TLE5012B CS引脚接地，始终使能，无需软件控制
// #define TLE5012_CS_PORT      GPIOB
// #define TLE5012_CS_PIN       GPIO_PIN_0

// TLE5012B 寄存器地址
#define TLE5012_REG_AVAL     0x02  // 角度值寄存器

/* 【新增】故障类型定义 */
typedef enum {
    TLE5012_FAULT_NONE = 0,     /* 无故障 */
    TLE5012_FAULT_CRC,          /* CRC校验错误 */
    TLE5012_FAULT_TIMEOUT,      /* 通信超时 */
    TLE5012_FAULT_DATA          /* 数据异常 */
} TLE5012_Fault_t;

/* 【新增】故障回调函数类型 */
typedef void (*TLE5012_FaultCallback_t)(TLE5012_Fault_t fault);

typedef struct {
    float angle;            // 角度值 0.0 ~ 360.0
    uint16_t raw_angle;     // 原始角度数据
    uint8_t status;         // 状态字节 (Safety Word低8位)
    uint8_t crc_error;      // CRC错误标志 (1=错误)
    uint8_t update_flag;    // 数据更新标志
    uint8_t data_valid;     // 【新增】数据有效标志
} TLE5012_Data_t;

void TLE5012_Init(void);
void TLE5012_StartRead(void); // 触发异步读取
void TLE5012_ProcessData(uint16_t *rx_buf); // 处理接收到的数据
float TLE5012_GetAngle(void); // 获取角度值（0-360度）

/* 【新增】故障处理函数 */
void TLE5012_RegisterFaultCallback(TLE5012_FaultCallback_t callback);
uint8_t TLE5012_GetCRCErrorCount(void);
void TLE5012_ClearCRCErrorCount(void);
uint8_t TLE5012_IsDataValid(void);

extern TLE5012_Data_t tle5012_sensor;
extern uint16_t tle5012_rx_buf[3];  // SPI接收缓冲区

#ifdef __cplusplus
}
#endif

#endif /* __TLE5012_H */
