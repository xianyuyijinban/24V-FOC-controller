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

// TLE5012B 使用软件片选，NSS 连接到 MCU PA15
// Encoder board: CN2.5/CN2.6 share the encoder DATA net in 3-wire SSC mode.
#define TLE5012_CS_PORT      TLE5012_NSS_GPIO_Port
#define TLE5012_CS_PIN       TLE5012_NSS_Pin

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
    uint16_t raw_word;      // 最近一次完整Data Word
    uint16_t safety_word;   // 最近一次完整Safety Word
    uint8_t status;         // 状态字节 (Safety Word高8位，含bit15复位/看门狗状态)
    uint8_t reset_fault;    // Safety Word bit15=0，表示芯片复位/看门狗异常
    uint8_t crc_error;      // CRC错误标志 (1=错误)
    uint8_t received_crc;   // Safety Word低8位
    uint8_t calculated_crc; // 本地计算CRC
    uint8_t data_ok;        // Interface OK + Angle Valid
    uint8_t update_flag;    // 数据更新标志
    uint8_t data_valid;     // 【新增】数据有效标志
    uint16_t spi_error_count;
    uint16_t crc_error_total;
} TLE5012_Data_t;

typedef struct {
    uint8_t active;
    uint8_t step;
    uint8_t cs_level;
    uint8_t sck_level;
    uint8_t data_out;
    uint8_t data_in;
    uint32_t last_tick;
} TLE5012_GpioDiagState_t;

void TLE5012_Init(void);
void TLE5012_StartRead(void); // 触发异步读取
void TLE5012_HandleTxComplete(void); // 发送命令完成，切换到接收阶段
void TLE5012_ProcessData(uint16_t *rx_buf); // 处理接收到的数据
void TLE5012_HandleTransferError(void); // SPI错误/启动失败恢复
float TLE5012_GetAngle(void); // 获取角度值（0-360度）
void TLE5012_GpioDiagStart(void);
void TLE5012_GpioDiagStop(void);
void TLE5012_GpioDiagService(void);
uint8_t TLE5012_IsGpioDiagActive(void);

/* 【新增】故障处理函数 */
void TLE5012_RegisterFaultCallback(TLE5012_FaultCallback_t callback);
uint8_t TLE5012_GetCRCErrorCount(void);
void TLE5012_ClearCRCErrorCount(void);
uint8_t TLE5012_IsDataValid(void);
uint16_t TLE5012_GetSpiErrorCount(void);
uint16_t TLE5012_GetCrcErrorTotal(void);
void TLE5012_ClearDiagnosticCounters(void);

extern TLE5012_Data_t tle5012_sensor;
extern volatile TLE5012_GpioDiagState_t tle5012_gpio_diag;
extern uint16_t tle5012_rx_buf[2];  // SPI接收缓冲区：Data + Safety

#ifdef __cplusplus
}
#endif

#endif /* __TLE5012_H */
