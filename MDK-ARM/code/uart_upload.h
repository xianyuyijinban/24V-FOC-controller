/**
 * @file    uart_upload.h
 * @brief   DRV8350S 参数和故障数据通过 UART1 DMA 上传到电脑
 * @note    支持周期性数据上传和故障触发上传
 */

#ifndef __UART_UPLOAD_H
#define __UART_UPLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "drv8350s.h"
#include "tle5012.h"
#include <stdint.h>
#include <stdbool.h>

/* 配置宏 -------------------------------------------------------------------*/
#define DRV_UART_BUF_SIZE           512     /* 发送缓冲区大小 */
#define DRV_UPLOAD_INTERVAL_MS      100     /* 正常数据上传间隔 (ms) */
#define DRV_FAULT_HISTORY_SIZE      8       /* 故障历史记录数量 */

/* 数据包类型 ---------------------------------------------------------------*/
typedef enum {
    DRV_PKT_TYPE_NORMAL = 0,    /* 正常周期性数据 */
    DRV_PKT_TYPE_FAULT,         /* 故障数据 */
    DRV_PKT_TYPE_RESPONSE,      /* 响应数据 */
    DRV_PKT_TYPE_DEBUG          /* 调试数据 */
} DrvUart_PacketType_t;

/* 上传数据结构 -------------------------------------------------------------*/
typedef struct {
    uint32_t timestamp;         /* 时间戳 (ms) */

    /* TLE5012 编码器数据 */
    float    angle;             /* 角度值 (0.0 ~ 360.0 度) */
    uint16_t rawAngle;          /* 原始角度数据 */
    uint8_t  crcError;          /* CRC 错误标志 */

    /* DRV8350S 驱动器数据 */
    uint16_t faultStatus1;      /* FAULT_STATUS_1 寄存器 */
    uint16_t vgsStatus2;        /* VGS_STATUS_2 寄存器 */
    uint16_t driverCtrl;        /* DRIVER_CTRL 寄存器 */
    uint16_t ocpCtrl;           /* OCP_CTRL 寄存器 */
    uint32_t faultFlags;        /* 解析后的故障标志 */
    uint8_t  isFaultActive;     /* 是否有故障 */

    /* FOC控制数据 - 新增 */
    float    Id;                /* D轴电流 */
    float    Iq;                /* Q轴电流 */
    float    Vd;                /* D轴电压 */
    float    Vq;                /* Q轴电压 */
    float    speed;             /* 转速 (rad/s) */
    float    Id_ref;            /* D轴电流参考 */
    float    Iq_ref;            /* Q轴电流参考 */
    uint8_t  focState;          /* FOC状态 */

    uint8_t  packetType;        /* 数据包类型 */
} DrvUart_DataPacket_t;

/* 故障记录结构 -------------------------------------------------------------*/
typedef struct {
    DrvUart_DataPacket_t data;
    uint8_t valid;
} DrvUart_FaultRecord_t;

/* 统计信息结构 -------------------------------------------------------------*/
typedef struct {
    uint32_t totalUploads;      /* 总上传次数 */
    uint32_t faultUploads;      /* 故障上传次数 */
    uint32_t lastUploadTime;    /* 上次上传时间 */
    uint32_t txErrors;          /* 发送错误次数 */
    uint8_t  isTxBusy;          /* 是否正在发送 */
} DrvUart_Statistics_t;

/* 函数声明 -----------------------------------------------------------------*/

/**
 * @brief 初始化 UART 上传模块
 * @param huart UART句柄 (huart1)
 * @param drvHandle DRV8350S句柄
 * @return HAL_OK 成功, HAL_ERROR 失败
 */
HAL_StatusTypeDef DrvUart_Init(UART_HandleTypeDef* huart, DRV8350S_Handle_t* drvHandle);

/**
 * @brief 反初始化 UART 上传模块
 */
void DrvUart_DeInit(void);

/**
 * @brief 主处理函数，在 main 循环中调用
 * @note 处理周期性上传和故障检测上传
 */
void DrvUart_Process(void);

/**
 * @brief 立即上传当前数据（阻塞模式，用于紧急故障）
 */
void DrvUart_UploadImmediate(void);

/**
 * @brief 强制上传故障数据（非阻塞 DMA 模式）
 * @return true 开始发送, false 发送忙
 */
bool DrvUart_UploadFault(void);

/**
 * @brief 设置上传使能/禁用
 */
void DrvUart_SetEnable(bool enable);

/**
 * @brief 设置上传间隔
 */
void DrvUart_SetInterval(uint32_t intervalMs);

/**
 * @brief 获取统计信息
 */
void DrvUart_GetStatistics(DrvUart_Statistics_t* stats);

/**
 * @brief 清除故障历史
 */
void DrvUart_ClearFaultHistory(void);

/**
 * @brief 获取故障历史数量
 */
uint8_t DrvUart_GetFaultHistoryCount(void);

/**
 * @brief UART 发送完成回调（在 HAL_UART_TxCpltCallback 中调用）
 */
void DrvUart_TxCpltCallback(UART_HandleTypeDef* huart);

/**
 * @brief 检查是否有活跃故障
 */
bool DrvUart_HasActiveFault(void);

/**
 * @brief 获取最后一次故障数据
 */
void DrvUart_GetLastFault(DrvUart_DataPacket_t* packet);

#ifdef __cplusplus
}
#endif

#endif /* __DRV8353S_UART_UPLOAD_H */
