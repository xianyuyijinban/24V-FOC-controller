/**
 * @file    uart_upload.c
 * @brief   DRV8350S 参数和故障数据 UART DMA 上传实现
 */

#include "uart_upload.h"
#include "foc_app.h"
#include "tle5012.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 私有变量 -----------------------------------------------------------------*/
static UART_HandleTypeDef* s_huart = NULL;
static DRV8350S_Handle_t* s_drvHandle = NULL;

static uint8_t s_txBuf[DRV_UART_BUF_SIZE];
static volatile bool s_txBusy = false;
static volatile bool s_enabled = true;
static uint32_t s_uploadInterval = DRV_UPLOAD_INTERVAL_MS;
static uint32_t s_lastUploadTime = 0;

static DrvUart_FaultRecord_t s_faultHistory[DRV_FAULT_HISTORY_SIZE];
static uint8_t s_faultHistoryHead = 0;
static uint8_t s_faultHistoryCount = 0;

static DrvUart_Statistics_t s_stats = {0};
static DrvUart_DataPacket_t s_lastFault = {0};

static uint32_t s_lastFaultFlags = 0;  /* 用于检测新故障 */

/* 私有函数声明 -------------------------------------------------------------*/
static void DrvUart_CollectData(DrvUart_DataPacket_t* packet, uint8_t type);
static int16_t DrvUart_FormatNormal(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize);
static int16_t DrvUart_FormatFault(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize);
static void DrvUart_AddFaultHistory(const DrvUart_DataPacket_t* packet);
static bool DrvUart_StartSend(uint16_t len);
static int16_t DrvUart_Append(uint8_t* buf, uint16_t bufSize, int16_t len, const char* fmt, ...);

/* 函数实现 -----------------------------------------------------------------*/

/**
 * @brief 初始化 UART 上传模块
 */
HAL_StatusTypeDef DrvUart_Init(UART_HandleTypeDef* huart, DRV8350S_Handle_t* drvHandle)
{
    if (huart == NULL || drvHandle == NULL) {
        return HAL_ERROR;
    }
    
    s_huart = huart;
    s_drvHandle = drvHandle;
    s_txBusy = false;
    s_enabled = true;
    s_lastUploadTime = 0;
    s_uploadInterval = DRV_UPLOAD_INTERVAL_MS;
    
    memset(s_txBuf, 0, sizeof(s_txBuf));
    memset(s_faultHistory, 0, sizeof(s_faultHistory));
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_lastFault, 0, sizeof(s_lastFault));
    
    s_faultHistoryHead = 0;
    s_faultHistoryCount = 0;
    s_lastFaultFlags = 0;
    
    return HAL_OK;
}

/**
 * @brief 反初始化 UART 上传模块
 */
void DrvUart_DeInit(void)
{
    if (s_huart != NULL && s_txBusy) {
        HAL_UART_DMAStop(s_huart);
    }
    
    s_huart = NULL;
    s_drvHandle = NULL;
    s_txBusy = false;
    s_enabled = false;
}

/* 外部声明 - FOC应用层数据 */
extern FOC_AppHandle_t g_foc_app;

/**
 * @brief 收集 DRV8350S、TLE5012 和 FOC 数据
 */
static void DrvUart_CollectData(DrvUart_DataPacket_t* packet, uint8_t type)
{
    if (packet == NULL || s_drvHandle == NULL) {
        return;
    }

    packet->timestamp = HAL_GetTick();
    packet->packetType = type;

    /* 从 TLE5012 获取编码器数据 */
    packet->angle = tle5012_sensor.angle;
    packet->rawAngle = tle5012_sensor.raw_angle;
    packet->crcError = tle5012_sensor.crc_error;

    /* 从 DRV8350S 句柄获取数据 */
    packet->faultStatus1 = s_drvHandle->runtime.regFaultStatus1;
    packet->vgsStatus2 = s_drvHandle->runtime.regVgsStatus2;
    packet->driverCtrl = s_drvHandle->runtime.regDriverCtrl;
    packet->ocpCtrl = s_drvHandle->runtime.regOcpCtrl;
    packet->faultFlags = s_drvHandle->runtime.faultFlags;
    packet->isFaultActive = s_drvHandle->runtime.isFaultActive;

    /* 从 FOC 应用层获取数据 - 新增 */
    packet->Id = g_foc_app.foc.Idq.d;
    packet->Iq = g_foc_app.foc.Idq.q;
    packet->Vd = g_foc_app.foc.Vdq.d;
    packet->Vq = g_foc_app.foc.Vdq.q;
    packet->speed = g_foc_app.speed_mech;
    packet->Id_ref = g_foc_app.Id_ref;
    packet->Iq_ref = g_foc_app.Iq_ref;
    packet->focState = (uint8_t)g_foc_app.state;
}

/**
 * @brief 格式化正常数据为清晰文本
 */
static int16_t DrvUart_FormatNormal(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)
{
    int16_t len = 0;
#define APPEND_FMT(...) \
    do { \
        len = DrvUart_Append(buf, bufSize, len, __VA_ARGS__); \
        if (len < 0) { \
            return -1; \
        } \
    } while (0)
    
    if (packet == NULL || buf == NULL || bufSize < 300) {
        return -1;
    }
    
    /* 标题 */
    APPEND_FMT("\r\n========== FOC Controller Status ==========\r\n");
    
    /* 时间戳 */
    APPEND_FMT("Time: %lu ms\r\n\r\n", packet->timestamp);
    
    /* TLE5012 编码器数据 */
    APPEND_FMT("[TLE5012 Encoder]\r\n");
    APPEND_FMT("  Angle:  %7.2f deg\r\n", packet->angle);
    APPEND_FMT("  Raw:    %5u (0x%04X)\r\n", packet->rawAngle, packet->rawAngle);
    APPEND_FMT("  CRC:    %s\r\n\r\n", packet->crcError ? "ERROR!" : "OK");
    
    /* DRV8350S 驱动器数据 */
    APPEND_FMT("[DRV8350S Driver]\r\n");
    APPEND_FMT("  FAULT1: 0x%04X\r\n", packet->faultStatus1);
    APPEND_FMT("  VGS2:   0x%04X\r\n", packet->vgsStatus2);
    APPEND_FMT("  Status: %s\r\n", packet->isFaultActive ? ">>> FAULT <<<" : "Normal");

    /* FOC 控制数据 - 新增 */
    APPEND_FMT("\r\n[FOC Control]\r\n");
    APPEND_FMT("  State:  %d\r\n", packet->focState);
    APPEND_FMT("  Id:     %7.3f A (ref: %7.3f)\r\n", packet->Id, packet->Id_ref);
    APPEND_FMT("  Iq:     %7.3f A (ref: %7.3f)\r\n", packet->Iq, packet->Iq_ref);
    APPEND_FMT("  Vd:     %7.3f V\r\n", packet->Vd);
    APPEND_FMT("  Vq:     %7.3f V\r\n", packet->Vq);
    APPEND_FMT("  Speed:  %7.2f rad/s\r\n", packet->speed);

    /* 分隔线 */
    APPEND_FMT("===========================================\r\n");

#undef APPEND_FMT
    
    return (len > 0 && len < bufSize) ? len : -1;
}

/**
 * @brief 格式化故障数据为详细文本
 */
static int16_t DrvUart_FormatFault(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)
{
    int16_t len = 0;
    uint16_t fs1, vs2;
#define APPEND_FMT(...) \
    do { \
        len = DrvUart_Append(buf, bufSize, len, __VA_ARGS__); \
        if (len < 0) { \
            return -1; \
        } \
    } while (0)
    
    if (packet == NULL || buf == NULL || bufSize < 400) {
        return -1;
    }
    
    fs1 = packet->faultStatus1;
    vs2 = packet->vgsStatus2;
    
    /* 故障标题 */
    APPEND_FMT("\r\n========== !!! FAULT DETECTED !!! ==========\r\n");
    
    /* 时间戳 */
    APPEND_FMT("Time: %lu ms\r\n\r\n", packet->timestamp);
    
    /* TLE5012 编码器数据 (故障时也显示) */
    APPEND_FMT("[TLE5012 Encoder]\r\n");
    APPEND_FMT("  Angle:  %7.2f deg\r\n", packet->angle);
    APPEND_FMT("  Raw:    %5u (0x%04X)\r\n", packet->rawAngle, packet->rawAngle);
    APPEND_FMT("  CRC:    %s\r\n\r\n", packet->crcError ? "ERROR!" : "OK");
    
    /* DRV8350S 故障详情 */
    APPEND_FMT("[DRV8350S Fault Details]\r\n");
    APPEND_FMT("  FAULT1: 0x%04X | VGS2: 0x%04X\r\n\r\n", fs1, vs2);
    
    /* FAULT_STATUS_1 故障 */
    if (fs1 & (1U << 10))
        APPEND_FMT("  [FAULT] General Fault\r\n");
    if (fs1 & (1U << 9))
        APPEND_FMT("  [CRIT]  VDS Overcurrent!\r\n");
    if (fs1 & (1U << 8))
        APPEND_FMT("  [CRIT]  Gate Drive Fault!\r\n");
    if (fs1 & (1U << 7))
        APPEND_FMT("  [CRIT]  Undervoltage Lockout!\r\n");
    if (fs1 & (1U << 6))
        APPEND_FMT("  [CRIT]  Overtemperature Shutdown!\r\n");
    
    /* VDS 过流相别 */
    if (fs1 & 0x003F) {
        APPEND_FMT("\r\n  VDS OCP Phase:\r\n");
        if (fs1 & (1U << 5)) APPEND_FMT("    - A High-Side\r\n");
        if (fs1 & (1U << 4)) APPEND_FMT("    - A Low-Side\r\n");
        if (fs1 & (1U << 3)) APPEND_FMT("    - B High-Side\r\n");
        if (fs1 & (1U << 2)) APPEND_FMT("    - B Low-Side\r\n");
        if (fs1 & (1U << 1)) APPEND_FMT("    - C High-Side\r\n");
        if (fs1 & (1U << 0)) APPEND_FMT("    - C Low-Side\r\n");
    }
    
    /* VGS_STATUS_2 故障 */
    /* Note: DRV8350S does NOT have CSA, Bit 10-8 are Reserved */
    if (vs2 & (1U << 7))
        APPEND_FMT("  [WARN]  Overtemperature Warning\r\n");
    if (vs2 & (1U << 6))
        APPEND_FMT("  [WARN]  Gate Drive UVLO\r\n");
    
    /* VGS 故障相别 */
    if (vs2 & 0x003F) {
        APPEND_FMT("\r\n  VGS Fault Phase:\r\n");
        if (vs2 & (1U << 5)) APPEND_FMT("    - A High-Side\r\n");
        if (vs2 & (1U << 4)) APPEND_FMT("    - A Low-Side\r\n");
        if (vs2 & (1U << 3)) APPEND_FMT("    - B High-Side\r\n");
        if (vs2 & (1U << 2)) APPEND_FMT("    - B Low-Side\r\n");
        if (vs2 & (1U << 1)) APPEND_FMT("    - C High-Side\r\n");
        if (vs2 & (1U << 0)) APPEND_FMT("    - C Low-Side\r\n");
    }
    
    /* 建议操作 */
    APPEND_FMT("\r\n>>> ACTION REQUIRED <<<\r\n");
    APPEND_FMT("  1. Disable PWM immediately\r\n");
    APPEND_FMT("  2. Check power supply\r\n");
    APPEND_FMT("  3. Verify MOSFETs status\r\n");
    
    APPEND_FMT("=============================================\r\n");

#undef APPEND_FMT
    
    return (len > 0 && len < bufSize) ? len : -1;
}

/**
 * @brief 启动 DMA 发送
 */
static bool DrvUart_StartSend(uint16_t len)
{
    if (s_huart == NULL || len == 0 || len > DRV_UART_BUF_SIZE) {
        return false;
    }
    
    if (s_txBusy) {
        return false;  /* 正在发送 */
    }
    
    if (HAL_UART_Transmit_DMA(s_huart, s_txBuf, len) != HAL_OK) {
        s_stats.txErrors++;
        return false;
    }
    
    s_txBusy = true;
    s_stats.isTxBusy = 1;
    return true;
}

static int16_t DrvUart_Append(uint8_t* buf, uint16_t bufSize, int16_t len, const char* fmt, ...)
{
    int written;
    va_list args;

    if ((buf == NULL) || (fmt == NULL) || (len < 0) || ((uint16_t)len >= bufSize)) {
        return -1;
    }

    va_start(args, fmt);
    written = vsnprintf((char*)buf + len, (size_t)(bufSize - (uint16_t)len), fmt, args);
    va_end(args);

    if ((written < 0) || (written >= (int)(bufSize - (uint16_t)len))) {
        return -1;
    }

    return (int16_t)(len + written);
}

/**
 * @brief 添加故障到历史记录
 */
static void DrvUart_AddFaultHistory(const DrvUart_DataPacket_t* packet)
{
    if (packet == NULL) {
        return;
    }
    
    memcpy(&s_faultHistory[s_faultHistoryHead].data, packet, sizeof(DrvUart_DataPacket_t));
    s_faultHistory[s_faultHistoryHead].valid = 1;
    
    s_faultHistoryHead = (s_faultHistoryHead + 1) % DRV_FAULT_HISTORY_SIZE;
    
    if (s_faultHistoryCount < DRV_FAULT_HISTORY_SIZE) {
        s_faultHistoryCount++;
    }
    
    /* 保存最后一次故障 */
    memcpy(&s_lastFault, packet, sizeof(DrvUart_DataPacket_t));
}

/**
 * @brief 主处理函数
 */
void DrvUart_Process(void)
{
    uint32_t currentTime;
    DrvUart_DataPacket_t packet;
    int16_t len;
    bool isNewFault;
    
    if (!s_enabled || s_huart == NULL || s_drvHandle == NULL) {
        return;
    }
    
    currentTime = HAL_GetTick();
    
    /* 检测新故障（故障标志变化） */
    isNewFault = (s_drvHandle->runtime.faultFlags != 0) && 
                 (s_drvHandle->runtime.faultFlags != s_lastFaultFlags);
    
    if (isNewFault) {
        /* 新故障，立即上传 */
        DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
        
        len = DrvUart_FormatFault(&packet, s_txBuf, DRV_UART_BUF_SIZE);
        if (len > 0 && DrvUart_StartSend(len)) {
            DrvUart_AddFaultHistory(&packet);
            s_stats.faultUploads++;
            s_stats.totalUploads++;
            s_stats.lastUploadTime = currentTime;
            s_lastUploadTime = currentTime;
            /* 仅在故障包成功进入发送流程后更新，避免首次上报被忙状态吞掉 */
            s_lastFaultFlags = s_drvHandle->runtime.faultFlags;
        }
    }
    else if (s_drvHandle->runtime.isFaultActive && !s_txBusy) {
        /* 故障持续中，每 500ms 重复发送 */
        if ((currentTime - s_stats.lastUploadTime) >= 500) {
            DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
            len = DrvUart_FormatFault(&packet, s_txBuf, DRV_UART_BUF_SIZE);
            if (len > 0 && DrvUart_StartSend(len)) {
                s_stats.totalUploads++;
                s_stats.lastUploadTime = currentTime;
                s_lastUploadTime = currentTime;
            }
        }
    }
    else if (!s_txBusy && (currentTime - s_lastUploadTime) >= s_uploadInterval) {
        /* 正常周期性上传 */
        DrvUart_CollectData(&packet, DRV_PKT_TYPE_NORMAL);
        
        len = DrvUart_FormatNormal(&packet, s_txBuf, DRV_UART_BUF_SIZE);
        if (len > 0) {
            if (DrvUart_StartSend(len)) {
                s_stats.totalUploads++;
                s_stats.lastUploadTime = currentTime;
                s_lastUploadTime = currentTime;
            }
        }
        
        /* 清除故障标志记录（故障已恢复） */
        if (s_lastFaultFlags != 0 && !s_drvHandle->runtime.isFaultActive) {
            s_lastFaultFlags = 0;
        }
    }
}

/**
 * @brief 立即上传（阻塞模式）
 */
void DrvUart_UploadImmediate(void)
{
    DrvUart_DataPacket_t packet;
    int16_t len;
    uint32_t waitStart;
    
    if (s_huart == NULL || s_drvHandle == NULL) {
        return;
    }
    
    /* 等待当前 DMA 完成（带超时，避免异常情况下死等） */
    waitStart = HAL_GetTick();
    while (s_txBusy) {
        if ((HAL_GetTick() - waitStart) > 100U) {
            s_stats.txErrors++;
            return;
        }
    }
    
    DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
    len = DrvUart_FormatFault(&packet, s_txBuf, DRV_UART_BUF_SIZE);
    
    if (len > 0) {
        HAL_UART_Transmit(s_huart, s_txBuf, len, 100);
    }
}

/**
 * @brief 强制上传故障数据
 */
bool DrvUart_UploadFault(void)
{
    DrvUart_DataPacket_t packet;
    int16_t len;
    
    if (s_huart == NULL || s_drvHandle == NULL) {
        return false;
    }
    
    DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
    DrvUart_AddFaultHistory(&packet);
    
    len = DrvUart_FormatFault(&packet, s_txBuf, DRV_UART_BUF_SIZE);
    if (len > 0) {
        return DrvUart_StartSend(len);
    }
    
    return false;
}

/**
 * @brief UART 发送完成回调
 */
void DrvUart_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == s_huart) {
        s_txBusy = false;
        s_stats.isTxBusy = 0;
    }
}

/**
 * @brief 设置上传使能
 */
void DrvUart_SetEnable(bool enable)
{
    s_enabled = enable;
}

/**
 * @brief 设置上传间隔
 */
void DrvUart_SetInterval(uint32_t intervalMs)
{
    if (intervalMs >= 10 && intervalMs <= 10000) {
        s_uploadInterval = intervalMs;
    }
}

/**
 * @brief 获取统计信息
 */
void DrvUart_GetStatistics(DrvUart_Statistics_t* stats)
{
    if (stats != NULL) {
        memcpy(stats, (void*)&s_stats, sizeof(DrvUart_Statistics_t));
    }
}

/**
 * @brief 清除故障历史
 */
void DrvUart_ClearFaultHistory(void)
{
    s_faultHistoryHead = 0;
    s_faultHistoryCount = 0;
    memset(s_faultHistory, 0, sizeof(s_faultHistory));
}

/**
 * @brief 获取故障历史数量
 */
uint8_t DrvUart_GetFaultHistoryCount(void)
{
    return s_faultHistoryCount;
}

/**
 * @brief 检查是否有活跃故障
 */
bool DrvUart_HasActiveFault(void)
{
    if (s_drvHandle == NULL) {
        return false;
    }
    return (s_drvHandle->runtime.isFaultActive != 0);
}

/**
 * @brief 获取最后一次故障
 */
void DrvUart_GetLastFault(DrvUart_DataPacket_t* packet)
{
    if (packet != NULL && s_lastFault.timestamp != 0) {
        memcpy(packet, &s_lastFault, sizeof(DrvUart_DataPacket_t));
    }
}
