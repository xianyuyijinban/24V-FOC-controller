/**
 * @file    tle5012.c
 * @brief   TLE5012B磁编码器驱动实现
 * 
 * 【修复记录】
 * v1.1 - 添加CRC错误计数和故障回调机制 (修复TLE5012-001)
 */

#include "head.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;

TLE5012_Data_t tle5012_sensor = {0};

// 函数前置声明
static uint8_t TLE5012_CalculateCRC8(uint16_t data);

// 私有变量
static uint8_t is_busy = 0;
static uint16_t tle5012_tx_buf[3];
uint16_t tle5012_rx_buf[3];  // 非静态，供外部使用

// 【新增】CRC错误计数和超时保护
static uint8_t crc_error_count = 0;
static uint32_t busy_start_time = 0;
static TLE5012_FaultCallback_t fault_callback = NULL;

#define CRC_ERROR_THRESHOLD     3       /* CRC错误触发故障的阈值 */
#define SPI_TIMEOUT_MS          10      /* SPI传输超时时间(ms) */

// 命令字构建: RW=1, Lock=0, UPD=0, ADDR=0x02, ND=1
// 1 0000 0 000010 0001 = 0x8021
#define TLE5012_READ_CMD 0x8021

void TLE5012_Init(void)
{
    memset(tle5012_tx_buf, 0, sizeof(tle5012_tx_buf));
    memset(tle5012_rx_buf, 0, sizeof(tle5012_rx_buf));
    crc_error_count = 0;
    is_busy = 0;
    
    // CS接地，始终使能，无需初始化GPIO
}

/**
 * @brief 注册故障回调函数
 * @param callback 故障回调函数指针
 * 
 * 【新增】当CRC错误超过阈值时调用
 */
void TLE5012_RegisterFaultCallback(TLE5012_FaultCallback_t callback)
{
    fault_callback = callback;
}

// 触发异步DMA读取
void TLE5012_StartRead(void)
{
    // 【修复】添加超时保护：如果忙状态持续超过SPI_TIMEOUT_MS，强制复位
    if (is_busy) {
        if ((HAL_GetTick() - busy_start_time) > SPI_TIMEOUT_MS) {
            // 超时：强制复位忙标志
            is_busy = 0;
            crc_error_count++;  // 计为一次通信错误
            
            if (fault_callback && crc_error_count >= CRC_ERROR_THRESHOLD) {
                fault_callback(TLE5012_FAULT_TIMEOUT);
            }
        } else {
            return; // 正常忙状态，跳过本次读取
        }
    }

    is_busy = 1;
    busy_start_time = HAL_GetTick();
    
    tle5012_tx_buf[0] = TLE5012_READ_CMD; // Command
    tle5012_tx_buf[1] = 0x0000;           // Dummy (为了接收Data)
    tle5012_tx_buf[2] = 0x0000;           // Dummy (为了接收Safety)

    // CS接地，始终使能，直接启动SPI DMA
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi3, 
        (uint8_t*)tle5012_tx_buf, (uint8_t*)tle5012_rx_buf, 3);
    
    // 如果启动失败，恢复状态
    if (status != HAL_OK) {
        is_busy = 0;
        crc_error_count++;
    }
}

// DMA传输完成回调处理函数
void TLE5012_ProcessData(uint16_t *rx_buf)
{
    // CS接地，无需拉高
    is_busy = 0;

    // rx_buf[0]: 收到的是发送出去的命令 (回环)
    // rx_buf[1]: 角度数据
    // rx_buf[2]: 安全字

    uint16_t raw_data = rx_buf[1];
    uint16_t safety_word = rx_buf[2];

    // 1. 提取角度值 (15位)
    tle5012_sensor.raw_angle = raw_data & 0x7FFF;
    
    // 转换为角度 (0-360度)
    // TLE5012B 15位分辨率，范围 0-32767
    tle5012_sensor.angle = (float)tle5012_sensor.raw_angle * (360.0f / 32768.0f);

    // 2. 提取状态信息
    tle5012_sensor.status = (uint8_t)(safety_word & 0xFF);

    // 3. CRC校验
    // Safety Word [15:8] 是 CRC (多项式 x^8+x^4+x^3+x^2+1, 初始值0xFF, 结果取反)
    uint8_t received_crc = (uint8_t)((safety_word >> 8) & 0xFF);
    uint8_t calculated_crc = TLE5012_CalculateCRC8(raw_data);
    
    // 【修复】添加CRC错误计数和故障升级机制
    if (received_crc != calculated_crc) {
        tle5012_sensor.crc_error = 1;
        crc_error_count++;
        
        // 错误计数超过阈值，触发故障回调
        if (crc_error_count >= CRC_ERROR_THRESHOLD) {
            if (fault_callback) {
                fault_callback(TLE5012_FAULT_CRC);
            }
            // 标记传感器数据无效
            tle5012_sensor.data_valid = 0;
        }
    } else {
        tle5012_sensor.crc_error = 0;
        tle5012_sensor.data_valid = 1;
        
        // 连续成功，减少错误计数
        if (crc_error_count > 0) {
            crc_error_count--;
        }
    }
    
    tle5012_sensor.update_flag = 1;
}

// 获取角度值（0-360度）
float TLE5012_GetAngle(void)
{
    return tle5012_sensor.angle;
}

/**
 * @brief 获取CRC错误计数
 * @return 当前CRC错误计数
 */
uint8_t TLE5012_GetCRCErrorCount(void)
{
    return crc_error_count;
}

/**
 * @brief 清除CRC错误计数
 */
void TLE5012_ClearCRCErrorCount(void)
{
    crc_error_count = 0;
}

/**
 * @brief 检查传感器数据是否有效
 * @return 1有效，0无效
 */
uint8_t TLE5012_IsDataValid(void)
{
    return tle5012_sensor.data_valid;
}

// CRC8 校验函数 (TLE5012B 专用)
// 多项式: x^8 + x^4 + x^3 + x^2 + 1 (0x1D)
// 初始值: 0xFF, 结果取反
static uint8_t TLE5012_CalculateCRC8(uint16_t data)
{
    uint8_t crc = 0xFF;
    uint8_t poly = 0x1D;
    
    // 处理高字节
    crc ^= (data >> 8);
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ poly;
        } else {
            crc <<= 1;
        }
    }
    
    // 处理低字节
    crc ^= (data & 0xFF);
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ poly;
        } else {
            crc <<= 1;
        }
    }
    
    return ~crc; // 结果取反
}
