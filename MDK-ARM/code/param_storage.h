/**
 * @file    param_storage.h
 * @brief   电机参数Flash存储管理
 * @note    使用STM32H743内部Flash存储电机参数
 */

#ifndef __PARAM_STORAGE_H
#define __PARAM_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "motor_identify.h"

/*==================== 配置参数 ====================*/

/* Flash存储地址（使用最后一个扇区） */
#define PARAM_FLASH_ADDR        0x081E0000  /* Bank2 Sector7 结束地址 */
#define PARAM_FLASH_SECTOR      FLASH_SECTOR_7
#define PARAM_FLASH_BANK        FLASH_BANK_2

/* 参数魔数（用于验证数据有效性） */
#define PARAM_MAGIC_NUMBER      0x4D4F544F  /* "MOTO" */
#define PARAM_VERSION_PRE_ALIGN_D_AXIS   0x00010003
#define PARAM_VERSION           0x00010004  /* 版本 1.4：编码器对齐零点改为d轴电零位 */

/* 最大重试次数 */
#define PARAM_MAX_RETRY         3

/* 电机参数有效范围：24N22P云台/关节类电机实测Rs可能超过10Ω */
#define PARAM_RS_MAX_OHM       30.0f

/* 参数有效性位图：用于故障详情定位 Param_IsValid 拒绝的具体字段 */
#define PARAM_INVALID_VALID_FLAG  (1UL << 0)
#define PARAM_INVALID_RS          (1UL << 1)
#define PARAM_INVALID_LD          (1UL << 2)
#define PARAM_INVALID_LQ          (1UL << 3)
#define PARAM_INVALID_KE          (1UL << 4)
#define PARAM_INVALID_PN          (1UL << 5)
#define PARAM_INVALID_ENCODER_DIR (1UL << 6)
#define PARAM_INVALID_J           (1UL << 7)

/*==================== 数据结构 ====================*/

/* 参数存储头部 */
typedef struct {
    uint32_t magic;         /* 魔数 */
    uint32_t version;       /* 版本号 */
    uint32_t size;          /* 数据大小 */
    uint32_t crc32;         /* CRC校验 */
    uint32_t timestamp;     /* 时间戳 */
    uint32_t reserved[3];   /* 保留 */
} ParamHeader_t;

/* 完整参数包 */
typedef struct {
    ParamHeader_t header;
    MotorParam_t motor;
    /* 可以扩展其他参数 */
} ParamPackage_t;

/* 存储状态 */
typedef enum {
    PARAM_OK = 0,
    PARAM_ERR_FLASH,
    PARAM_ERR_CRC,
    PARAM_ERR_INVALID,
    PARAM_ERR_ERASE,
    PARAM_ERR_WRITE,
} ParamStatus_t;

/*==================== 函数声明 ====================*/

/* 初始化 */
ParamStatus_t Param_Init(void);

/* 参数读写 */
ParamStatus_t Param_Load(MotorParam_t *param);
ParamStatus_t Param_Save(const MotorParam_t *param);

/* 参数验证 */
uint8_t Param_IsValid(const MotorParam_t *param);
uint32_t Param_GetInvalidFlags(const MotorParam_t *param);
void Param_SetDefault(MotorParam_t *param);

/* 工具函数 */
uint32_t Param_CalculateCRC32(const void *data, uint32_t size);
const char* Param_GetStatusString(ParamStatus_t status);

/* Flash操作 */
ParamStatus_t Param_EraseSector(void);
ParamStatus_t Param_WriteFlash(uint32_t addr, const uint32_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_STORAGE_H */
