/**
 * @file    param_storage.c
 * @brief   电机参数Flash存储管理实现
 * @note    使用STM32H743内部Flash存储电机参数
 */

#include "param_storage.h"
#include <string.h>

/*==================== CRC32表（标准多项式 0x04C11DB7）====================*/
static const uint32_t crc32_table[256] = {
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B,
    0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
    0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 0x4C11DB70, 0x48D0C6C7,
    0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
    0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3,
    0x709F7B7A, 0x745E66CD, 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039,
    0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF,
    0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
    0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB,
    0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1,
    0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 0x34867077, 0x30476DC0,
    0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
    0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4,
    0x0808D07D, 0x0CC9CDCA, 0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE,
    0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08,
    0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
    0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC,
    0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6,
    0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 0xE0B41DE7, 0xE4750050,
    0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
    0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34,
    0xDC3ABDED, 0xD8FBA05A, 0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637,
    0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1,
    0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
    0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5,
    0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF,
    0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 0xF12F560E, 0xF5EE4BB9,
    0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
    0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD,
    0xCDA1F604, 0xC960EBB3, 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7,
    0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71,
    0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
    0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2,
    0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8,
    0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 0x119B4BE9, 0x155A565E,
    0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
    0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A,
    0x2D15EBE3, 0x29D4F654, 0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0,
    0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676,
    0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
    0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662,
    0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668,
    0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
};

/*==================== 函数实现 ====================*/

/**
 * @brief 初始化参数存储模块
 * @return 状态码
 */
ParamStatus_t Param_Init(void)
{
    /* 检查Flash地址是否有效 */
    if (PARAM_FLASH_ADDR < 0x08000000 || PARAM_FLASH_ADDR > 0x08200000) {
        return PARAM_ERR_FLASH;
    }
    return PARAM_OK;
}

/**
 * @brief 从Flash加载参数
 * @param param 电机参数结构体指针
 * @return 状态码
 */
ParamStatus_t Param_Load(MotorParam_t *param)
{
    if (param == NULL) {
        return PARAM_ERR_INVALID;
    }

    ParamPackage_t *package = (ParamPackage_t *)PARAM_FLASH_ADDR;
    
    /* 检查魔数 */
    if (package->header.magic != PARAM_MAGIC_NUMBER) {
        return PARAM_ERR_INVALID;
    }
    
    /* 检查版本 */
    if (package->header.version != PARAM_VERSION) {
        return PARAM_ERR_INVALID;
    }
    
    /* 计算并验证CRC32 */
    uint32_t crc_calc = Param_CalculateCRC32(&package->motor, sizeof(MotorParam_t));
    if (crc_calc != package->header.crc32) {
        return PARAM_ERR_CRC;
    }
    
    /* 复制参数 */
    memcpy(param, &package->motor, sizeof(MotorParam_t));
    
    return PARAM_OK;
}

/**
 * @brief 保存参数到Flash
 * @param param 电机参数结构体指针
 * @return 状态码
 */
ParamStatus_t Param_Save(const MotorParam_t *param)
{
    ParamPackage_t package;
    ParamStatus_t status;

    if (param == NULL) {
        return PARAM_ERR_INVALID;
    }
    
    /* 填充头部 */
    package.header.magic = PARAM_MAGIC_NUMBER;
    package.header.version = PARAM_VERSION;
    package.header.size = sizeof(MotorParam_t);
    package.header.timestamp = HAL_GetTick();
    memset(package.header.reserved, 0, sizeof(package.header.reserved));
    
    /* 复制电机参数 */
    memcpy(&package.motor, param, sizeof(MotorParam_t));
    
    /* 计算CRC32 */
    package.header.crc32 = Param_CalculateCRC32(&package.motor, sizeof(MotorParam_t));
    
    /* 解锁Flash */
    HAL_FLASH_Unlock();
    
    /* 擦除扇区 */
    status = Param_EraseSector();
    if (status != PARAM_OK) {
        HAL_FLASH_Lock();
        return status;
    }
    
    /* 写入Flash */
    status = Param_WriteFlash(PARAM_FLASH_ADDR, (uint32_t *)&package, sizeof(ParamPackage_t));
    
    /* 锁定Flash */
    HAL_FLASH_Lock();
    
    return status;
}

/**
 * @brief 检查参数是否有效
 * @param param 电机参数结构体指针
 * @return 1有效，0无效
 */
uint8_t Param_IsValid(const MotorParam_t *param)
{
    /* 检查有效性标志 */
    if (param->valid_flag != 0xFFFFFFFF) {
        return 0;
    }
    
    /* 检查参数范围 */
    if (param->Rs <= 0 || param->Rs > 10.0f) return 0;      /* 电阻 0~10Ω */
    if (param->Ld <= 0 || param->Ld > 0.01f) return 0;      /* 电感 0~10mH */
    if (param->Lq <= 0 || param->Lq > 0.01f) return 0;
    if (param->Ke < 0 || param->Ke > 1.0f) return 0;        /* 反电动势常数 */
    if (param->Pn == 0 || param->Pn > 50) return 0;         /* 极对数 */
    if (param->J < 0 || param->J > 1.0f) return 0;          /* 转动惯量 */
    
    return 1;
}

/**
 * @brief 设置默认参数
 * @param param 电机参数结构体指针
 */
void Param_SetDefault(MotorParam_t *param)
{
    memset(param, 0, sizeof(MotorParam_t));
    
    /* 典型24V关节电机默认值 */
    param->Rs = 0.5f;           /* 0.5Ω */
    param->Ld = 0.0005f;        /* 0.5mH */
    param->Lq = 0.0005f;        /* 0.5mH */
    param->Ke = 0.05f;          /* 0.05 V/(rad/s) */
    param->Pn = 7;              /* 7对极 */
    param->J = 0.0001f;         /* 0.0001 kg·m² */
    param->B = 0.001f;          /* 0.001 N·m·s/rad */
    param->theta_offset = 0.0f;
    param->valid_flag = 0;
}

/**
 * @brief 计算CRC32校验
 * @param data 数据指针
 * @param size 数据大小（字节）
 * @return CRC32值
 */
uint32_t Param_CalculateCRC32(const void *data, uint32_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < size; i++) {
        uint8_t table_index = (uint8_t)((crc >> 24) ^ bytes[i]);
        crc = crc32_table[table_index] ^ (crc << 8);
    }
    
    return crc ^ 0xFFFFFFFF;
}

/**
 * @brief 获取状态字符串
 * @param status 状态码
 * @return 状态描述字符串
 */
const char* Param_GetStatusString(ParamStatus_t status)
{
    switch (status) {
        case PARAM_OK:          return "OK";
        case PARAM_ERR_FLASH:   return "Flash Error";
        case PARAM_ERR_CRC:     return "CRC Error";
        case PARAM_ERR_INVALID: return "Invalid Data";
        case PARAM_ERR_ERASE:   return "Erase Error";
        case PARAM_ERR_WRITE:   return "Write Error";
        default:                return "Unknown";
    }
}

/**
 * @brief 擦除Flash扇区
 * @return 状态码
 */
ParamStatus_t Param_EraseSector(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = PARAM_FLASH_BANK;
    erase_init.Sector = PARAM_FLASH_SECTOR;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7V~3.6V */
    
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    
    if (status != HAL_OK) {
        return PARAM_ERR_ERASE;
    }
    
    return PARAM_OK;
}

/**
 * @brief 写入Flash
 * @param addr Flash地址
 * @param data 数据指针
 * @param size 数据大小（字节）
 * @return 状态码
 */
ParamStatus_t Param_WriteFlash(uint32_t addr, const uint32_t *data, uint32_t size)
{
    uint32_t offset = 0U;
    uint8_t flashword_buf[32];
    const uint8_t *src;

    if ((data == NULL) || (size == 0U)) {
        return PARAM_ERR_WRITE;
    }

    src = (const uint8_t *)data;

    /* STM32H7 Flash编程：每次写入32字节（256位） */
    while (offset < size) {
        uint32_t write_addr = addr + offset;
        uint32_t remain = size - offset;
        uint32_t chunk = (remain >= sizeof(flashword_buf)) ? sizeof(flashword_buf) : remain;

        /* 最后一个块不足32字节时，用擦除态0xFF填充，避免越界读取源缓冲区 */
        memset(flashword_buf, 0xFF, sizeof(flashword_buf));
        memcpy(flashword_buf, &src[offset], chunk);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, write_addr, (uint32_t)flashword_buf) != HAL_OK) {
            return PARAM_ERR_WRITE;
        }

        if (memcmp((void *)write_addr, flashword_buf, sizeof(flashword_buf)) != 0) {
            return PARAM_ERR_WRITE;
        }

        offset += sizeof(flashword_buf);
    }

    return PARAM_OK;
}
