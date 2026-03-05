/**
 * @file    drv8350s.c
 * @brief   DRV8350S driver implementation for STM32H743
 * @note    SPI1 + DMA, synchronized with TIM1 20kHz update interrupt
 */

#include "drv8350s.h"
#include <string.h>

/* Private Macros ------------------------------------------------------------*/
#define DRV8350S_SPI_TIMEOUT_MS         100U
#define DRV8350S_READ_BIT               0x8000U
#define DRV8350S_WRITE_MASK             0x7FFFU
#define DRV8350S_ADDR_SHIFT             11U
#define DRV8350S_DATA_MASK              0x07FFU

#define NSCS_LOW(handle)    HAL_GPIO_WritePin((handle)->nscsPort, (handle)->nscsPin, GPIO_PIN_RESET)
#define NSCS_HIGH(handle)   HAL_GPIO_WritePin((handle)->nscsPort, (handle)->nscsPin, GPIO_PIN_SET)

/* Static Function Prototypes ------------------------------------------------*/
static inline uint16_t DRV8350S_BuildWriteFrame(uint8_t addr, uint16_t data);
static inline uint16_t DRV8350S_BuildReadFrame(uint8_t addr);
static inline uint16_t DRV8350S_ParseResponse(uint16_t rxData);
static void DRV8350S_ParseFaultStatus(DRV8350S_Handle_t* handle);
static int8_t DRV8350S_BusLock(DRV8350S_Handle_t* handle, uint32_t timeoutMs);
static void DRV8350S_BusUnlock(DRV8350S_Handle_t* handle);
static int8_t DRV8350S_WriteRegisterUnlocked(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t data);
static int8_t DRV8350S_ReadRegisterUnlocked(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t* data);

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief Initialize DRV8350S driver
 */
int8_t DRV8350S_Init(DRV8350S_Handle_t* handle,
                     SPI_HandleTypeDef* hspi,
                     TIM_HandleTypeDef* htim,
                     GPIO_TypeDef* nscsPort,
                     uint16_t nscsPin)
{
    if (handle == NULL || hspi == NULL || htim == NULL) {
        return -1;
    }

    memset(handle, 0, sizeof(DRV8350S_Handle_t));

    handle->hspi = hspi;
    handle->htim = htim;
    handle->nscsPort = nscsPort;
    handle->nscsPin = nscsPin;

    /* Ensure chip select is high (inactive) */
    NSCS_HIGH(handle);

    /* Clear DMA busy flag */
    handle->runtime.dmaBusy = 0;
    handle->runtime.syncBusy = 0;

    return 0;
}

/**
 * @brief Deinitialize DRV8350S driver
 */
int8_t DRV8350S_DeInit(DRV8350S_Handle_t* handle)
{
    if (handle == NULL) {
        return -1;
    }

    /* Abort any ongoing DMA transfer */
    HAL_SPI_DMAStop(handle->hspi);
    NSCS_HIGH(handle);

    memset(handle, 0, sizeof(DRV8350S_Handle_t));

    return 0;
}

/**
 * @brief Set default configuration
 */
void DRV8350S_SetDefaultConfig(DRV8350S_Config_t* config)
{
    if (config == NULL) return;

    memset(config, 0, sizeof(DRV8350S_Config_t));

    /* Default: 6x PWM mode */
    config->pwmMode = DRV8350S_PWM_MODE_6X;

    /* Default gate drive: 1A source, 2A sink (maximum) */
    config->idriveP_hs = DRV8350S_IDRIVE_1000MA;
    config->idriveN_hs = DRV8350S_IDRIVE_1000MA;
    config->idriveP_ls = DRV8350S_IDRIVE_1000MA;
    config->idriveN_ls = DRV8350S_IDRIVE_1000MA;

    /* Default TDRIVE: 4us (safest) */
    config->tdrive = DRV8350S_TDRIVE_4000NS;

    /* Default protection: retry mode */
    config->ocpMode = DRV8350S_OCP_MODE_RETRY;
    config->ocpDeglitch = DRV8350S_OCP_DEG_4US;
    config->vdsLvl = 0x09;      /* 0.6V default */
    config->deadTime = DRV8350S_DEAD_TIME_100NS;
    config->tretry = DRV8350S_TRETRY_50US;

    /* Default functional settings */
    config->otwReport = 1;      /* Report OTW */
    config->disGdf = 0;         /* Enable GDF */
    config->disGdUv = 0;        /* Enable GD UVLO */
    config->ocpAct = 0;         /* Shutdown associated half-bridge only */
}

/**
 * @brief Configure DRV8350S with given settings
 * @note This uses blocking SPI calls, call during initialization only
 */
int8_t DRV8350S_Configure(DRV8350S_Handle_t* handle, const DRV8350S_Config_t* config)
{
    int8_t status = 0;
    uint16_t regVal;

    if (handle == NULL || config == NULL) {
        return -1;
    }

    /* Save configuration */
    memcpy(&handle->config, config, sizeof(DRV8350S_Config_t));

    /* First, unlock registers */
    status |= DRV8350S_UnlockRegisters(handle);

    /* Configure Driver Control Register (0x02) */
    regVal = ((config->ocpAct & 0x01) << DRV8350S_OCP_ACT_POS) |
             ((config->disGdUv & 0x01) << DRV8350S_DIS_GDUV_POS) |
             ((config->disGdf & 0x01) << DRV8350S_DIS_GDF_POS) |
             ((config->otwReport & 0x01) << DRV8350S_OTW_REP_POS) |
             ((config->pwmMode & 0x03) << DRV8350S_PWM_MODE_POS) |
             (0 << DRV8350S_1PWM_COM_POS) |  /* Synchronous rectification */
             (0 << DRV8350S_1PWM_DIR_POS);   /* Direction bit */
    status |= DRV8350S_WriteRegister(handle, DRV8350S_REG_DRIVER_CTRL, regVal);

    /* Configure Gate Drive HS Register (0x03) */
    regVal = ((DRV8350S_LOCK_KEY_UNLOCK & 0x07) << DRV8350S_LOCK_POS) |
             ((config->idriveP_hs & 0x0F) << DRV8350S_IDRIVEP_HS_POS) |
             ((config->idriveN_hs & 0x0F) << DRV8350S_IDRIVEN_HS_POS);
    status |= DRV8350S_WriteRegister(handle, DRV8350S_REG_GATE_DRIVE_HS, regVal);

    /* Configure Gate Drive LS Register (0x04) */
    regVal = ((1 & 0x01) << DRV8350S_CBC_POS) |  /* CBC enabled */
             ((config->tdrive & 0x03) << DRV8350S_TDRIVE_POS) |
             ((config->idriveP_ls & 0x0F) << DRV8350S_IDRIVEP_LS_POS) |
             ((config->idriveN_ls & 0x0F) << DRV8350S_IDRIVEN_LS_POS);
    status |= DRV8350S_WriteRegister(handle, DRV8350S_REG_GATE_DRIVE_LS, regVal);

    /* Configure OCP Control Register (0x05) */
    regVal = ((config->tretry & 0x01) << DRV8350S_TRETRY_POS) |
             ((config->deadTime & 0x03) << DRV8350S_DEAD_TIME_POS) |
             ((config->ocpMode & 0x03) << DRV8350S_OCP_MODE_POS) |
             ((config->ocpDeglitch & 0x03) << DRV8350S_OCP_DEG_POS) |
             ((config->vdsLvl & 0x0F) << DRV8350S_VDS_LVL_POS);
    status |= DRV8350S_WriteRegister(handle, DRV8350S_REG_OCP_CTRL, regVal);

    /* Lock registers after configuration */
    // status |= DRV8350S_LockRegisters(handle);  /* Optional: lock after config */

    /* Clear any existing faults */
    status |= DRV8350S_ClearFaults(handle);

    return status;
}

/**
 * @brief Lock SPI registers
 */
int8_t DRV8350S_LockRegisters(DRV8350S_Handle_t* handle)
{
    uint16_t regVal;

    /* Read current value */
    if (DRV8350S_ReadRegister(handle, DRV8350S_REG_GATE_DRIVE_HS, &regVal) != 0) {
        return -1;
    }

    /* Set lock bits */
    regVal &= ~(0x07 << DRV8350S_LOCK_POS);
    regVal |= (DRV8350S_LOCK_KEY_LOCK << DRV8350S_LOCK_POS);

    return DRV8350S_WriteRegister(handle, DRV8350S_REG_GATE_DRIVE_HS, regVal);
}

/**
 * @brief Unlock SPI registers
 */
int8_t DRV8350S_UnlockRegisters(DRV8350S_Handle_t* handle)
{
    uint16_t regVal;

    /* Read current value */
    if (DRV8350S_ReadRegister(handle, DRV8350S_REG_GATE_DRIVE_HS, &regVal) != 0) {
        return -1;
    }

    /* Set unlock bits */
    regVal &= ~(0x07 << DRV8350S_LOCK_POS);
    regVal |= (DRV8350S_LOCK_KEY_UNLOCK << DRV8350S_LOCK_POS);

    return DRV8350S_WriteRegister(handle, DRV8350S_REG_GATE_DRIVE_HS, regVal);
}

/**
 * @brief Write register (blocking, for initialization only)
 */
int8_t DRV8350S_WriteRegister(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t data)
{
    if (handle == NULL || regAddr > 5) {
        return -1;
    }

    if (DRV8350S_BusLock(handle, DRV8350S_SPI_TIMEOUT_MS) != 0) {
        handle->runtime.errorCount++;
        return -1;
    }

    {
        int8_t ret = DRV8350S_WriteRegisterUnlocked(handle, regAddr, data);
        DRV8350S_BusUnlock(handle);
        return ret;
    }
}

/**
 * @brief Read register (blocking, for initialization only)
 */
int8_t DRV8350S_ReadRegister(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t* data)
{
    if (handle == NULL || regAddr > 5 || data == NULL) {
        return -1;
    }

    if (DRV8350S_BusLock(handle, DRV8350S_SPI_TIMEOUT_MS) != 0) {
        handle->runtime.errorCount++;
        return -1;
    }

    {
        int8_t ret = DRV8350S_ReadRegisterUnlocked(handle, regAddr, data);
        DRV8350S_BusUnlock(handle);
        return ret;
    }
}

/**
 * @brief Trigger asynchronous read of a specific register
 * @note Call this from TIM1 Update interrupt at 20kHz
 */
int8_t DRV8350S_TriggerAsyncRead(DRV8350S_Handle_t* handle, uint8_t regAddr)
{
    if (handle == NULL || regAddr > 5) {
        return -1;
    }

    /* 阻塞式SPI访问持锁期间，异步轮询让路，避免总线竞争 */
    if (handle->runtime.syncBusy) {
        return -2;
    }

    /* Check if previous DMA transfer is still ongoing */
    if (handle->runtime.dmaBusy) {
        handle->runtime.errorCount++;
        return -1;  /* Still busy */
    }

    /* Set up read request */
    handle->readReq.registerAddr = regAddr;
    handle->readReq.pending = 1;

    /* Prepare TX buffer for read command */
    handle->txBuf[0] = DRV8350S_BuildReadFrame(regAddr);
    handle->txBuf[1] = 0x0000;  /* NOP to get response */

    /* Assert chip select */
    NSCS_LOW(handle);

    /* Start DMA transfer: 2 words (command + NOP) */
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        handle->hspi,
        (uint8_t*)handle->txBuf,
        (uint8_t*)handle->rxBuf,
        2  /* 2 x 16-bit transfers */
    );

    if (status == HAL_OK) {
        handle->runtime.dmaBusy = 1;
        handle->runtime.commCount++;
        return 0;
    } else {
        NSCS_HIGH(handle);
        handle->readReq.pending = 0;
        handle->runtime.spiError = 1;
        handle->runtime.errorCount++;
        return -1;
    }
}

/**
 * @brief Trigger asynchronous read of status registers (recommended for 20kHz loop)
 * @note Reads FAULT_STATUS_1 and VGS_STATUS_2 in sequence
 */
int8_t DRV8350S_TriggerAsyncReadAll(DRV8350S_Handle_t* handle)
{
    int8_t ret;

    /* For continuous monitoring, alternate between fault status registers */
    static uint8_t toggle = 0;

    if (toggle) {
        ret = DRV8350S_TriggerAsyncRead(handle, DRV8350S_REG_FAULT_STATUS_1);
    } else {
        ret = DRV8350S_TriggerAsyncRead(handle, DRV8350S_REG_VGS_STATUS_2);
    }
    if (ret == 0) {
        toggle ^= 1;
    }

    return ret;
}

/**
 * @brief TIM1 Update Event Callback - Call this from TIM1_IRQHandler
 * @note This should be called at 20kHz PWM update rate
 *
 * Example usage in stm32h7xx_it.c:
 *
 * void TIM1_UP_IRQHandler(void)
 * {
 *     if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)) {
 *         __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
 *
 *         // Your motor control code here...
 *
 *         // Trigger DRV8350S status read via DMA
 *         DRV8350S_TIM1_UpdateCallback(&drvHandle);
 *     }
 * }
 */
void DRV8350S_TIM1_UpdateCallback(DRV8350S_Handle_t* handle)
{
    if (handle == NULL) return;

    /* Only trigger new read if previous one completed */
    if (!handle->runtime.dmaBusy) {
        /* Read fault status registers alternately */
        static uint8_t readSequence = 0;
        int8_t ret = 0;

        switch (readSequence) {
            case 0:
                ret = DRV8350S_TriggerAsyncRead(handle, DRV8350S_REG_FAULT_STATUS_1);
                break;
            case 1:
                ret = DRV8350S_TriggerAsyncRead(handle, DRV8350S_REG_VGS_STATUS_2);
                break;
            case 2:
                ret = DRV8350S_TriggerAsyncRead(handle, DRV8350S_REG_DRIVER_CTRL);
                break;
            case 3:
                ret = DRV8350S_TriggerAsyncRead(handle, DRV8350S_REG_OCP_CTRL);
                break;
            default:
                readSequence = 0;
                break;
        }
        if (ret == 0) {
            readSequence = (readSequence + 1U) % 4U;
        }
    }
    /* If still busy, we missed a cycle - count as error but don't block */
    else {
        handle->runtime.errorCount++;
    }
}

/**
 * @brief DMA Complete Callback - Call from HAL_SPI_TxRxCpltCallback
 *
 * Example usage:
 *
 * void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
 * {
 *     if (hspi == &hspi1) {
 *         DRV8350S_DMA_CompleteCallback(&drvHandle);
 *     }
 * }
 */
void DRV8350S_DMA_CompleteCallback(DRV8350S_Handle_t* handle)
{
    if (handle == NULL) return;

    /* Deassert chip select */
    NSCS_HIGH(handle);

    /* Release SPI bus */
    handle->runtime.dmaBusy = 0;

    /* Parse received data */
    /* rxBuf[0] contains previous register data (don't care for first read) */
    /* rxBuf[1] contains actual response data */
    uint16_t response = DRV8350S_ParseResponse(handle->rxBuf[1]);

    /* Store based on which register was read */
    switch (handle->readReq.registerAddr) {
        case DRV8350S_REG_FAULT_STATUS_1:
            handle->runtime.regFaultStatus1 = response;
            /* Parse faults immediately */
            DRV8350S_ParseFaultStatus(handle);
            break;
        case DRV8350S_REG_VGS_STATUS_2:
            handle->runtime.regVgsStatus2 = response;
            break;
        case DRV8350S_REG_DRIVER_CTRL:
            handle->runtime.regDriverCtrl = response;
            break;
        case DRV8350S_REG_GATE_DRIVE_HS:
            handle->runtime.regGateDriveHs = response;
            break;
        case DRV8350S_REG_GATE_DRIVE_LS:
            handle->runtime.regGateDriveLs = response;
            break;
        case DRV8350S_REG_OCP_CTRL:
            handle->runtime.regOcpCtrl = response;
            break;
        default:
            break;
    }

    /* Invoke callback if registered */
    if (handle->readCompleteCallback != NULL) {
        handle->readCompleteCallback(handle->readReq.registerAddr, response);
    }

    /* Invoke fault callback if fault detected */
    if (handle->runtime.isFaultActive && handle->faultCallback != NULL) {
        handle->faultCallback(handle->runtime.faultFlags);
    }

    handle->readReq.pending = 0;
}

/**
 * @brief DMA Error Callback - Call from HAL_SPI_ErrorCallback
 */
void DRV8350S_DMA_ErrorCallback(DRV8350S_Handle_t* handle)
{
    if (handle == NULL) return;

    /* Deassert chip select */
    NSCS_HIGH(handle);

    /* Release SPI bus and mark error */
    handle->runtime.dmaBusy = 0;
    handle->readReq.pending = 0;
    handle->runtime.spiError = 1;
    handle->runtime.errorCount++;

    /* Abort DMA to clean state */
    HAL_SPI_DMAStop(handle->hspi);
}

/**
 * @brief Clear all latched faults
 */
int8_t DRV8350S_ClearFaults(DRV8350S_Handle_t* handle)
{
    uint16_t regVal;
    int8_t ret = -1;

    if (handle == NULL) {
        return -1;
    }

    if (DRV8350S_BusLock(handle, DRV8350S_SPI_TIMEOUT_MS) != 0) {
        handle->runtime.errorCount++;
        return -1;
    }

    if (DRV8350S_ReadRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, &regVal) == 0) {
        regVal |= (1U << DRV8350S_CLR_FLT_POS);
        ret = DRV8350S_WriteRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, regVal);
    }

    DRV8350S_BusUnlock(handle);
    return ret;
}

/**
 * @brief Enable gate drivers (clear COAST/STOP condition)
 */
int8_t DRV8350S_EnableGateDrivers(DRV8350S_Handle_t* handle)
{
    uint16_t regVal;
    int8_t ret = -1;

    if (handle == NULL) {
        return -1;
    }

    if (DRV8350S_BusLock(handle, DRV8350S_SPI_TIMEOUT_MS) != 0) {
        handle->runtime.errorCount++;
        return -1;
    }

    if (DRV8350S_ReadRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, &regVal) == 0) {
        regVal &= ~(1U << DRV8350S_COAST_POS);
        regVal &= ~(1U << DRV8350S_BRAKE_POS);
        ret = DRV8350S_WriteRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, regVal);
    }

    DRV8350S_BusUnlock(handle);
    return ret;
}

/**
 * @brief Disable gate drivers (COAST - Hi-Z all MOSFETs)
 */
int8_t DRV8350S_DisableGateDrivers(DRV8350S_Handle_t* handle)
{
    uint16_t regVal;
    int8_t ret = -1;

    if (handle == NULL) {
        return -1;
    }

    if (DRV8350S_BusLock(handle, DRV8350S_SPI_TIMEOUT_MS) != 0) {
        handle->runtime.errorCount++;
        return -1;
    }

    if (DRV8350S_ReadRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, &regVal) == 0) {
        regVal |= (1U << DRV8350S_COAST_POS);
        regVal &= ~(1U << DRV8350S_BRAKE_POS);
        ret = DRV8350S_WriteRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, regVal);
    }

    DRV8350S_BusUnlock(handle);
    return ret;
}

/**
 * @brief Set brake mode (all low-side MOSFETs on)
 */
int8_t DRV8350S_SetBrake(DRV8350S_Handle_t* handle)
{
    uint16_t regVal;
    int8_t ret = -1;

    if (handle == NULL) {
        return -1;
    }

    if (DRV8350S_BusLock(handle, DRV8350S_SPI_TIMEOUT_MS) != 0) {
        handle->runtime.errorCount++;
        return -1;
    }

    if (DRV8350S_ReadRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, &regVal) == 0) {
        regVal |= (1U << DRV8350S_BRAKE_POS);
        regVal &= ~(1U << DRV8350S_COAST_POS);
        ret = DRV8350S_WriteRegisterUnlocked(handle, DRV8350S_REG_DRIVER_CTRL, regVal);
    }

    DRV8350S_BusUnlock(handle);
    return ret;
}

/**
 * @brief Set coast mode (Hi-Z all MOSFETs)
 */
int8_t DRV8350S_SetCoast(DRV8350S_Handle_t* handle)
{
    return DRV8350S_DisableGateDrivers(handle);
}

/**
 * @brief Get current fault flags
 */
uint32_t DRV8350S_GetFaultFlags(DRV8350S_Handle_t* handle)
{
    if (handle == NULL) return 0xFFFFFFFFU;

    return handle->runtime.faultFlags;
}

/**
 * @brief Convert fault bit to string description
 */
const char* DRV8350S_FaultToString(uint32_t faultBit)
{
    switch (faultBit) {
        /* Fault Status Register 1 bits (0x00) */
        case DRV8350S_FAULT_BIT:        return "General Fault";
        case DRV8350S_VDS_OCP_BIT:      return "VDS Overcurrent";
        case DRV8350S_GDF_BIT:          return "Gate Drive Fault";
        case DRV8350S_UVLO_BIT:         return "Undervoltage Lockout";
        case DRV8350S_OTSD_BIT:         return "Overtemperature Shutdown";
        case DRV8350S_VDS_HA_BIT:       return "VDS OCP Phase A High";
        case DRV8350S_VDS_LA_BIT:       return "VDS OCP Phase A Low";
        case DRV8350S_VDS_HB_BIT:       return "VDS OCP Phase B High";
        case DRV8350S_VDS_LB_BIT:       return "VDS OCP Phase B Low";
        case DRV8350S_VDS_HC_BIT:       return "VDS OCP Phase C High";
        case DRV8350S_VDS_LC_BIT:       return "VDS OCP Phase C Low";
        
        /* VGS Status Register 2 bits (0x01) - use upper 16 bits to distinguish */
        case (uint32_t)DRV8350S_OTW_BIT:          return "Overtemperature Warning";
        case (uint32_t)DRV8350S_GDUV_BIT:         return "Gate Drive Undervoltage";
        case (uint32_t)DRV8350S_VGS_HA_BIT:       return "VGS Fault Phase A High";
        case (uint32_t)DRV8350S_VGS_LA_BIT:       return "VGS Fault Phase A Low";
        case (uint32_t)DRV8350S_VGS_HB_BIT:       return "VGS Fault Phase B High";
        case (uint32_t)DRV8350S_VGS_LB_BIT:       return "VGS Fault Phase B Low";
        case (uint32_t)DRV8350S_VGS_HC_BIT:       return "VGS Fault Phase C High";
        case (uint32_t)DRV8350S_VGS_LC_BIT:       return "VGS Fault Phase C Low";
        
        default:                        return "Unknown Fault";
    }
}


/* Private Functions ---------------------------------------------------------*/

/**
 * @brief 获取SPI1总线独占权，并等待异步DMA事务退出
 */
static int8_t DRV8350S_BusLock(DRV8350S_Handle_t* handle, uint32_t timeoutMs)
{
    uint32_t startTick;

    if (handle == NULL) {
        return -1;
    }

    startTick = HAL_GetTick();

    while (1) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (handle->runtime.syncBusy == 0U) {
            handle->runtime.syncBusy = 1U;
            if (primask == 0U) {
                __enable_irq();
            }
            break;
        }
        if (primask == 0U) {
            __enable_irq();
        }

        if ((HAL_GetTick() - startTick) > timeoutMs) {
            return -1;
        }
    }

    while ((handle->runtime.dmaBusy != 0U) || (handle->readReq.pending != 0U)) {
        if ((HAL_GetTick() - startTick) > timeoutMs) {
            DRV8350S_BusUnlock(handle);
            return -1;
        }
    }

    return 0;
}

/**
 * @brief 释放SPI1总线独占权
 */
static void DRV8350S_BusUnlock(DRV8350S_Handle_t* handle)
{
    uint32_t primask;

    if (handle == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    handle->runtime.syncBusy = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 无锁写寄存器（调用方需先持有总线锁）
 */
static int8_t DRV8350S_WriteRegisterUnlocked(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t data)
{
    uint16_t txFrame, rxDummy;
    HAL_StatusTypeDef halStatus;

    if (handle == NULL || regAddr > 5U) {
        return -1;
    }

    txFrame = DRV8350S_BuildWriteFrame(regAddr, data);

    NSCS_LOW(handle);
    halStatus = HAL_SPI_TransmitReceive(
        handle->hspi,
        (uint8_t*)&txFrame,
        (uint8_t*)&rxDummy,
        1,
        DRV8350S_SPI_TIMEOUT_MS
    );

    /* t_READY = 1ms (datasheet max) */
    HAL_Delay(1);
    NSCS_HIGH(handle);

    return (halStatus == HAL_OK) ? 0 : -1;
}

/**
 * @brief 无锁读寄存器（调用方需先持有总线锁）
 */
static int8_t DRV8350S_ReadRegisterUnlocked(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t* data)
{
    uint16_t txFrame, rxFrame;
    HAL_StatusTypeDef halStatus;

    if (handle == NULL || regAddr > 5U || data == NULL) {
        return -1;
    }

    txFrame = DRV8350S_BuildReadFrame(regAddr);
    NSCS_LOW(handle);
    halStatus = HAL_SPI_TransmitReceive(
        handle->hspi,
        (uint8_t*)&txFrame,
        (uint8_t*)&rxFrame,
        1,
        DRV8350S_SPI_TIMEOUT_MS
    );

    NSCS_HIGH(handle);
    __NOP(); __NOP(); __NOP(); __NOP();

    txFrame = 0x0000U;
    NSCS_LOW(handle);
    halStatus |= HAL_SPI_TransmitReceive(
        handle->hspi,
        (uint8_t*)&txFrame,
        (uint8_t*)&rxFrame,
        1,
        DRV8350S_SPI_TIMEOUT_MS
    );
    NSCS_HIGH(handle);

    *data = DRV8350S_ParseResponse(rxFrame);
    return (halStatus == HAL_OK) ? 0 : -1;
}

/**
 * @brief Build SPI write frame
 */
static inline uint16_t DRV8350S_BuildWriteFrame(uint8_t addr, uint16_t data)
{
    /* W=0, Address[3:0], Data[10:0] */
    return ((addr & 0x0F) << DRV8350S_ADDR_SHIFT) | (data & DRV8350S_DATA_MASK);
}

/**
 * @brief Build SPI read frame
 */
static inline uint16_t DRV8350S_BuildReadFrame(uint8_t addr)
{
    /* W=1, Address[3:0], Data[10:0]=0 */
    return DRV8350S_READ_BIT | ((addr & 0x0F) << DRV8350S_ADDR_SHIFT);
}

/**
 * @brief Parse SPI response (extract data bits)
 */
static inline uint16_t DRV8350S_ParseResponse(uint16_t rxData)
{
    /* Response: Don't care[15:11], Data[10:0] */
    return rxData & DRV8350S_DATA_MASK;
}

/**
 * @brief Parse fault status and update runtime flags
 */
static void DRV8350S_ParseFaultStatus(DRV8350S_Handle_t* handle)
{
    uint32_t faults = 0;
    uint16_t fs1 = handle->runtime.regFaultStatus1;
    uint16_t fs2 = handle->runtime.regVgsStatus2;

    /* Fault Status 1 */
    if (fs1 & (1U << 10)) faults |= DRV8350S_FAULT_BIT;
    if (fs1 & (1U << 9))  faults |= DRV8350S_VDS_OCP_BIT;
    if (fs1 & (1U << 8))  faults |= DRV8350S_GDF_BIT;
    if (fs1 & (1U << 7))  faults |= DRV8350S_UVLO_BIT;
    if (fs1 & (1U << 6))  faults |= DRV8350S_OTSD_BIT;
    if (fs1 & (1U << 5))  faults |= DRV8350S_VDS_HA_BIT;
    if (fs1 & (1U << 4))  faults |= DRV8350S_VDS_LA_BIT;
    if (fs1 & (1U << 3))  faults |= DRV8350S_VDS_HB_BIT;
    if (fs1 & (1U << 2))  faults |= DRV8350S_VDS_LB_BIT;
    if (fs1 & (1U << 1))  faults |= DRV8350S_VDS_HC_BIT;
    if (fs1 & (1U << 0))  faults |= DRV8350S_VDS_LC_BIT;

    /* VGS Status 2 */
    /* Note: DRV8350S does NOT have CSA, Bit 10-8 are Reserved */
    if (fs2 & (1U << 7))  faults |= DRV8350S_OTW_BIT;
    if (fs2 & (1U << 6))  faults |= DRV8350S_GDUV_BIT;
    if (fs2 & (1U << 5))  faults |= DRV8350S_VGS_HA_BIT;
    if (fs2 & (1U << 4))  faults |= DRV8350S_VGS_LA_BIT;
    if (fs2 & (1U << 3))  faults |= DRV8350S_VGS_HB_BIT;
    if (fs2 & (1U << 2))  faults |= DRV8350S_VGS_LB_BIT;
    if (fs2 & (1U << 1))  faults |= DRV8350S_VGS_HC_BIT;
    if (fs2 & (1U << 0))  faults |= DRV8350S_VGS_LC_BIT;

    handle->runtime.faultFlags = faults;
    handle->runtime.isFaultActive = (faults != 0);
}
