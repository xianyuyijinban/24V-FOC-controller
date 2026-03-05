/**
 * @file    drv8350s.h
 * @brief   DRV8350S Three-Phase Smart Gate Driver driver for STM32H743
 * @note    SPI1 + DMA, synchronized with TIM1 20kHz update interrupt
 */

#ifndef __DRV8350S_H
#define __DRV8350S_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* DRV8350S Register Addresses -----------------------------------------------*/
#define DRV8350S_REG_FAULT_STATUS_1     0x00U
#define DRV8350S_REG_VGS_STATUS_2       0x01U
#define DRV8350S_REG_DRIVER_CTRL        0x02U
#define DRV8350S_REG_GATE_DRIVE_HS      0x03U
#define DRV8350S_REG_GATE_DRIVE_LS      0x04U
#define DRV8350S_REG_OCP_CTRL           0x05U

/* Register Count ------------------------------------------------------------*/
#define DRV8350S_REG_COUNT              6U

/* Fault Status Register 1 (0x00) Bit Definitions -----------------------------*/
#define DRV8350S_FAULT_BIT              (1U << 10)
#define DRV8350S_VDS_OCP_BIT            (1U << 9)
#define DRV8350S_GDF_BIT                (1U << 8)
#define DRV8350S_UVLO_BIT               (1U << 7)
#define DRV8350S_OTSD_BIT               (1U << 6)
#define DRV8350S_VDS_HA_BIT             (1U << 5)
#define DRV8350S_VDS_LA_BIT             (1U << 4)
#define DRV8350S_VDS_HB_BIT             (1U << 3)
#define DRV8350S_VDS_LB_BIT             (1U << 2)
#define DRV8350S_VDS_HC_BIT             (1U << 1)
#define DRV8350S_VDS_LC_BIT             (1U << 0)

/* VGS Status Register 2 (0x01) Bit Definitions -------------------------------*/
/* Shift by 16 to avoid conflict with FAULT_STATUS_1 bits in combined fault flags */
#define DRV8350S_VGS_STATUS_SHIFT       16U

/* DRV8350S does NOT have CSA (Current Sense Amplifier), Bit 10-8 are Reserved */
/* #define DRV8350S_SA_OC_BIT   - Not applicable for DRV8350S */
/* #define DRV8350S_SB_OC_BIT   - Not applicable for DRV8350S */
/* #define DRV8350S_SC_OC_BIT   - Not applicable for DRV8350S */
#define DRV8350S_OTW_BIT                ((uint32_t)(1U << 7)  << DRV8350S_VGS_STATUS_SHIFT)
#define DRV8350S_GDUV_BIT               ((uint32_t)(1U << 6)  << DRV8350S_VGS_STATUS_SHIFT)
#define DRV8350S_VGS_HA_BIT             ((uint32_t)(1U << 5)  << DRV8350S_VGS_STATUS_SHIFT)
#define DRV8350S_VGS_LA_BIT             ((uint32_t)(1U << 4)  << DRV8350S_VGS_STATUS_SHIFT)
#define DRV8350S_VGS_HB_BIT             ((uint32_t)(1U << 3)  << DRV8350S_VGS_STATUS_SHIFT)
#define DRV8350S_VGS_LB_BIT             ((uint32_t)(1U << 2)  << DRV8350S_VGS_STATUS_SHIFT)
#define DRV8350S_VGS_HC_BIT             ((uint32_t)(1U << 1)  << DRV8350S_VGS_STATUS_SHIFT)
#define DRV8350S_VGS_LC_BIT             ((uint32_t)(1U << 0)  << DRV8350S_VGS_STATUS_SHIFT)

/* Raw register bit masks for parsing (without shift) */
/* DRV8350S: Bit 10-8 are Reserved, not CSA Overcurrent */
#define DRV8350S_VGS_OTW_MASK           (1U << 7)
#define DRV8350S_VGS_GDUV_MASK          (1U << 6)
#define DRV8350S_VGS_HA_MASK            (1U << 5)
#define DRV8350S_VGS_LA_MASK            (1U << 4)
#define DRV8350S_VGS_HB_MASK            (1U << 3)
#define DRV8350S_VGS_LB_MASK            (1U << 2)
#define DRV8350S_VGS_HC_MASK            (1U << 1)
#define DRV8350S_VGS_LC_MASK            (1U << 0)

/* Driver Control Register (0x02) Bit Definitions -----------------------------*/
#define DRV8350S_OCP_ACT_POS            10U
#define DRV8350S_DIS_GDUV_POS           9U
#define DRV8350S_DIS_GDF_POS            8U
#define DRV8350S_OTW_REP_POS            7U
#define DRV8350S_PWM_MODE_POS           5U
#define DRV8350S_1PWM_COM_POS           4U
#define DRV8350S_1PWM_DIR_POS           3U
#define DRV8350S_COAST_POS              2U
#define DRV8350S_BRAKE_POS              1U
#define DRV8350S_CLR_FLT_POS            0U

#define DRV8350S_PWM_MODE_6X            0U
#define DRV8350S_PWM_MODE_3X            1U
#define DRV8350S_PWM_MODE_1X            2U
#define DRV8350S_PWM_MODE_INDEPENDENT   3U

/* Gate Drive HS Register (0x03) Bit Definitions ------------------------------*/
#define DRV8350S_LOCK_POS               8U
#define DRV8350S_IDRIVEP_HS_POS         4U
#define DRV8350S_IDRIVEN_HS_POS         0U

#define DRV8350S_LOCK_KEY_UNLOCK        0x03U
#define DRV8350S_LOCK_KEY_LOCK          0x06U

/* IDRIVE current settings (mA) */
typedef enum {
    DRV8350S_IDRIVE_50MA    = 0x00,
    DRV8350S_IDRIVE_100MA   = 0x02,
    DRV8350S_IDRIVE_150MA   = 0x03,
    DRV8350S_IDRIVE_300MA   = 0x04,
    DRV8350S_IDRIVE_350MA   = 0x05,
    DRV8350S_IDRIVE_400MA   = 0x06,
    DRV8350S_IDRIVE_450MA   = 0x07,
    DRV8350S_IDRIVE_550MA   = 0x08,
    DRV8350S_IDRIVE_600MA   = 0x09,
    DRV8350S_IDRIVE_650MA   = 0x0A,
    DRV8350S_IDRIVE_700MA   = 0x0B,
    DRV8350S_IDRIVE_850MA   = 0x0C,
    DRV8350S_IDRIVE_900MA   = 0x0D,
    DRV8350S_IDRIVE_950MA   = 0x0E,
    DRV8350S_IDRIVE_1000MA  = 0x0F
} DRV8350S_IDrive_t;

/* Gate Drive LS Register (0x04) Bit Definitions ------------------------------*/
#define DRV8350S_CBC_POS                10U
#define DRV8350S_TDRIVE_POS             8U
#define DRV8350S_IDRIVEP_LS_POS         4U
#define DRV8350S_IDRIVEN_LS_POS         0U

#define DRV8350S_TDRIVE_500NS           0U
#define DRV8350S_TDRIVE_1000NS          1U
#define DRV8350S_TDRIVE_2000NS          2U
#define DRV8350S_TDRIVE_4000NS          3U

/* OCP Control Register (0x05) Bit Definitions --------------------------------*/
#define DRV8350S_TRETRY_POS             10U
#define DRV8350S_DEAD_TIME_POS          8U
#define DRV8350S_OCP_MODE_POS           6U
#define DRV8350S_OCP_DEG_POS            4U
#define DRV8350S_VDS_LVL_POS            0U

#define DRV8350S_TRETRY_8MS             0U
#define DRV8350S_TRETRY_50US            1U

#define DRV8350S_DEAD_TIME_50NS         0U
#define DRV8350S_DEAD_TIME_100NS        1U
#define DRV8350S_DEAD_TIME_200NS        2U
#define DRV8350S_DEAD_TIME_400NS        3U

#define DRV8350S_OCP_MODE_LATCHED       0U
#define DRV8350S_OCP_MODE_RETRY         1U
#define DRV8350S_OCP_MODE_REPORT_ONLY   2U
#define DRV8350S_OCP_MODE_DISABLE       3U

#define DRV8350S_OCP_DEG_1US            0U
#define DRV8350S_OCP_DEG_2US            1U
#define DRV8350S_OCP_DEG_4US            2U
#define DRV8350S_OCP_DEG_8US            3U

/* Configuration Structure ---------------------------------------------------*/
typedef struct {
    /* PWM Mode */
    uint8_t pwmMode;                    /* 6x, 3x, 1x, or independent */

    /* Gate Drive Settings */
    DRV8350S_IDrive_t idriveP_hs;       /* High-side peak source current */
    DRV8350S_IDrive_t idriveN_hs;       /* High-side peak sink current */
    DRV8350S_IDrive_t idriveP_ls;       /* Low-side peak source current */
    DRV8350S_IDrive_t idriveN_ls;       /* Low-side peak sink current */
    uint8_t tdrive;                     /* Peak gate-current drive time */

    /* Protection Settings */
    uint8_t ocpMode;                    /* Overcurrent protection mode */
    uint8_t ocpDeglitch;                /* OCP deglitch time */
    uint8_t vdsLvl;                     /* VDS overcurrent threshold */
    uint8_t deadTime;                   /* Dead time insertion */
    uint8_t tretry;                     /* Retry time */

    /* Functional Settings */
    uint8_t otwReport;                  /* Report overtemperature warning */
    uint8_t disGdf;                     /* Disable gate drive fault */
    uint8_t disGdUv;                    /* Disable GD undervoltage fault */
    uint8_t ocpAct;                     /* OCP action on all bridges */

} DRV8350S_Config_t;

/* Runtime Data Structure ----------------------------------------------------*/
typedef struct {
    /* Raw register values (updated by DMA) */
    volatile uint16_t regFaultStatus1;  /* 0x00 */
    volatile uint16_t regVgsStatus2;    /* 0x01 */
    volatile uint16_t regDriverCtrl;    /* 0x02 */
    volatile uint16_t regGateDriveHs;   /* 0x03 */
    volatile uint16_t regGateDriveLs;   /* 0x04 */
    volatile uint16_t regOcpCtrl;       /* 0x05 */

    /* Parsed fault status */
    volatile uint32_t faultFlags;
    volatile uint8_t  isFaultActive;

    /* Communication status */
    volatile uint8_t  dmaBusy;
    volatile uint8_t  syncBusy;         /* 阻塞式SPI访问占用标志 */
    volatile uint8_t  spiError;
    volatile uint32_t commCount;
    volatile uint32_t errorCount;

} DRV8350S_Runtime_t;

/* Asynchronous Read Request -------------------------------------------------*/
typedef struct {
    uint8_t registerAddr;               /* Target register to read */
    uint8_t pending;                    /* Request pending flag */
} DRV8350S_ReadReq_t;

/* Handle Structure ----------------------------------------------------------*/
typedef struct {
    SPI_HandleTypeDef*  hspi;           /* SPI1 handle */
    TIM_HandleTypeDef*  htim;           /* TIM1 handle for synchronization */
    GPIO_TypeDef*       nscsPort;       /* Chip select port */
    uint16_t            nscsPin;        /* Chip select pin */

    /* DMA Buffers */
    uint16_t            txBuf[2];       /* TX buffer for SPI transaction */
    uint16_t            rxBuf[2];       /* RX buffer for SPI transaction */

    /* State */
    DRV8350S_Config_t   config;
    DRV8350S_Runtime_t  runtime;
    DRV8350S_ReadReq_t  readReq;

    /* Callbacks */
    void (*faultCallback)(uint32_t faultFlags);
    void (*readCompleteCallback)(uint8_t regAddr, uint16_t data);

} DRV8350S_Handle_t;

/* Function Prototypes -------------------------------------------------------*/

/* Initialization */
int8_t DRV8350S_Init(DRV8350S_Handle_t* handle,
                     SPI_HandleTypeDef* hspi,
                     TIM_HandleTypeDef* htim,
                     GPIO_TypeDef* nscsPort,
                     uint16_t nscsPin);

int8_t DRV8350S_DeInit(DRV8350S_Handle_t* handle);

/* Configuration */
void DRV8350S_SetDefaultConfig(DRV8350S_Config_t* config);
int8_t DRV8350S_Configure(DRV8350S_Handle_t* handle, const DRV8350S_Config_t* config);
int8_t DRV8350S_LockRegisters(DRV8350S_Handle_t* handle);
int8_t DRV8350S_UnlockRegisters(DRV8350S_Handle_t* handle);

/* Asynchronous Operations (called from TIM1 ISR at 20kHz) */
int8_t DRV8350S_TriggerAsyncRead(DRV8350S_Handle_t* handle, uint8_t regAddr);
int8_t DRV8350S_TriggerAsyncReadAll(DRV8350S_Handle_t* handle);
void DRV8350S_TIM1_UpdateCallback(DRV8350S_Handle_t* handle);

/* Synchronous Operations (blocking, for initialization only) */
int8_t DRV8350S_WriteRegister(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t data);
int8_t DRV8350S_ReadRegister(DRV8350S_Handle_t* handle, uint8_t regAddr, uint16_t* data);

/* Control */
int8_t DRV8350S_ClearFaults(DRV8350S_Handle_t* handle);
int8_t DRV8350S_EnableGateDrivers(DRV8350S_Handle_t* handle);
int8_t DRV8350S_DisableGateDrivers(DRV8350S_Handle_t* handle);
int8_t DRV8350S_SetCoast(DRV8350S_Handle_t* handle);
int8_t DRV8350S_SetBrake(DRV8350S_Handle_t* handle);

/* Status & Diagnostics */
uint32_t DRV8350S_GetFaultFlags(DRV8350S_Handle_t* handle);
const char* DRV8350S_FaultToString(uint32_t faultBit);

/* DMA Callbacks (call from HAL_SPI_TxRxCpltCallback) */
void DRV8350S_DMA_CompleteCallback(DRV8350S_Handle_t* handle);
void DRV8350S_DMA_ErrorCallback(DRV8350S_Handle_t* handle);



#ifdef __cplusplus
}
#endif

#endif /* __DRV8350S_H */
