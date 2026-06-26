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
#define DRV_UART_BUF_SIZE           3072    /* 发送缓冲区大小，需覆盖最坏故障文本 */
#define DRV_UPLOAD_INTERVAL_MS      20      /* 正常数据上传间隔 (ms) */
#define DRV_PHASE_CURRENT_UPLOAD_INTERVAL_MS 5 /* 相电流轻量包上传间隔 (ms) */
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
    uint16_t encoderRawWord;    /* 最近一次完整Data Word */
    uint16_t encoderSafetyWord; /* 最近一次完整Safety Word */
    uint8_t  crcError;          /* CRC 错误标志 */
    uint8_t  encoderReceivedCrc; /* Safety Word接收CRC */
    uint8_t  encoderCalculatedCrc; /* 本地计算CRC */
    uint8_t  encoderCrcErrorCount; /* 连续CRC/数据坏帧计数 */
    uint8_t  encoderGpioDiagActive; /* GPIO诊断模式是否占用TLE总线 */
    uint8_t  encoderSafetyStatus; /* Safety Word高8位 */
    uint8_t  encoderResetFault; /* Safety Word bit15=0，表示复位/看门狗异常 */
    uint8_t  encoderDetected;   /* TLE5012 在线检测状态 */

    /* DRV8350S 驱动器数据 */
    uint16_t faultStatus1;      /* FAULT_STATUS_1 寄存器 */
    uint16_t vgsStatus2;        /* VGS_STATUS_2 寄存器 */
    uint16_t driverCtrl;        /* DRIVER_CTRL 寄存器 */
    uint16_t ocpCtrl;           /* OCP_CTRL 寄存器 */
    uint32_t faultFlags;        /* 解析后的故障标志 */
    uint8_t  isFaultActive;     /* 是否有故障 */
    uint8_t  drvCommFaultActive;/* DRV SPI通信是否无效 */
    uint8_t  drvCommValidated;  /* DRV配置寄存器回读是否已验证 */
    uint16_t drvLastRxFrame;    /* 最近一次SPI原始回读帧 */

    /* FOC控制数据 - 新增 */
    float    Id;                /* D轴电流 */
    float    Iq;                /* Q轴电流 */
    float    adcId;             /* 最新ADC帧重算D轴电流 */
    float    adcIq;             /* 最新ADC帧重算Q轴电流 */
    float    Vd;                /* D轴电压 */
    float    Vq;                /* Q轴电压 */
    float    speed;             /* 转速 (rad/s) */
    float    thetaMech;         /* 当前机械角(rad) */
    float    thetaElec;         /* 当前电角(rad) */
    float    Id_ref;            /* D轴电流参考 */
    float    Iq_ref;            /* Q轴电流参考 */
    float    speed_ref;         /* 速度参考 */
    float    pos_ref;           /* 位置参考 */
    float    speedLoopRef;      /* 速度环最近一次内部速度给定 */
    float    speedLoopMech;     /* 速度环最近一次机械速度反馈 */
    float    speedLoopError;    /* 速度环最近一次误差 */
    float    speedLoopIqMech;   /* 速度环最近一次机械方向Iq输出 */
    float    speedLoopFriction; /* 速度环最近一次静摩擦补偿 */
    float    speedLoopIqCmd;    /* 速度环最近一次映射后的Iq命令 */
    float    speedLoopPIq;      /* 速度环比例贡献 A */
    float    speedLoopIIq;      /* 速度环积分贡献 A */
    uint8_t  speedLoopIState;   /* 积分状态: 0=inactive 1=active 2=frozen 3=cleared 4=sat_hold */
    float    positionLoopError; /* 位置环最近一次位置误差 */
    float    positionLoopPdOut; /* 位置环PD输出速度给定 */
    uint8_t  positionLoopPdSat; /* 位置环PD输出是否触及速度上限 */
    uint8_t  positionLoopRampSat; /* 位置模式速度斜坡是否限制给定 */
    uint8_t  positionLoopIqPosSat; /* 位置模式正向Iq是否触顶 */
    uint8_t  positionLoopIqNegSat; /* 位置模式负向Iq是否触底 */
    uint8_t  trajActive;          /* V4 巡航模式激活标志 */
    float    trajCmd;             /* V4 最终速度指令 rad/s */
    uint32_t positionPrefCmdCount; /* 最近PREF命令计数 */
    float    positionPrefRaw;    /* 最近PREF用户原始目标 */
    float    positionPrefMapped; /* 最近PREF映射后目标 */
    float    positionPrefBefore; /* 最近PREF执行前pos_ref */
    float    positionPrefAfter;  /* 最近PREF执行后pos_ref */
    uint8_t  positionPrefUserSet; /* 最近PREF执行后user_set */
    float    undervoltageLimit; /* 当前欠压阈值(V) */
    float    overvoltageLimit;  /* 当前过压阈值(V) */
    uint8_t  focState;          /* FOC状态 */
    uint8_t  controlMode;       /* 控制模式 */
    uint8_t  pwmEnabled;        /* 应用层PWM使能状态 */
    uint8_t  tim1MoeEnabled;    /* TIM1主输出MOE状态 */
    uint32_t appWarningFlags;   /* 应用层告警位 */
    uint8_t  appFaultCode;      /* 应用层故障码 */
    uint8_t  motorIdentified;   /* 电机参数是否已识别 */
    uint8_t  stallModeArmed;    /* 未识别堵转模式是否已授权 */
    uint8_t  stallOpenLoopActive; /* 堵转模式是否正在执行开环试转 */
    uint8_t  identifyState;     /* 电机识别状态机状态 */
    uint8_t  identifyError;     /* 电机识别失败原因 */
    float    identifyRsCurrentTarget; /* Rs识别当前闭环测试电流目标 */
    float    identifyRsLastVavg;      /* Rs识别最近平均d轴电压 */
    float    identifyRsLastIavg;      /* Rs识别最近平均d轴电流 */
    float    identifyRsLastImagAvg;   /* Rs识别最近平均dq电流幅值 */
    float    identifyRsLastVecRs;     /* Rs识别最近矢量投影电阻 */
    uint32_t identifyRsLastSamples;   /* Rs识别最近统计样本数 */
    float    identifyRsPositive;      /* Rs识别正半周期估计值 */
    float    identifyRsNegative;      /* Rs识别负半周期估计值 */
    float    identifyLsVrms;          /* Ls识别注入电压RMS */
    float    identifyLsIrms;          /* Ls识别电流RMS */
    float    identifyLsZ;             /* Ls识别阻抗幅值 */
    float    identifyLsXl;            /* Ls识别感抗 */
    float    identifyLsL;             /* Ls识别计算电感 */
    uint8_t  identifyLsUsedFallback;  /* Ls是否回退到默认值 */
    float    identifyPnCurrentTarget; /* Pn识别当前开环拖动电流目标 */
    float    identifyPnDeltaMech;     /* Pn识别最近机械角累计变化 rad */
    float    identifyPnDeltaElec;     /* Pn识别最近电角指令变化 rad */
    float    identifyPnCalc;          /* Pn识别最近计算值 */
    int8_t   identifyPnObservedDir;   /* 正电角拖动时实测机械方向 */
    int8_t   identifyVerifyLockedDir; /* 单向运动认证锁定实测机械方向 */
    uint8_t  identifyVerifyPhase;     /* 运动认证阶段 */
    float    identifyVerifyAccum;     /* 锁定方向累计机械角 rad */
    uint32_t paramInvalidFlags;        /* Param_IsValid拒绝原因位图 */
    uint32_t paramValidFlag;           /* motor_param.valid_flag原值 */
    uint32_t latchedFaultFlags;        /* DRV首次故障锁存位图 */
    uint16_t latchedFaultStatus1;      /* DRV首次故障锁存FAULT_STATUS_1 */
    uint16_t latchedVgsStatus2;        /* DRV首次故障锁存VGS_STATUS_2 */
    float    paramRs;                  /* 当前识别/加载的Rs */
    float    paramLd;                  /* 当前识别/加载的Ld */
    float    paramLq;                  /* 当前识别/加载的Lq */
    float    paramKe;                  /* 当前识别/加载的Ke */
    uint8_t  paramPn;                  /* 当前极对数 */
    int8_t   paramEncoderDir;          /* 当前编码器方向 */
    float    paramThetaOffset;         /* 当前电角零位偏置(rad) */
    float    paramThetaMechZero;       /* 当前机械零位(rad) */
    float    paramJ;                   /* 当前转动惯量 */
    uint8_t  svpwmSector;              /* 当前SVPWM扇区 */
    float    svpwmTa;                  /* 当前SVPWM占空比A(0~1) */
    float    svpwmTb;                  /* 当前SVPWM占空比B(0~1) */
    float    svpwmTc;                  /* 当前SVPWM占空比C(0~1) */

    /* ADC采样诊断 */
    char     adcTriggerSource[24];        /* ADC触发源描述 */
    float    adcCurrentSampleTimeCycles;  /* 电流通道采样时间 */
    float    adcVbusSampleTimeCycles;     /* 母线通道采样时间 */
    uint32_t adcFrameSequence;            /* ADC帧序号 */
    uint32_t adcFrameAgeCycles;           /* ADC帧年龄(控制周期) */
    uint32_t adcSampleMissCount;          /* 缺帧计数 */
    uint32_t adcInvalidWindowCount;       /* 晚帧/窗口无效计数 */
    uint32_t adcValidLowSideCount;        /* 运行态低边三相有效帧计数 */
    uint32_t adcInvalidLowSideCount;      /* 运行态低边三相无效帧计数 */
    uint32_t adcForcedLowSideCount;       /* 0/0/0饥饿逃逸强制更新计数 */
    uint16_t adcRawCurrentA;              /* A相原始ADC */
    uint16_t adcRawCurrentB;              /* B相原始ADC */
    uint16_t adcRawCurrentC;              /* C相原始ADC */
    uint16_t adcRawVbus;                  /* 母线电压原始ADC */
    uint16_t adcPwmPeriod;                /* ADC DMA完成时TIM1 ARR */
    uint16_t adcPwmCompareA;              /* ADC DMA完成时TIM1 CCR1 */
    uint16_t adcPwmCompareB;              /* ADC DMA完成时TIM1 CCR2 */
    uint16_t adcPwmCompareC;              /* ADC DMA完成时TIM1 CCR3 */
    uint16_t adcTriggerCompare;           /* ADC DMA完成时TIM1 CCR4 */
    uint16_t adcTimerCount;               /* ADC DMA完成时TIM1 CNT */
    uint8_t  adcTimerCountingDown;        /* ADC DMA完成时TIM1方向 */
    uint8_t  adcLowSideValidA;            /* A相是否落在低边有效采样窗 */
    uint8_t  adcLowSideValidB;            /* B相是否落在低边有效采样窗 */
    uint8_t  adcLowSideValidC;            /* C相是否落在低边有效采样窗 */
    uint8_t  adcCalibStatus;              /* ADC零点校准状态 */
    int16_t  adcOffsetA;                  /* A相零点偏置 */
    int16_t  adcOffsetB;                  /* B相零点偏置 */
    int16_t  adcOffsetC;                  /* C相零点偏置 */
    float    adcCurrentA;                 /* A相电流(A) */
    float    adcCurrentB;                 /* B相电流(A) */
    float    adcCurrentC;                 /* C相电流(A) */
    float    loopCurrentA;                /* 控制环实际采用A相电流(A) */
    float    loopCurrentB;                /* 控制环实际采用B相电流(A) */
    float    loopCurrentC;                /* 控制环实际采用C相电流(A) */
    int16_t  adcDeltaA;                   /* A相raw-offset差值(LSB) */
    int16_t  adcDeltaB;                   /* B相raw-offset差值(LSB) */
    int16_t  adcDeltaC;                   /* C相raw-offset差值(LSB) */
    float    adcVbus;                     /* 母线电压(V) */

    /* FFDiag 前馈诊断 (v4_safe_baseline) */
    float    ffBemfVd;           /* P1 BEMF Vd补偿量 V */
    float    ffBemfVq;           /* P1 BEMF Vq补偿量 V */
    float    ffInertiaIq;        /* P2 惯量 Iq贡献 A */
    float    ffFrictionIq;       /* P3 摩擦 Iq贡献 A */
    float    ffCoggingIq;        /* P0 齿槽 Iq贡献 A */
    float    ffObserverIq;       /* P4 观测器 Iq贡献 A */
    float    ffTotalIq;          /* 前馈总 Iq贡献 A */
    uint8_t  ffBemfEnabled;      /* P1 使能状态 */
    uint8_t  ffInertiaBlocked;   /* P2 被门禁阻止 */
    uint8_t  ffFrictionEnabled;  /* P3 使能状态 */
    uint8_t  ffCoggingEnabled;   /* P0 使能状态 */
    uint8_t  ffObserverEnabled;  /* P4 使能状态 */
    uint8_t  ffEncDirBlocked;    /* enc_dir阻止所有前馈 */

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
 * @brief 上传J/B识别诊断短报文（非阻塞）
 * @return true 开始发送, false 发送忙
 */
bool DrvUart_UploadJDiag(void);

/**
 * @brief 查询当前P0 cogging配置
 */
void DrvUart_QueryCogCfg(void);

/**
 * @brief 浮点数格式化为固定小数位字符串（用于串口输出）
 * @param dst 目标缓冲区
 * @param dstSize 缓冲区大小
 * @param value 浮点值
 * @param decimals 小数位数
 */
void DrvUart_FormatFixed(char* dst, uint16_t dstSize, float value, uint8_t decimals);

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
