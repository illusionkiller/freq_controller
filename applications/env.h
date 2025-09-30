#ifndef __ENV_H__
#define __ENV_H__

#include "flashdb.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define DTU_FIRMWARE_VERSION 1
#define DTU_DEVICE_DEFAULT_SN "DTHT-" // 最长12位
#define DTU_DEVICE_SN_MAX_LEN 12
#define DEVICE_FRE_CONTROL_RANGE_KA
typedef struct
{
    uint32_t IP;       // 0x1000  /rw
    uint32_t IP_MASK;  // 0x1002
    uint32_t GATE_WAY; // 0x1004
    uint8_t SN[DTU_DEVICE_SN_MAX_LEN];    // 设备SN,本地IP  //0x1006
    uint16_t LW1011;        // 中频选择
    uint32_t CH1_INPUT_FRE; // 通道1输入频率 Khz
    uint32_t CH2_INPUT_FRE; // 通道2输入频率 Khz
    uint16_t LW1016;        // 链路增益
} sys_reg_t;

rt_err_t save_sys_regs(void);

#ifdef __cplusplus
}
#endif
#endif
