#ifndef __KA_H__
#define __KA_H__

#include "modbus_rtu.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if defined BSP_USING_KA_UP

typedef union
{
    struct
    {
        uint16_t CLK_LOCK : 1;
        uint16_t RESET_CODE : 15;
    };
    uint16_t val;
} clk_node_status_t;
typedef union
{
    struct
    {
        uint16_t OUTPUT_EN : 1; // 与本振节点定义保持一致
        uint16_t RESERVED : 15;
    };
    uint16_t val;
} clk_node_ctrl_t;
// 本振节点属性
typedef struct
{
    uint16_t mb_id;
    rtu_modbus_device_t channel_handle;
    clk_node_status_t status;
    clk_node_ctrl_t ctrl_reg_get;
    clk_node_ctrl_t ctrl_reg_set;
    uint16_t temperature;
    uint16_t version;
    uint16_t spur_param_set;
    uint16_t spur_param_get;
    uint64_t freq_set;
    uint64_t freq_get;
    rt_bool_t link_status;
} clk_node_attr_t;

typedef union
{
    struct
    {
        uint16_t PLL_LOCK : 1; // D0
        uint16_t INPUT_REF_STA : 2;
        uint16_t RESERVED : 5;
        uint16_t RESET_CODE : 8;
    };
    uint16_t val;
} pll_node_status_t;
typedef struct
{
    uint16_t mb_id;
    rtu_modbus_device_t channel_handle;
    pll_node_status_t status;
    uint16_t ctrl_reg_set;
    uint16_t ctrl_reg_get;
    uint16_t temperature;
    uint16_t version;
    uint16_t input_ref_vol;
    uint64_t freq_set;
    uint64_t freq_get;
    rt_bool_t link_status;
} pll_node_attr_t;

// 变频模块节点属性
typedef struct
{
    uint16_t mb_id;
    rtu_modbus_device_t channel_handle;
    uint16_t status;
    uint16_t ctrl_reg_set;
    uint16_t ctrl_reg_get;
    uint16_t temperature;
    uint16_t version;
    uint16_t decay_value_set;
    uint16_t decay_value_get;
    rt_bool_t link_status;
} frecov_node_attr_t;

// 二次变频频率组合定义
typedef struct
{
    uint64_t IF_freq; // Hz
    uint64_t f3_2[2]; // Hz
} Combo;

// 2中频属性定义
typedef struct
{
    uint64_t IF_freq;  // Hz
    uint32_t pfd1_min; // Hz  距离PFD频率的最小距离,优化杂散
    uint32_t pfd2_min; // Hz
    /* data */
} IF_table_t;

#define DEFAULT_IF_SELECT_TABLE()      \
    {900000000ULL, 10000000, 10000000} \


// 定义一个衰减补偿表给后级衰减使用
typedef struct
{
    uint64_t freq;
    uint8_t value;
} decay_compensation_t; // 衰减补偿

#define DEFAULT_DECAY_COMPENSATION_TABLE() \
{                                          \
    {24000000000ULL, 0},                   \
    {25000000000ULL, 0},                   \
    {25500000000ULL, 0},                    \
}
typedef struct
{
    uint64_t output_freq_hz;
    uint64_t decay_compensation_value;
    uint8_t decay_value;
    Combo IF_info[128];
    pll_node_attr_t pll_node;
    frecov_node_attr_t frecov_node[2];
    clk_node_attr_t clk_node[3];
    rt_bool_t output_en;
} channel_attr_t;

channel_attr_t get_channel_attr(void);
rt_err_t config_channel_fre(uint64_t freq_hz);
rt_err_t config_channel_link_decay(uint8_t decay_value);
rt_err_t config_channel_output(rt_bool_t output_en);
rt_err_t get_pll_info(void);
int controler_init(void);
#endif
#ifdef __cplusplus
}
#endif
#endif
