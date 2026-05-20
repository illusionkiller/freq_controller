#ifndef __ENV_H__
#define __ENV_H__

#include "rtthread.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include "posix/stdlib.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define STR1(R) #R
#define STR2(R) STR1(R)
#define DTU_DEVICE_DEFAULT_SN "DTHT-" // 最长12位
#define DTU_FIRMWARE_VERSION 2
// 环境变量文件路径
#define ENV_FILE_PATH "/spiflash/env.txt"
#if defined BSP_USING_KA_DOWN
#define DEFAULT_ENV_PARAM() {                \
    {"SN", DTU_DEVICE_DEFAULT_SN},           \
    {"version", STR2(DTU_FIRMWARE_VERSION)}, \
    {"IP", "192.168.1.10"},                  \
    {"IP_MASK", "255.255.255.0"},            \
    {"GATE_WAY", "192.168.1.1"},             \
    {"INPUT_FRE", "18000000000"},           \
    {"IF_LEVEL", "0"},                   \
}
#elif defined BSP_USING_KA_UP
#define DEFAULT_ENV_PARAM() {                \
    {"SN", DTU_DEVICE_DEFAULT_SN},           \
    {"version", STR2(DTU_FIRMWARE_VERSION)}, \
    {"IP", "192.168.1.10"},                  \
    {"IP_MASK", "255.255.255.0"},            \
    {"GATE_WAY", "192.168.1.1"},             \
    {"OUTPUT_FRE", "28000000000"},          \
}
#elif defined BSP_USING_KU_DOWN
#define DEFAULT_ENV_PARAM() {                \
    {"SN", DTU_DEVICE_DEFAULT_SN},           \
    {"version", STR2(DTU_FIRMWARE_VERSION)}, \
    {"IP", "192.168.1.10"},                  \
    {"IP_MASK", "255.255.255.0"},            \
    {"GATE_WAY", "192.168.1.1"},             \
    {"INPUT_FRE", "14000000000"},        \
    {"IF_LEVEL", "0"},                       \
}
#endif

// 环境变量迭代器结构
typedef struct
{
  char **current;
} env_iterator_t;

// 环境变量迭代器API
void env_iter_init(env_iterator_t *iter);
int env_iter_next(env_iterator_t *iter, char **key, size_t *key_len, char **value, size_t *value_len);
char *env_iter_next_full(env_iterator_t *iter);

// 环境变量管理API
int env_init(void);
int env_save(void);
int env_load(void);
int env_print(void);
int env_clean(void);
int env_restore_default(void);

#ifdef __cplusplus
}
#endif
#endif
