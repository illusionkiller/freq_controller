# 控制器项目说明

## 项目简介

本项目是一个基于 `STM32H743` 和 `RT-Thread` 的控制器固件工程，面向多种下变频/射频控制板型。  
应用层按板型拆分为 `KA / X / Q / Ku` 四类实现，底层统一复用 RT-Thread BSP、STM32 HAL 驱动、网络、文件系统和串口/以太网/QSPI 等通用外设。

从当前配置看，工程默认启用了：

- `BSP_USING_Q_DOWN`
- 以太网 `LAN8720A`
- QSPI Flash + LittleFS
- 4 路串口对外通信
- 串口 DMA
- I-Cache / D-Cache
- 看门狗

## 软件架构

项目大致分为 4 层：

1. BSP 与板级启动
   - 位于 [`board/`](./board)
   - 负责时钟、启动文件、链接脚本、CubeMX 生成配置

2. HAL 驱动适配层
   - 位于 [`libraries/HAL_Drivers/drivers/`](./libraries/HAL_Drivers/drivers)
   - 把 STM32 HAL 外设封装成 RT-Thread 设备驱动

3. RT-Thread 与第三方组件
   - 位于 [`rt-thread/`](./rt-thread) 和 [`packages/`](./packages)
   - 包含内核、lwIP、FAL、LittleFS、HAL Driver、CMSIS、netutils 等

4. 应用层
   - 位于 [`applications/`](./applications)
   - 包含不同板型的 DTU 协议处理、通道控制、环境变量管理和主入口

## 启动流程

系统启动后的主流程可以概括为：

1. 启动文件和 BSP 完成时钟、内存、外设基础初始化
2. RT-Thread 初始化内核、设备驱动和组件
3. `applications/main.c` 中执行：
   - 记录固件版本
   - 检查复位原因
   - 初始化看门狗
   - 调用 `dtu_init_thread(NULL)` 启动当前板型对应的应用线程
4. 主线程进入 LED 心跳循环，并周期性喂狗

## 目录结构

```text
project/
├─ applications/                 应用层代码
│  ├─ main.c                     主入口
│  ├─ env.c / env.h              环境变量管理
│  ├─ env_cmd.c                  FinSH/MSH 环境变量命令
│  ├─ dtu_Ka.c                   KA 型应用逻辑
│  ├─ dtu_X.c                    X 型应用逻辑
│  ├─ dtu_Q.c                    Q 型应用逻辑
│  ├─ dtu_ku.c                   Ku 型应用逻辑
│  ├─ Ka_down.c / .h             KA 板型控制逻辑
│  ├─ X_down.c / .h              X 板型控制逻辑
│  ├─ Q_down.c / .h              Q 板型控制逻辑
│  ├─ Ku_down.c / .h             Ku 板型控制逻辑
│  └─ SConscript                 应用层构建脚本
├─ board/                        板级支持包
│  ├─ board.c / board.h          时钟、板级定义
│  ├─ Kconfig                    BSP 配置入口
│  ├─ linker_scripts/            GCC/MDK/IAR 链接脚本
│  └─ CubeMX_Config/             CubeMX 生成代码与工程文件
├─ libraries/                    HAL 适配与扩展驱动
│  └─ HAL_Drivers/drivers/       RT-Thread 风格外设驱动
├─ packages/                     RT-Thread 包管理下载的第三方组件
├─ rt-thread/                    RT-Thread 源码
├─ SConstruct                    SCons 顶层构建脚本
├─ Kconfig                       menuconfig 顶层入口
├─ rtconfig.h                    当前编译配置导出
└─ README.md                     项目说明
```

## 板型与应用切换

应用层通过 `applications/SConscript` 和 `board/Kconfig` 按板型选择不同源码。

当前支持的板型开关位于 `menuconfig` 中：

- `BSP_USING_KA_DOWN`
- `BSP_USING_X_DOWN`
- `BSP_USING_Q_DOWN`
- `BSP_USING_Ku_DOWN`

不同板型会切换：

- `dtu_*.c`
- `*_down.c`
- 默认环境变量中的频点参数

当前 `rtconfig.h` 显示默认编译的是 `Q` 型版本。

## 应用层功能概览

### 1. DTU/寄存器访问

`dtu_*.c` 是核心业务文件，负责：

- 建立 Modbus TCP / Modbus RTU 设备
- 映射系统寄存器和通道寄存器
- 处理网络参数、序列号、版本号、系统复位等控制项
- 调用对应板型的控制接口完成频点、输出、参考源等配置

### 2. 通道控制

`Ka_down.c`、`X_down.c`、`Q_down.c`、`Ku_down.c` 封装不同板型的通道属性和硬件控制逻辑。

### 3. 环境变量管理

`env.c` 提供一套轻量级环境变量系统，特点是：

- 启动时加载默认参数
- 若存在 `/spiflash/env.txt`，则用文件内容覆盖默认值
- 支持保存、清除、恢复默认值

当前默认环境变量包含：

- `SN`
- `version`
- `IP`
- `IP_MASK`
- `GATE_WAY`
- 每个板型对应的 `CH1/CH2_INPUT_FRE`
- 每个板型对应的 `CH1/CH2_IF_LEVEL`

### 4. MSH 命令

`env_cmd.c` 暴露了以下 shell 命令：

- `getenv <key>`
- `setenv <key> <value>`
- `unsetenv <key>`
- `saveenv`
- `loadenv`
- `printenv`

环境变量文件默认保存在：

```text
/spiflash/env.txt
```

## 当前默认硬件配置

从当前 [`rtconfig.h`](./rtconfig.h) 可以看到工程默认启用了：

- 控制台：`UART1`
- 对外串口：
  - `COM1 -> UART7`
  - `COM2 -> UART8`
  - `COM3 -> UART3`
  - `COM4 -> UART2`
- 以太网：`LAN8720A`
- QSPI Flash：已启用
- 文件系统：`LittleFS`
- 网络：`lwIP + netdev + SAL`
- 看门狗：已启用

## 构建方式

### 依赖

工程使用 `SCons + RT-Thread 工具链配置` 构建。  
首次拉取后需要确保 `packages/` 中的依赖完整。

顶层 `SConstruct` 在构建前会检查以下包是否存在：

- `packages/CMSIS-Core-latest`
- `packages/stm32h7_cmsis_driver-latest`
- `packages/stm32h7_hal_driver-latest`

如果缺失，需要先执行：

```bash
pkgs --update
```

若本地包索引异常，可先执行：

```bash
pkgs --upgrade
pkgs --update
```

### menuconfig

可通过 RT-Thread 的配置系统调整板型和外设：

```bash
menuconfig
```

常见配置项位于：

- `Hardware Drivers Config`
- `Board type select(KA/X/Q)`

### SCons 构建

```bash
scons -j8
```

生成产物通常包括：

- `rt-thread.elf`
- `rtthread.bin`
- `rtthread.map`

同时工程目录中还保留了若干针对不同板型的二进制文件。

## IDE 与工程文件

项目保留了多套工程入口：

- Keil MDK：[`project.uvprojx`](./project.uvprojx)
- IAR EWARM：[`project.ewp`](./project.ewp)
- SCons/命令行：[`SConstruct`](./SConstruct)

如果使用 GCC/SCons，建议以 `menuconfig + scons` 为主流程。  
如果使用 MDK/IAR，请同步确认链接脚本和配置是否与当前代码一致。

## 网络与存储

### 网络

当前工程启用了：

- `RT_USING_LWIP`
- `RT_USING_NETDEV`
- `RT_USING_SAL`
- `netutils/iperf`

默认静态地址配置在 `rtconfig.h` 中为：

- IP: `192.168.1.10`
- Mask: `255.255.255.0`
- Gateway: `192.168.1.1`

应用运行后也可通过环境变量和寄存器映射进行调整。

### 存储

工程启用了：

- 板载 QSPI Flash
- FAL
- LittleFS

环境变量依赖 QSPI Flash 文件系统持久化保存。

## 关键文件速览

- [`applications/main.c`](./applications/main.c)
  - 主线程入口、心跳灯、喂狗
- [`applications/dtu_Q.c`](./applications/dtu_Q.c)
  - Q 型 DTU 业务逻辑示例
- [`applications/env.c`](./applications/env.c)
  - 环境变量加载/保存/恢复
- [`applications/env_cmd.c`](./applications/env_cmd.c)
  - Shell 命令接口
- [`applications/SConscript`](./applications/SConscript)
  - 板型源码选择
- [`board/Kconfig`](./board/Kconfig)
  - BSP 外设和板型选项
- [`board/board.c`](./board/board.c)
  - 系统时钟配置
- [`libraries/HAL_Drivers/drivers/`](./libraries/HAL_Drivers/drivers)
  - 串口、以太网、QSPI、MPU、Flash 等驱动

## OTA 升级

本项目支持通过 TFTP 进行固件 OTA 升级。系统启动后会自动启动 TFTP 服务器（端口 69），可通过网络传输升级文件。

### Flash 分区布局

| 分区名 | 存储设备 | 起始偏移 | 大小 | 说明 |
|--------|----------|----------|------|------|
| bootloader | onchip_flash_128k | 0x0 | 512KB | 引导程序 |
| app | onchip_flash_128k | 0x80000 | 1536KB | 应用固件 |
| spiflash0 | 外部 QSPI Flash | 0x0 | 16MB | 文件系统/下载区 |

### 升级文件说明

升级需要传输两个文件：

| 文件名 | 说明 |
|--------|------|
| `rtthread.bin` | 固件二进制文件 |
| `ota_md5.txt` | 固件 MD5 校验文件 |

**注意：文件名称不能随意更改，bootloader 依赖固定文件名进行识别和校验。**

`ota_md5.txt` 内容为固件文件的 MD5 哈希值（32 位十六进制字符串），用于升级前的完整性校验。

### 升级操作步骤

1. 确保 PC 与设备处于同一网段（设备默认 IP：`192.168.1.10`）
2. 在 PC 上打开 TFTP 客户端工具（如 Tftpd64）
3. 配置 TFTP 客户端：
   - Server IP：设备 IP 地址（默认 `192.168.1.10`）
   - 端口：`69`
4. 依次上传以下文件到设备根目录 `/`：
   - `rtthread.bin` — 编译生成的固件文件
   - `ota_md5.txt` — 对应固件的 MD5 校验文件
5. 文件传输完成后，重启设备，bootloader 将自动完成校验和升级

### 生成 MD5 校验文件

Linux/macOS：

```bash
md5sum rtthread.bin | awk '{print $1}' > ota_md5.txt
```

Windows (PowerShell)：

```powershell
(Get-FileHash rtthread.bin -Algorithm MD5).Hash.ToLower() | Out-File -Encoding ascii ota_md5.txt
```

### 注意事项

- 升级过程中请勿断电，否则可能导致设备无法启动
- 确保 `ota_md5.txt` 中的 MD5 值与 `rtthread.bin` 一致，校验失败将拒绝升级
- TFTP 传输不加密，建议在安全的局域网环境中操作
- 升级完成后可通过 `telnet` 连接设备，查看固件版本确认升级成功

## 开发建议

- 修改板型前，先确认 `menuconfig` 中只启用一个 `BSP_USING_*_DOWN`
- 修改环境变量后，记得执行 `saveenv`
- 若涉及 QSPI、ETH、串口 DMA 等驱动，优先确认 cache 与 DMA 一致性
- 若需要定位构建差异，优先查看：
  - `.config`
  - `rtconfig.h`
  - `board/Kconfig`
  - `applications/SConscript`

## 当前 README 依据

本文档基于当前仓库代码状态整理，重点参考了：

- `applications/main.c`
- `applications/env.c`
- `applications/env_cmd.c`
- `applications/dtu_Q.c`
- `applications/SConscript`
- `board/Kconfig`
- `board/board.c`
- `rtconfig.h`
- `SConstruct`
