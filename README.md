# 控制器项目说明

## 项目概述

本项目是基于STM32H743开发板的控制器系统，集成了modbus协议栈与各子模块（本振、锁相源、倍频器等）进行通信，屏幕和上位机通信协议也采用了modbus协议。

## 项目功能

- 基于STM32H743微控制器的控制系统
- 集成Modbus通信协议栈
- 与本振、锁相源、倍频器等子模块通信
- 支持屏幕和上位机通信
- 使用RT-Thread操作系统

## 目录结构

```
project/
├── applications/        # 应用程序目录
│   ├── Ka_down.c/h      # Ka频段下变频相关代码
│   ├── X_down.c/h       # X频段下变频相关代码
│   ├── dtu_*.c          # 数据传输单元相关代码
│   ├── env.c/h          # 环境监测相关代码
│   ├── env_cmd.c        # 环境监测命令处理
│   ├── main.c           # 主应用程序
│   ├── serial_test.c    # 串口测试代码
│   └── SConscript       # 构建脚本
├── board/               # 板级支持包
│   ├── CubeMX_Config/   # CubeMX配置
│   │   ├── Core/        # 核心配置
│   │   └── Drivers/     # 驱动文件
│   └── .ignore_format.yml
├── .config              # 配置文件
├── .config.old          # 旧配置文件
├── .gitignore           # Git忽略文件
├── Kconfig              # 配置选项
├── project.ewp          # IAR工程文件
├── project.eww          # IAR工作区文件
├── project.uvprojx      # Keil工程文件
├── project.uvoptx       # Keil工程选项文件
├── README.md            # 项目说明文件
├── SConstruct           # 主构建脚本
├── SConscript           # 构建脚本
├── rtconfig.h           # RT-Thread配置头文件
├── rtconfig.py          # RT-Thread配置脚本
├── rtthread.bin         # 编译生成的二进制文件
├── rtthread.map         # 链接映射文件
├── rt-thread.elf        # 编译生成的ELF文件
├── rtthread-KA.bin      # Ka频段配置的二进制文件
├── rtthread-KA-70M.bin  # Ka频段70M配置的二进制文件
├── template.ewp         # 模板工程文件
├── template.eww         # 模板工作区文件
├── template.uvprojx     # 模板Keil工程文件
├── template.uvoptx      # 模板Keil工程选项文件
└── vscode.code-workspace # VSCode工作区文件
```

## 系统架构

1. **硬件平台**：STM32H743微控制器
2. **操作系统**：RT-Thread
3. **通信协议**：Modbus
4. **功能模块**：
   - 本振控制
   - 锁相源控制
   - 倍频器控制
   - 环境监测
   - 屏幕显示
   - 上位机通信

## 开发环境

- 支持的IDE：
  - Keil MDK
  - IAR Embedded Workbench
  - VSCode (通过RT-Thread插件)
- 构建系统：SCons
- 配置工具：menuconfig

## 构建与运行

1. 使用RT-Thread的menuconfig配置系统
2. 通过SCons构建项目
3. 烧录生成的二进制文件到STM32H743开发板

## 通信协议

系统采用Modbus协议与各子模块及上位机通信，确保数据传输的可靠性和一致性。

## 注意事项

- 详情请参考方案设计文档以及相关共享参考资料
- 项目使用RT-Thread操作系统，需要熟悉其基本操作和API
- 各子模块的通信参数需要根据实际硬件进行配置