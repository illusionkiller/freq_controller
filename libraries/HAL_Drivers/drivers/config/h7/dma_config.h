/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-01-02     zylx         first version
 * 2019-01-08     SummerGift   clean up the code
 * 2020-05-02     whj4674672   support stm32h7 dma1 and dma2
 */

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BSP_UART1_RX_USING_DMA) && !defined(UART1_RX_DMA_INSTANCE)
#define UART1_DMA_RX_IRQHandler          DMA2_Stream0_IRQHandler
#define UART1_RX_DMA_RCC                 RCC_AHB1ENR_DMA2EN
#define UART1_RX_DMA_INSTANCE            DMA2_Stream0
#define UART1_RX_DMA_REQUEST             DMA_REQUEST_USART1_RX
#define UART1_RX_DMA_IRQ                 DMA2_Stream0_IRQn
#endif

#if defined(BSP_UART1_TX_USING_DMA) && !defined(UART1_TX_DMA_INSTANCE)
#define UART1_DMA_TX_IRQHandler          DMA2_Stream1_IRQHandler
#define UART1_TX_DMA_RCC                 RCC_AHB1ENR_DMA2EN
#define UART1_TX_DMA_INSTANCE            DMA2_Stream1
#define UART1_TX_DMA_REQUEST             DMA_REQUEST_USART1_TX
#define UART1_TX_DMA_IRQ                 DMA2_Stream1_IRQn
#endif

#if defined(BSP_UART2_RX_USING_DMA) && !defined(UART2_RX_DMA_INSTANCE)
#define UART2_DMA_RX_IRQHandler          DMA1_Stream4_IRQHandler
#define UART2_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART2_RX_DMA_INSTANCE            DMA1_Stream4
#define UART2_RX_DMA_REQUEST             DMA_REQUEST_USART2_RX
#define UART2_RX_DMA_IRQ                 DMA1_Stream4_IRQn
#endif

#if defined(BSP_UART2_TX_USING_DMA) && !defined(UART2_TX_DMA_INSTANCE)
#define UART2_DMA_TX_IRQHandler          DMA1_Stream5_IRQHandler
#define UART2_TX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART2_TX_DMA_INSTANCE            DMA1_Stream5
#define UART2_TX_DMA_REQUEST             DMA_REQUEST_USART2_TX
#define UART2_TX_DMA_IRQ                 DMA1_Stream5_IRQn
#endif

#if defined(BSP_UART3_RX_USING_DMA) && !defined(UART3_RX_DMA_INSTANCE)
#define UART3_DMA_RX_IRQHandler          DMA1_Stream6_IRQHandler
#define UART3_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART3_RX_DMA_INSTANCE            DMA1_Stream6
#define UART3_RX_DMA_REQUEST             DMA_REQUEST_USART3_RX
#define UART3_RX_DMA_IRQ                 DMA1_Stream6_IRQn
#endif

#if defined(BSP_UART3_TX_USING_DMA) && !defined(UART3_TX_DMA_INSTANCE)
#define UART3_DMA_TX_IRQHandler          DMA1_Stream7_IRQHandler
#define UART3_TX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART3_TX_DMA_INSTANCE            DMA1_Stream7
#define UART3_TX_DMA_REQUEST             DMA_REQUEST_USART3_TX
#define UART3_TX_DMA_IRQ                 DMA1_Stream7_IRQn
#endif  

#if defined(BSP_UART7_RX_USING_DMA) && !defined(UART7_RX_DMA_INSTANCE)
#define UART7_DMA_RX_IRQHandler          DMA1_Stream0_IRQHandler
#define UART7_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART7_RX_DMA_INSTANCE            DMA1_Stream0
#define UART7_RX_DMA_REQUEST             DMA_REQUEST_UART7_RX
#define UART7_RX_DMA_IRQ                 DMA1_Stream0_IRQn
#endif

#if defined(BSP_UART7_TX_USING_DMA) && !defined(UART7_TX_DMA_INSTANCE)
#define UART7_DMA_TX_IRQHandler          DMA1_Stream1_IRQHandler
#define UART7_TX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART7_TX_DMA_INSTANCE            DMA1_Stream1
#define UART7_TX_DMA_REQUEST             DMA_REQUEST_UART7_TX
#define UART7_TX_DMA_IRQ                 DMA1_Stream1_IRQn
#endif

#if defined(BSP_UART8_RX_USING_DMA) && !defined(UART8_RX_DMA_INSTANCE)
#define UART8_DMA_RX_IRQHandler          DMA1_Stream2_IRQHandler
#define UART8_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART8_RX_DMA_INSTANCE            DMA1_Stream2
#define UART8_RX_DMA_REQUEST             DMA_REQUEST_UART8_RX
#define UART8_RX_DMA_IRQ                 DMA1_Stream2_IRQn
#endif

#if defined(BSP_UART8_TX_USING_DMA) && !defined(UART8_TX_DMA_INSTANCE)
#define UART8_DMA_TX_IRQHandler          DMA1_Stream3_IRQHandler
#define UART8_TX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART8_TX_DMA_INSTANCE            DMA1_Stream3
#define UART8_TX_DMA_REQUEST             DMA_REQUEST_UART8_TX
#define UART8_TX_DMA_IRQ                 DMA1_Stream3_IRQn
#endif

#if defined(BSP_QSPI_USING_DMA) && !defined(QSPI_DMA_INSTANCE)
#define QSPI_DMA_IRQHandler              MDMA_IRQHandler
#define QSPI_DMA_RCC                     RCC_AHB3ENR_MDMAEN
#define QSPI_DMA_INSTANCE                MDMA_Channel0
#define QSPI_DMA_IRQ                     MDMA_IRQn
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */
