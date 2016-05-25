/**
* @file      Commu_Uart.c
* @brief     
* @details   
* @author    ranwei
* @version     
* @date      2016/5/11 11:11:51:172
* @copyright Efficien
* @par         (c) COPYRIGHT 2010-2018 by Efficien Systems, Inc.    
*                        All rights reserved.
*                                                                    
*       This software is confidential and proprietary to Efficien 
*     Systems, Inc.  No part of this software may be reproduced,    
*     stored, transmitted, disclosed or used in any form or by any means
*     other than as expressly provided by the written license agreement    
*     between Efficien Systems and its licensee.
* @par History      
*         1.Date         -- 2016/5/11 11:11:51:172
*           Author       -- ranwei
*           Modification -- Created file
*
*/
    
#define  COMMU_UART_GLOBAL

/* includes-------------------------------------------------------------------*/
#include <string.h>
#include "stm32f10x.h"
#include "Communication\Commu_Uart.h"


    
/* Private typedef&macro&definde----------------------------------------------*/
#ifdef __COMMU_USE_USART1
/* Definition for USARTx resources */
#define COMMU_UART                           USART1
#define COMMU_UART_CLK                       RCC_APB2Periph_USART1
#define COMMU_UART_CLK_INIT                  RCC_APB2PeriphClockCmd
#define COMMU_UART_IRQn                      USART1_IRQn
#define COMMU_UART_IRQHandler                USART1_IRQHandler
#define COMMU_UART_TX_PIN                    GPIO_Pin_9                
#define COMMU_UART_TX_GPIO_PORT              GPIOA                       
#define COMMU_UART_TX_GPIO_CLK               RCC_APB2Periph_GPIOA
#define COMMU_UART_TX_GPIO_CLK_INIT          RCC_APB2PeriphClockCmd
#define COMMU_UART_TX_SOURCE                 GPIO_PinSource9
#define COMMU_UART_RX_PIN                    GPIO_Pin_10                
#define COMMU_UART_RX_GPIO_PORT              GPIOA                    
#define COMMU_UART_RX_GPIO_CLK               RCC_APB2Periph_GPIOA
#define COMMU_UART_RX_GPIO_CLK_INIT          RCC_APB2PeriphClockCmd
#define COMMU_UART_RX_SOURCE                 GPIO_PinSource10
#define COMMU_UART_BAUDRATE                  256000

/* Definition for DMAx resources */
#define COMMU_UART_DR_ADDRESS                ((uint32_t)USART1 + 0x04) 
#define COMMU_UART_DMA                       DMA1

#define COMMU_UART_TX_DMA_CLK                RCC_AHBPeriph_DMA1
#define COMMU_UART_TX_DMA_CLK_INIT           RCC_AHBPeriphClockCmd
#define COMMU_UART_TX_DMA_CHANNEL            DMA1_Channel4
#define COMMU_UART_TX_DMA_FLAG_GL            DMA1_FLAG_GL4
#define COMMU_UART_TX_DMA_FLAG_TC            DMA1_FLAG_TC4
#define COMMU_UART_TX_DMA_FLAG_HT            DMA1_FLAG_HT4
#define COMMU_UART_TX_DMA_FLAG_TE            DMA1_FLAG_TE4
#define COMMU_UART_TX_DMA_IT_GL              DMA1_IT_GL4
#define COMMU_UART_TX_DMA_IT_TC              DMA1_IT_TC4
#define COMMU_UART_TX_DMA_IT_HT              DMA1_IT_HT4
#define COMMU_UART_TX_DMA_IT_TE              DMA1_IT_TE4

#define COMMU_UART_RX_DMA_CLK                RCC_AHBPeriph_DMA1
#define COMMU_UART_RX_DMA_CLK_INIT           RCC_AHBPeriphClockCmd
#define COMMU_UART_RX_DMA_CHANNEL            DMA1_Channel5
#define COMMU_UART_RX_DMA_FLAG_GL            DMA1_FLAG_GL4
#define COMMU_UART_RX_DMA_FLAG_TC            DMA1_FLAG_TC4
#define COMMU_UART_RX_DMA_FLAG_HT            DMA1_FLAG_HT4
#define COMMU_UART_RX_DMA_FLAG_TE            DMA1_FLAG_TE4
#define COMMU_UART_RX_DMA_IT_GL              DMA1_IT_GL4
#define COMMU_UART_RX_DMA_IT_TC              DMA1_IT_TC4
#define COMMU_UART_RX_DMA_IT_HT              DMA1_IT_HT4
#define COMMU_UART_RX_DMA_IT_TE              DMA1_IT_TE4


#define COMMU_UART_DMA_TX_IRQn               DMA1_Channel4_IRQn

#define COMMU_UART_DMA_TX_IRQHandler         DMA1_Channel4_IRQHandler
#define COMMU_UART_DMA_RX_IRQHandler         USART1_IRQHandler

#endif


#ifdef __COMMU_USE_USART2
/* Definition for USARTx resources */
#define COMMU_UART                           USART2
#define COMMU_UART_CLK                       RCC_APB1Periph_USART2
#define COMMU_UART_CLK_INIT                  RCC_APB1PeriphClockCmd
#define COMMU_UART_IRQn                      USART2_IRQn
#define COMMU_UART_IRQHandler                USART2_IRQHandler
#define COMMU_UART_TX_PIN                    GPIO_Pin_2                
#define COMMU_UART_TX_GPIO_PORT              GPIOA                       
#define COMMU_UART_TX_GPIO_CLK               RCC_APB2Periph_GPIOA
#define COMMU_UART_TX_GPIO_CLK_INIT          RCC_APB2PeriphClockCmd
#define COMMU_UART_TX_SOURCE                 GPIO_PinSource2
#define COMMU_UART_RX_PIN                    GPIO_Pin_3                
#define COMMU_UART_RX_GPIO_PORT              GPIOA                    
#define COMMU_UART_RX_GPIO_CLK               RCC_APB2Periph_GPIOA
#define COMMU_UART_RX_GPIO_CLK_INIT          RCC_APB2PeriphClockCmd
#define COMMU_UART_RX_SOURCE                 GPIO_PinSource3
#define COMMU_UART_BAUDRATE                  115200

/* Definition for DMAx resources */
#define COMMU_UART_DR_ADDRESS                ((uint32_t)USART2 + 0x04) 
#define COMMU_UART_DMA                       DMA1

#define COMMU_UART_TX_DMA_CLK                RCC_AHBPeriph_DMA1
#define COMMU_UART_TX_DMA_CLK_INIT           RCC_AHBPeriphClockCmd
#define COMMU_UART_TX_DMA_CHANNEL            DMA1_Channel7
#define COMMU_UART_TX_DMA_FLAG_GL            DMA1_FLAG_GL7
#define COMMU_UART_TX_DMA_FLAG_TC            DMA1_FLAG_TC7
#define COMMU_UART_TX_DMA_FLAG_HT            DMA1_FLAG_HT7
#define COMMU_UART_TX_DMA_FLAG_TE            DMA1_FLAG_TE7
#define COMMU_UART_TX_DMA_IT_GL              DMA1_IT_GL7
#define COMMU_UART_TX_DMA_IT_TC              DMA1_IT_TC7
#define COMMU_UART_TX_DMA_IT_HT              DMA1_IT_HT7
#define COMMU_UART_TX_DMA_IT_TE              DMA1_IT_TE7

#define COMMU_UART_RX_DMA_CLK                RCC_AHBPeriph_DMA1
#define COMMU_UART_RX_DMA_CLK_INIT           RCC_AHBPeriphClockCmd
#define COMMU_UART_RX_DMA_CHANNEL            DMA1_Channel6
#define COMMU_UART_RX_DMA_FLAG_GL            DMA1_FLAG_GL6
#define COMMU_UART_RX_DMA_FLAG_TC            DMA1_FLAG_TC6
#define COMMU_UART_RX_DMA_FLAG_HT            DMA1_FLAG_HT6
#define COMMU_UART_RX_DMA_FLAG_TE            DMA1_FLAG_TE6
#define COMMU_UART_RX_DMA_IT_GL              DMA1_IT_GL6
#define COMMU_UART_RX_DMA_IT_TC              DMA1_IT_TC6
#define COMMU_UART_RX_DMA_IT_HT              DMA1_IT_HT6
#define COMMU_UART_RX_DMA_IT_TE              DMA1_IT_TE6


#define COMMU_UART_DMA_TX_IRQn               DMA1_Channel7_IRQn

#define COMMU_UART_DMA_TX_IRQHandler         DMA1_Channel7_IRQHandler
#define COMMU_UART_DMA_RX_IRQHandler         USART2_IRQHandler

#endif



/** 允许注册的串口数据accept回调函数的个数 */
#define SZ_COMMU_Uart_ACCEPT_HDL_NUM           5


#define COMMU_UART_RX_BUFF_LEN  SZ_COMMU_DMA_BUFF_MAX
#define COMMU_UART_TX_BUFF_LEN  SZ_COMMU_DMA_BUFF_MAX

#pragma pack(push, 1)


typedef struct{
    uint8_t aucRxBuff[COMMU_UART_RX_BUFF_LEN];       
    uint8_t aucTxBuff[COMMU_UART_TX_BUFF_LEN];       
    uint8_t bIsSendIdleFlag : 1; /** 发送是否空闲标识，1:空闲|0:不空闲 */
    uint8_t reserved1 : 7;
    Accept_Func_Type afAcceptHdlArray[SZ_COMMU_Uart_ACCEPT_HDL_NUM];
}ST_Commu_Uart_CB_T;

#pragma pack(pop)


/* Private variables ---------------------------------------------------------*/
ST_Commu_Uart_CB_T g_stCommuUartCB;




/* Private functions ---------------------------------------------------------*/
/* External functions --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/


static void _Commu_Uart_CB_Init(void)
{
    uint32_t iLoop;
    
    memset(g_stCommuUartCB.aucRxBuff, 0, COMMU_UART_RX_BUFF_LEN);
    memset(g_stCommuUartCB.aucTxBuff, 0, COMMU_UART_TX_BUFF_LEN);
    g_stCommuUartCB.bIsSendIdleFlag = 1; /** 发送是否空闲标识，1:空闲|0:不空闲 */

    for(iLoop = 0; iLoop < SZ_COMMU_Uart_ACCEPT_HDL_NUM; iLoop++)
    {
        g_stCommuUartCB.afAcceptHdlArray[iLoop] = NULL;
    }
}

void Commu_Uart_Init(void)
{       
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    _Commu_Uart_CB_Init();

    COMMU_UART_CLK_INIT(COMMU_UART_CLK, ENABLE);/** 打开串口对应的外设时钟 */

    /** ---------------------串口发DMA配置--------------------- */
    COMMU_UART_TX_DMA_CLK_INIT(COMMU_UART_TX_DMA_CLK, ENABLE); 

    /** DMA发送中断设置 */
    NVIC_InitStructure.NVIC_IRQChannel = COMMU_UART_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(COMMU_UART_TX_DMA_CHANNEL); /** DMA1通道4配置 */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&COMMU_UART->DR); /** 外设地址 */	
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_stCommuUartCB.aucTxBuff; /** 内存地址 */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; /** dma传输方向单向 */
    DMA_InitStructure.DMA_BufferSize = COMMU_UART_TX_BUFF_LEN; /** 设置DMA在传输时缓冲区的长度 */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /** 设置DMA的外设递增模式，一个外设 */	
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /** 设置DMA的内存递增模式 */ 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /** 外设数据字长 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; /** 内存数据字长 */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; /** 设置DMA的传输模式 */	
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; /** 设置DMA的优先级别 */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; /** 设置DMA的2个memory中的变量互相访问 */
    DMA_Init(COMMU_UART_TX_DMA_CHANNEL, &DMA_InitStructure);
    DMA_ITConfig(COMMU_UART_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    /** 使能发送通道 */
    //DMA_Cmd(COMMU_UART_TX_DMA_CHANNEL, ENABLE);


    /** ---------------------串口收DMA配置--------------------- */
    COMMU_UART_RX_DMA_CLK_INIT(COMMU_UART_RX_DMA_CLK, ENABLE); 

    DMA_DeInit(COMMU_UART_RX_DMA_CHANNEL); /** 接收DMA通道配置 */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&COMMU_UART->DR); /** 外设地址 */
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_stCommuUartCB.aucRxBuff; /** 内存地址 */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; /** dma传输方向单向 */
    DMA_InitStructure.DMA_BufferSize = COMMU_UART_RX_BUFF_LEN; /** 设置DMA在传输时缓冲区的长度 */	
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /** 设置DMA的外设递增模式，一个外设 */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /** 设置DMA的内存递增模式 */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /** 外设数据字长 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; /** 内存数据字长 */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; /** 设置DMA的传输模式 */
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; /** 设置DMA的优先级别 */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; /** 设置DMA的2个memory中的变量互相访问 */
    DMA_Init(COMMU_UART_RX_DMA_CHANNEL, &DMA_InitStructure);

    /** 使能接收通道 */
    DMA_Cmd(COMMU_UART_RX_DMA_CHANNEL, ENABLE);


    /** ---------------------串口配置--------------------- */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = COMMU_UART_BAUDRATE; 
    USART_Init(COMMU_UART, &USART_InitStructure);  
    //TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
    //USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

    /** 中断配置 */
    USART_ITConfig(COMMU_UART, USART_IT_TC, DISABLE);
    USART_ITConfig(COMMU_UART, USART_IT_RXNE, DISABLE);
    USART_ITConfig(COMMU_UART, USART_IT_IDLE, ENABLE);  

    /** 配置UART中断 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel = COMMU_UART_IRQn;               //通道设置为串口1中断  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //中断占先等级0  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //中断响应优先级0  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断  
    NVIC_Init(&NVIC_InitStructure);   

    /** 采用DMA方式发送 */
    USART_DMACmd(COMMU_UART, USART_DMAReq_Tx, ENABLE);
    /** 采用DMA方式接收 */
    USART_DMACmd(COMMU_UART, USART_DMAReq_Rx, ENABLE);
    /** 启动串口   */
    USART_Cmd(COMMU_UART, ENABLE); 

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
    COMMU_UART_TX_GPIO_CLK_INIT(COMMU_UART_TX_GPIO_CLK, ENABLE);
    COMMU_UART_RX_GPIO_CLK_INIT(COMMU_UART_RX_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = COMMU_UART_TX_PIN;                       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 
    GPIO_Init(COMMU_UART_TX_GPIO_PORT, &GPIO_InitStructure);                          
    GPIO_InitStructure.GPIO_Pin = COMMU_UART_RX_PIN;                      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           
    GPIO_Init(COMMU_UART_RX_GPIO_PORT, &GPIO_InitStructure);         
}


void COMMU_UART_DMA_TX_IRQHandler(void)  
{  
    if(DMA_GetITStatus(COMMU_UART_TX_DMA_IT_TC) != RESET)   
    {  
        /** 清除标志位 */
        DMA_ClearFlag(COMMU_UART_TX_DMA_FLAG_TC);  
        
        /** 关闭DMA */  
        DMA_Cmd(COMMU_UART_TX_DMA_CHANNEL, DISABLE);  

        /** 允许再次发送 */ 
        g_stCommuUartCB.bIsSendIdleFlag = 1;
    }   
}  


void COMMU_UART_DMA_RX_IRQHandler(void)                                 
{     
    uint32_t temp = 0, iLoop;  
      
    if(USART_GetITStatus(COMMU_UART, USART_IT_IDLE) != RESET)  
    {  
        //USART_ClearFlag(COMMU_UART, USART_IT_IDLE);  
        temp = COMMU_UART->SR;  
        temp = COMMU_UART->DR;  /** 清USART_IT_IDLE标志 */
        DMA_Cmd(COMMU_UART_RX_DMA_CHANNEL, DISABLE);  
  
        temp = COMMU_UART_RX_BUFF_LEN - DMA_GetCurrDataCounter(COMMU_UART_RX_DMA_CHANNEL);  

        //Commu_Uart_SendData(g_stCommuUartCB.aucRxBuff, temp);
        
        for(iLoop = 0; iLoop < SZ_COMMU_Uart_ACCEPT_HDL_NUM; iLoop++)
        {
            if(g_stCommuUartCB.afAcceptHdlArray[iLoop] != NULL)
            {
                g_stCommuUartCB.afAcceptHdlArray[iLoop](g_stCommuUartCB.aucRxBuff, temp);
            }
        }
        
        
        /** 设置传输数据长度 */
        DMA_SetCurrDataCounter(COMMU_UART_RX_DMA_CHANNEL, COMMU_UART_RX_BUFF_LEN);  
        /** 打开DMA */
        DMA_Cmd(COMMU_UART_RX_DMA_CHANNEL, ENABLE);  
    }   
      
    __nop();   
}   

uint32_t Commu_uart_IsTxLineValid(void)
{
    return (g_stCommuUartCB.bIsSendIdleFlag == 1);
}

int32_t Commu_Uart_SendData(uint8_t *pData, uint8_t len)
{
    /** 检查串口是否可以发送  */
    while(!g_stCommuUartCB.bIsSendIdleFlag)
    g_stCommuUartCB.bIsSendIdleFlag = 0;      

    /** 复制数据 */
    memcpy(g_stCommuUartCB.aucTxBuff, pData, len);   

    /** 设置传输数据长度  */
    DMA_SetCurrDataCounter(COMMU_UART_TX_DMA_CHANNEL, len);  
    /** 打开DMA */  
    DMA_Cmd(COMMU_UART_TX_DMA_CHANNEL, ENABLE);      

    return 0;
}

void Commu_Uart_Subscribe(Accept_Func_Type Accept)
{
    uint32_t iLoop;
    
    for(iLoop = 0; iLoop < SZ_COMMU_Uart_ACCEPT_HDL_NUM; iLoop++)
    {
        if(NULL == g_stCommuUartCB.afAcceptHdlArray[iLoop])
        {
            g_stCommuUartCB.afAcceptHdlArray[iLoop] = Accept;
            break;
        }
    }    
}

void Commu_Uart_UnSubscribe(Accept_Func_Type Accept)
{
    uint32_t iLoop;
    
    for(iLoop = 0; iLoop < SZ_COMMU_Uart_ACCEPT_HDL_NUM; iLoop++)
    {
        if(Accept == g_stCommuUartCB.afAcceptHdlArray[iLoop])
        {
            g_stCommuUartCB.afAcceptHdlArray[iLoop] = NULL;
            break;
        }
    }  
}



    
/**************** (C) COPYRIGHT 2010-2018 Efficient *****END OF FILE***********/
