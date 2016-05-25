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



/** ����ע��Ĵ�������accept�ص������ĸ��� */
#define SZ_COMMU_Uart_ACCEPT_HDL_NUM           5


#define COMMU_UART_RX_BUFF_LEN  SZ_COMMU_DMA_BUFF_MAX
#define COMMU_UART_TX_BUFF_LEN  SZ_COMMU_DMA_BUFF_MAX

#pragma pack(push, 1)


typedef struct{
    uint8_t aucRxBuff[COMMU_UART_RX_BUFF_LEN];       
    uint8_t aucTxBuff[COMMU_UART_TX_BUFF_LEN];       
    uint8_t bIsSendIdleFlag : 1; /** �����Ƿ���б�ʶ��1:����|0:������ */
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
    g_stCommuUartCB.bIsSendIdleFlag = 1; /** �����Ƿ���б�ʶ��1:����|0:������ */

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

    COMMU_UART_CLK_INIT(COMMU_UART_CLK, ENABLE);/** �򿪴��ڶ�Ӧ������ʱ�� */

    /** ---------------------���ڷ�DMA����--------------------- */
    COMMU_UART_TX_DMA_CLK_INIT(COMMU_UART_TX_DMA_CLK, ENABLE); 

    /** DMA�����ж����� */
    NVIC_InitStructure.NVIC_IRQChannel = COMMU_UART_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(COMMU_UART_TX_DMA_CHANNEL); /** DMA1ͨ��4���� */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&COMMU_UART->DR); /** �����ַ */	
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_stCommuUartCB.aucTxBuff; /** �ڴ��ַ */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; /** dma���䷽���� */
    DMA_InitStructure.DMA_BufferSize = COMMU_UART_TX_BUFF_LEN; /** ����DMA�ڴ���ʱ�������ĳ��� */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /** ����DMA���������ģʽ��һ������ */	
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /** ����DMA���ڴ����ģʽ */ 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /** ���������ֳ� */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; /** �ڴ������ֳ� */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; /** ����DMA�Ĵ���ģʽ */	
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; /** ����DMA�����ȼ��� */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; /** ����DMA��2��memory�еı���������� */
    DMA_Init(COMMU_UART_TX_DMA_CHANNEL, &DMA_InitStructure);
    DMA_ITConfig(COMMU_UART_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    /** ʹ�ܷ���ͨ�� */
    //DMA_Cmd(COMMU_UART_TX_DMA_CHANNEL, ENABLE);


    /** ---------------------������DMA����--------------------- */
    COMMU_UART_RX_DMA_CLK_INIT(COMMU_UART_RX_DMA_CLK, ENABLE); 

    DMA_DeInit(COMMU_UART_RX_DMA_CHANNEL); /** ����DMAͨ������ */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&COMMU_UART->DR); /** �����ַ */
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_stCommuUartCB.aucRxBuff; /** �ڴ��ַ */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; /** dma���䷽���� */
    DMA_InitStructure.DMA_BufferSize = COMMU_UART_RX_BUFF_LEN; /** ����DMA�ڴ���ʱ�������ĳ��� */	
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /** ����DMA���������ģʽ��һ������ */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /** ����DMA���ڴ����ģʽ */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /** ���������ֳ� */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; /** �ڴ������ֳ� */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; /** ����DMA�Ĵ���ģʽ */
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; /** ����DMA�����ȼ��� */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; /** ����DMA��2��memory�еı���������� */
    DMA_Init(COMMU_UART_RX_DMA_CHANNEL, &DMA_InitStructure);

    /** ʹ�ܽ���ͨ�� */
    DMA_Cmd(COMMU_UART_RX_DMA_CHANNEL, ENABLE);


    /** ---------------------��������--------------------- */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = COMMU_UART_BAUDRATE; 
    USART_Init(COMMU_UART, &USART_InitStructure);  
    //TXE�����ж�,TC��������ж�,RXNE�����ж�,PE��ż�����ж�,�����Ƕ��   
    //USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

    /** �ж����� */
    USART_ITConfig(COMMU_UART, USART_IT_TC, DISABLE);
    USART_ITConfig(COMMU_UART, USART_IT_RXNE, DISABLE);
    USART_ITConfig(COMMU_UART, USART_IT_IDLE, ENABLE);  

    /** ����UART�ж� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel = COMMU_UART_IRQn;               //ͨ������Ϊ����1�ж�  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //�ж�ռ�ȵȼ�0  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //�ж���Ӧ���ȼ�0  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //���ж�  
    NVIC_Init(&NVIC_InitStructure);   

    /** ����DMA��ʽ���� */
    USART_DMACmd(COMMU_UART, USART_DMAReq_Tx, ENABLE);
    /** ����DMA��ʽ���� */
    USART_DMACmd(COMMU_UART, USART_DMAReq_Rx, ENABLE);
    /** ��������   */
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
        /** �����־λ */
        DMA_ClearFlag(COMMU_UART_TX_DMA_FLAG_TC);  
        
        /** �ر�DMA */  
        DMA_Cmd(COMMU_UART_TX_DMA_CHANNEL, DISABLE);  

        /** �����ٴη��� */ 
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
        temp = COMMU_UART->DR;  /** ��USART_IT_IDLE��־ */
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
        
        
        /** ���ô������ݳ��� */
        DMA_SetCurrDataCounter(COMMU_UART_RX_DMA_CHANNEL, COMMU_UART_RX_BUFF_LEN);  
        /** ��DMA */
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
    /** ��鴮���Ƿ���Է���  */
    while(!g_stCommuUartCB.bIsSendIdleFlag)
    g_stCommuUartCB.bIsSendIdleFlag = 0;      

    /** �������� */
    memcpy(g_stCommuUartCB.aucTxBuff, pData, len);   

    /** ���ô������ݳ���  */
    DMA_SetCurrDataCounter(COMMU_UART_TX_DMA_CHANNEL, len);  
    /** ��DMA */  
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
