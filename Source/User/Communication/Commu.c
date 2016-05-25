/**
* @file      Commu.c
* @brief     
* @details   
* @author    ranwei
* @version     
* @date      2016/5/11 16:10:9:784
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
*         1.Date         -- 2016/5/11 16:10:9:784
*           Author       -- ranwei
*           Modification -- Created file
*
*/
    
#define  COMMU_GLOBAL

/* includes-------------------------------------------------------------------*/
#include <stdint.h>
#include "serial_debug\serial_debug.h"
#include "Communication\Commu_Uart.h"
#include "Communication\Commu.h"
#include "Mem\Mem.h"
#include "List\List.h"

    
/* Private typedef&macro&definde----------------------------------------------*/
#ifdef SERIAL_DEBUG
    #define __Commu_PROMPT__
    #define __Commu_ASSERT__
#endif

#define SOH               (0xF0)  /** start of 128-byte data packet */
#define EOT                (0xF1)  /** end of transmission */
#define ACK               (0xF2)  /** acknowledge */
#define NAK               (0xF3)  /** negative acknowledge */
#define CA                  (0xF4)  /** aborts transfer */
#define UPDATE        (0xF5)  /** update msg */


#define PACKET_SIZE             (128)
#define PACKET_1K_SIZE          (1024)

#define FILE_NAME_LENGTH        (50)
#define FILE_SIZE_LENGTH        (16)


#pragma pack(push, 1)


typedef struct{
    List *pRxDataList;
    List *pTxDataList;
}ST_Commu_CB_T;


#pragma pack(pop)



/* Private variables ---------------------------------------------------------*/
ST_Commu_CB_T g_stCommuCB;


/* Private functions ---------------------------------------------------------*/
/* External functions --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/


static void _Commu_Prompt(int line, const char *func, const char *format, ...)
#ifdef __Commu_PROMPT__
{
    va_list args;

    va_start(args, format);
    __Prompt__(__FILE__, line, func, format, args);
    va_end(args);    
}
#else
{
}
#endif


static void _Commu_Assert(int line, const char *func, uint8_t assertion, const char *format, ...)
#ifdef __Commu_ASSERT__
{
    va_list args;

    if(!assertion)
    {
        va_start(args, format);
        __Assert__(__FILE__, line, func, format, args);
        va_end(args);    
        while(1);
    }
}
#else
{
}
#endif


static void _Commu_CB_Init(void)
{
    g_stCommuCB.pRxDataList = List_Create();
    _Commu_Assert(__LINE__, __FUNCTION__, g_stCommuCB.pRxDataList != NULL, "can't create Rx list");
    
    g_stCommuCB.pTxDataList = List_Create();
    _Commu_Assert(__LINE__, __FUNCTION__, g_stCommuCB.pRxDataList != NULL, "can't create Tx list");
}

static void _Commu_Accept(uint8_t *pucData, uint16_t uslen)
{
    uint8_t *pDataBuff = (uint8_t *)Mem_Malloc(uslen + 1);
    if(NULL == pDataBuff)
    {
        _Commu_Prompt(__LINE__, __FUNCTION__, "can't malloc a space to save data");
    }
    else
    {
        pDataBuff[0] = (uint8_t)uslen;
        memcpy(&pDataBuff[1], pucData, uslen);
        g_stCommuCB.pRxDataList->HeadInsert(g_stCommuCB.pRxDataList, pDataBuff);
    }
}




static void __Commu_handle(uint8_t *frame, uint8_t len)
{   
   
}


static void _Commu_TxLoopProc(void)
{
    uint8_t ucLen, *pData;
    if(g_stCommuCB.pTxDataList->ucLen > 0)
    {
        if(Commu_uart_IsTxLineValid())
        {
            pData = g_stCommuCB.pTxDataList->TailRemove(g_stCommuCB.pTxDataList);
            if(pData != NULL)
            {
                ucLen = pData[0];
                Commu_Uart_SendData(&pData[1], ucLen);
                Mem_Free(pData);
                pData = NULL;
            }            
        }
    }    
}


static void _Commu_RxLoopProc(void)
{
    uint8_t ucLen, *pData;
    if(g_stCommuCB.pRxDataList->ucLen > 0)
    {
        pData = g_stCommuCB.pRxDataList->TailRemove(g_stCommuCB.pRxDataList);
        if(pData != NULL)
        {
            ucLen = pData[0];
            __Commu_handle(&pData[1], ucLen);
            Mem_Free(pData);
            pData = NULL;            
        }
    }
}

void Commu_Init(void)
{
    Commu_Uart_Init();
    _Commu_CB_Init();
    Commu_Uart_Subscribe(_Commu_Accept);
}



void Commu_LoopProc(void *arg)
{
    _Commu_RxLoopProc();
    _Commu_TxLoopProc();
    
}

int32_t Commu_SendData(uint8_t *pData, uint8_t len)
{
    uint8_t *pDataBuff = (uint8_t *)Mem_Malloc(len + 1);
    if(NULL == pDataBuff)
    {
        _Commu_Prompt(__LINE__, __FUNCTION__, "can't malloc a space to save data");
			return -1;
    }
    else
    {
        pDataBuff[0] = len;
        memcpy(&pDataBuff[1], pData, len);
        g_stCommuCB.pTxDataList->HeadInsert(g_stCommuCB.pTxDataList, pDataBuff);
    }    
		return 0;
}



    
/**************** (C) COPYRIGHT 2010-2018 Efficient *****END OF FILE***********/
