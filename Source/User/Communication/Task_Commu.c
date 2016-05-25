/**
* @file      Task_Commu.c
* @brief     
* @details   
* @author    ranwei
* @version     
* @date      2016/5/11 19:35:49:887
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
*         1.Date         -- 2016/5/11 19:35:49:887
*           Author       -- ranwei
*           Modification -- Created file
*
*/
    
#define  TASK_COMMU_GLOBAL

/* includes-------------------------------------------------------------------*/
#include "Communication\Commu.h"
    
/* Private typedef&macro&definde----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* External functions --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
    
void Task_Commu_PerioProc(void *arg)
{
  
}

void Task_Commu_LoopProc(void *arg)
{
    Commu_LoopProc(arg);
}

void Task_Commu_Init(void)
{
   Commu_Init();
}


    
/**************** (C) COPYRIGHT 2010-2018 Efficient *****END OF FILE***********/
