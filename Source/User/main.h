/**
* @file      main.h
* @brief     
* @details   
* @author    ranwei
* @version     
* @date      2016/5/13 9:54:52:812
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
*         1.Date         -- 2016/5/13 9:54:52:812
*           Author       -- ranwei
*           Modification -- Created file
*
*/
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C"{
#endif /** __cplusplus */

#ifdef  MAIN_GLOBAL
#define MAIN_EXT
#else
#define MAIN_EXT extern
#endif /** MAIN_GLOBAL */

/*============================================================================*/
/*                                  @INCLUDES                                 */
/*============================================================================*/
#include <stdint.h>


/*============================================================================*/
/*                              @MACROS & @TYPEDEFS                           */
/*============================================================================*/
typedef void (*Init_Func_Type)(void);

#define SECTION(x)                  __attribute__((section(x)))  
#define UNUSED                      __attribute__((unused))  
#define USED                        __attribute__((used))  
#define ALIGN(n)                    __attribute__((aligned(n)))  
#define UserSys_Init_Register(x)    static const Init_Func_Type __init_func_##x  SECTION("sys_init_func") USED = (x);  


#pragma pack(push, 1)

typedef struct{
    uint8_t ucUpdateFlag;
    uint8_t ucIsSystemFirstRun;
}ST_MainSystem_CB_Type;
#pragma pack(pop)



/*============================================================================*/
/*                               @GLOBAL VIRIABLES                            */
/*============================================================================*/
MAIN_EXT ST_MainSystem_CB_Type g_stMainSystemCB;



/*============================================================================*/
/*                                    @FUNCS                                  */
/*============================================================================*/

#ifdef __cplusplus
}
#endif /** __cplusplus */

#endif /** __MAIN_H__ */
/**************** (C) COPYRIGHT 2010-2018 Efficien ******END OF FILE***********/
