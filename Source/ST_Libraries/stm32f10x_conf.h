/*******************************************************************************
        (c) COPYRIGHT 2011 STMicroelectronics Systems, Inc.    
                          All rights reserved.
    
       This software is confidential and proprietary to STMicroelectronics 
     Systems, Inc.  No part of this software may be reproduced,    
     stored, transmitted, disclosed or used in any form or by any means
     other than as expressly provided by the written license agreement    
     between STMicroelectronics Systems and its licensee.
 FileName    : stm32f10x_conf.h
 Author      : ranwei
 Version     : V3.5.0
 Date        : 2016/3/2 11:46:5:613
 Description : Library configuration file.
 Others      : 
 Attention   : The present firmware which is for guidance only aims at providing
               customers with coding information regarding their products in order
               for them to save time. As a result, STMICROELECTRONICS shall not
               be held liable for any direct, indirect or consequential damages 
               with respect to any claims arising from the content of such 
               firmware AND/OR the use made by customers of the coding information
               contained herein in connection with there products.

 History      :
  1.Date         -- 08-April-2011
    Author       -- MCD Application Team
    Modification -- Created file

*******************************************************************************/
#ifndef __STM32F10X_CONF_H__
#define __STM32F10X_CONF_H__

#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */

#ifdef  STM32F10X_CONF_GLOBAL
#define STM32F10X_CONF_EXT
#else
#define STM32F10X_CONF_EXT extern
#endif /* STM32F10X_CONF_GLOBAL */

/*============================================================================*/
/*                                  @INCLUDES                                 */
/*============================================================================*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
#include "STM32F10x_StdPeriph_Driver\stm32f10x_adc.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_bkp.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_can.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_cec.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_crc.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_dac.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_dbgmcu.h"
#include "STM32F10x_StdPeriph_Driver\stm32f10x_dma.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_exti.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_flash.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_fsmc.h"
#include "STM32F10x_StdPeriph_Driver\stm32f10x_gpio.h"
#include "STM32F10x_StdPeriph_Driver\stm32f10x_i2c.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_iwdg.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_pwr.h"
#include "STM32F10x_StdPeriph_Driver\stm32f10x_rcc.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_rtc.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_sdio.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_spi.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_tim.h"
#include "STM32F10x_StdPeriph_Driver\stm32f10x_usart.h"
//#include "STM32F10x_StdPeriph_Driver\stm32f10x_wwdg.h"
#include "STM32F10x_StdPeriph_Driver\misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */



/*============================================================================*/
/*                              @MACROS & @TYPEDEFS                           */
/*============================================================================*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */




/*============================================================================*/
/*                               @GLOBAL VIRIABLES                            */
/*============================================================================*/

/*============================================================================*/
/*                                    @FUNCS                                  */
/*============================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F10X_CONF_H__ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
