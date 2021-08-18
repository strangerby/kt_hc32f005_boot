/*******************************************************************************
* Copyright (C) 2018, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file system_hc32l136.h
 **
 ** A detailed description is available at
 ** @link SampleGroup Some description @endlink
 **
 **   - 2019-08-09  1.1  Chenw First version.
 **
 ******************************************************************************/

#ifndef __SYSTEM_HC32L13X_H__
#define __SYSTEM_HC32L13X_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "hc32_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Global pre-processor symbols/macros ('define')                             */
/******************************************************************************/
#define HWWD_DISABLE        (1)



/**
 ******************************************************************************
 ** \brief Clock Setup macro definition
 **
 ** - 0: CLOCK_SETTING_NONE  - User provides own clock setting in application
 ** - 1: CLOCK_SETTING_CMSIS - 
 ******************************************************************************/
#define CLOCK_SETTING_NONE  0u
#define CLOCK_SETTING_CMSIS 1u

/**
 ******************************************************************************
 ** \brief IRQ name definition for all type MCUs
 ******************************************************************************/
    
#define PORTA_IRQHandler(void)              IRQ000_Handler(void)
#define PORTB_IRQHandler(void)              IRQ001_Handler(void)
#define PORTC_IRQHandler(void)              IRQ002_Handler(void)
#define PORTD_IRQHandler(void)              IRQ003_Handler(void)
#define DMAC_IRQHandler(void)               IRQ004_Handler(void)
#define TIM3_IRQHandler(void)               IRQ005_Handler(void)
#define UART0_IRQHandler(void)              IRQ006_Handler(void)
#define UART1_IRQHandler(void)              IRQ007_Handler(void)
#define LPUART0_IRQHandler(void)            IRQ008_Handler(void)
#define LPUART1_IRQHandler(void)            IRQ009_Handler(void)
#define SPI0_IRQHandler(void)               IRQ010_Handler(void)
#define SPI1_IRQHandler(void)               IRQ011_Handler(void)
#define I2C0_IRQHandler(void)               IRQ012_Handler(void)
#define I2C1_IRQHandler(void)               IRQ013_Handler(void)
#define TIM0_IRQHandler(void)               IRQ014_Handler(void)
#define TIM1_IRQHandler(void)               IRQ015_Handler(void)
#define TIM2_IRQHandler(void)               IRQ016_Handler(void)
#define LPTIM_IRQHandler(void)              IRQ017_Handler(void)
#define TIM4_IRQHandler(void)               IRQ018_Handler(void)
#define TIM5_IRQHandler(void)               IRQ019_Handler(void)
#define TIM6_IRQHandler(void)               IRQ020_Handler(void)
#define PCA_IRQHandler(void)                IRQ021_Handler(void)
#define WDT_IRQHandler(void)                IRQ022_Handler(void)
#define RTC_IRQHandler(void)                IRQ023_Handler(void)
#define ADC_IRQHandler(void)                IRQ024_Handler(void)
#define PCNT_IRQHandler(void)               IRQ025_Handler(void)
#define VC0_IRQHandler(void)                IRQ026_Handler(void)
#define VC1_IRQHandler(void)                IRQ027_Handler(void)
#define LVD_IRQHandler(void)                IRQ028_Handler(void)
#define LCD_IRQHandler(void)                IRQ029_Handler(void)
#define FLASH_RAM_IRQHandler(void)          IRQ030_Handler(void)
#define CLK_TRIM_IRQHandler(void)           IRQ031_Handler(void)
        
/******************************************************************************/
/*                                                                            */
/*                      START OF USER SETTINGS HERE                           */
/*                      ===========================                           */
/*                                                                            */
/*                 All lines with '<<<' can be set by user.                   */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/



#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_HC32L136_H__ */







