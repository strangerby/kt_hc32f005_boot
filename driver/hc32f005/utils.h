/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co., Ltd. ("HDSC").
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
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
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
/** \file utils.h
 **
 **   - 2019-08-22  1.0  Chenw  First version for UTILS module.
 **
 ******************************************************************************/
#ifndef __UTILS_H__
#define __UTILS_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_common.h"


/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* 串口相关宏定义. */
#define BR_9600                     (9600ul)                          //波特率9600Hz
#define BR_19200                    (19200ul)                         //波特率19200Hz
#define BR_38400                    (38400ul)                         //波特率38400Hz
#define BR_57600                    (57600ul)                         //波特率57600Hz
#define BR_76800                    (76800ul)                         //波特率76800Hz
#define BR_115200                   (115200ul)                        //波特率115200Hz  
#define BR_256000                   (256000ul)                        //波特率256000Hz 
#define BR_512000                   (512000ul)                        //波特率512000Hz 
#define BR_921600                   (921600ul)                        //波特率921600Hz 
/* 当前串口通讯波特率选择 */
#define BAUD_RATE                   BR_9600                         //当前系统波特率选择
/* 串口中断函数名重定义 */
#define UART_IRQHandler(void)       UART1_IRQHandler(void)
/* 定时器中断函数名重定义 */
#define Tim_IRQHandler(void)        TIM2_IRQHandler(void)

/* 系统时钟相关宏定义. */
#define SystemClockFreq             24000000                          //系统时钟，单位Hz
#define SystemClockMHz              24                                //系统时钟，单位MHz
    
/* Flash相关宏定义 */
#define FLASH_SECTOR_SIZE           0x200ul                           //一个sector的尺寸
#define FLASH_BASE                  ((uint32_t)0x00000000)            //flash基地址
#define FLASH_SIZE                  (64u * FLASH_SECTOR_SIZE)        //flash尺寸
/* RAM相关宏定义 */
#define SRAM_BASE                   ((uint32_t)0x20000000)            //RAM基地址
#define RAM_SIZE                    0x1000ul                          //RAM尺寸
/* BootLoader flash相关宏定义 */
#define BOOT_SIZE                   (8*FLASH_SECTOR_SIZE)             //BootLoader flash尺寸
#define BOOT_PARA_ADDRESS           (FLASH_BASE + BOOT_SIZE - 0x100u) //BootLoader para存储地址
/* APP flash相关宏定义 */
#define APP_FLAG                    ((uint32_t)0x67890123)            //从BootLoader para区读到此值，表示APP需要更新
#define APP_ADDRESS                 (FLASH_BASE + BOOT_SIZE)          //APP程序存储基地址
/* Flash擦写频率配置 */
#define FLASH_CONFIG_FREQ_4MHZ      0x01                              //flash擦写时间参数按4MHz配置，在flash初始化函数中使用
#define FLASH_CONFIG_FREQ_22_12MHZ  0x02                              //flash擦写时间参数按22.12MHz配置，在flash初始化函数中使用
#define FLASH_CONFIG_FREQ_24MHZ     0x02                              //flash擦写时间参数按22.12MHz配置，在flash初始化函数中使用

/************************************************************ 系统时钟相关操作相关函数 ********************************************************************/





typedef uint8_t      boolean_t;

__STATIC_INLINE void SetBit(uint32_t addr, uint32_t offset, boolean_t bFlag)
{
    if(TRUE == bFlag)
    {
        *((volatile uint32_t *)(addr)) |= ((1UL)<<(offset));
    }
    else
    {
        *((volatile uint32_t *)(addr)) &= (~(1UL<<(offset)));
    }    
    
}

__STATIC_INLINE boolean_t GetBit(uint32_t addr, uint32_t offset)
{
    return ((((*((volatile uint32_t *)(addr))) >> (offset)) & 1u) > 0) ? 1 : 0;
}


/**
 *******************************************************************************
 ** \brief 设置系统时钟为RCH 12.12MHz.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_SetSystemClockToRCH24MHz(void)
{

    M0P_SYSCTRL->SYSCTRL0_f.XTH_EN = 0;
    M0P_SYSCTRL->SYSCTRL0_f.RCH_EN = 1;
    M0P_SYSCTRL->SYSCTRL0_f.CLKSW = 0;
    M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
    M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS = 7;
    M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C08 ) ); //4M
    M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C06 ) ); //8M
    M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C04 ) ); //16M
    M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C00 ) ); //24M
    M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
    M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS = 0;
//    

//    M0P_SYSCTRL->RCH_CR_f.TRIM = (*((volatile uint16_t*) (0x00100C06ul)));    //Loader 8MHz Trimming value 
//    M0P_SYSCTRL->RCH_CR_f.TRIM = (*((volatile uint16_t*) (0x00100C04ul)));    //Loader 16MHz Trimming value 
//    M0P_SYSCTRL->RCH_CR_f.TRIM = (*((volatile uint16_t*) (0x00100C02ul)));    //Loader 22.12MHz Trimming value 

//    M0P_SystemCtrl->SYSCTRL2 = 0X5A5A;
//M0P_SystemCtrl->SYSCTRL2 = 0XA5A5;
//M0P_SystemCtrl->SYSCTRL0_f.HCLK_PRS = 7;
//M0P_SystemCtrl->RCH_CR = *((uint16 *)( 0X00100C08 ) ); //4M
//M0P_SystemCtrl->RCH_CR = *((uint16 *)( 0X00100C06 ) ); //8M
//M0P_SystemCtrl->RCH_CR = *((uint16 *)( 0X00100C04 ) ); //16M
//M0P_SystemCtrl->RCH_CR = *((uint16 *)( 0X00100C00 ) ); //24M
//M0P_SystemCtrl->SYSCTRL2 = 0X5A5A;
//M0P_SystemCtrl->SYSCTRL2 = 0XA5A5;
//M0P_SystemCtrl->SYSCTRL0_f.HCLK_PRS = 0;
    
}

/**
 *******************************************************************************
 ** \brief 恢复系统时钟到默认RCH 4MHz.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_RestSystemClockToRCH4MHz(void)
{
    M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
    M0P_SYSCTRL->RCH_CR_f.TRIM = (*((volatile uint16_t*) (0x00100C04ul)));    //Loader 16MHz Trimming value 
    M0P_SYSCTRL->RCH_CR_f.TRIM = (*((volatile uint16_t*) (0x00100C06ul)));    //Loader 8MHz Trimming value 
    M0P_SYSCTRL->RCH_CR_f.TRIM = (*((volatile uint16_t*) (0x00100C08ul)));    //Loader 4MHz Trimming value 
    M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
}

/************************************************************ CRC操作相关函数 ********************************************************************/

/**
 *******************************************************************************
 ** \brief 初始化CRC模块
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_InitCRC(void)
{
    M0P_SYSCTRL->PERI_CLKEN_f.CRC = 1;
    //M0P_SYSCTRL->PERI_CLKEN |= (1 << 26u);
}

/**
 *******************************************************************************
 ** \brief 复位CRC模块
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_DeInitCRC(void)
{
    M0P_SYSCTRL->PERI_CLKEN_f.CRC = 0;
    //M0P_SYSCTRL->PERI_CLKEN &= ~(1 << 26u);
}

/**
 *******************************************************************************
 ** \brief CRC16 编码(字节填充方式)，该函数主要用于生成CRC16编码.
 **
 ** \param [in]  pu8Data          待编码数据指针（字节方式输入）
 ** \param [in]  u32Len           待编码数据长度（字节数）
 ** 
 ** \retval CRC16                 CRC16编码值. 
 ******************************************************************************/
__STATIC_INLINE uint16_t HC32_CRC16Get8(const uint8_t* pu8Data, uint32_t u32Len)
{
    uint32_t u32Index = 0;
    
    //M0P_CRC->CR_f.CR = 0;       //16位
    M0P_CRC->RESULT_f.RESULT = 0xA28C;;                      //设置初始值，与上位机对应
    for(u32Index = 0;u32Index<u32Len;u32Index++)
    {
        *((volatile uint8_t*)(&(M0P_CRC->DATA)))  = pu8Data[u32Index];
    }

    return (M0P_CRC->RESULT_f.RESULT);
}


/************************************************************ 串口操作相关函数 ********************************************************************/

/**
 *******************************************************************************
 ** \brief 获取串口异常状态
 **
 ** \param [in] None
 **
 ** \retval 获取串口数据接收标志位
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t HC32_GetUartErrStatus(void)
{
    return (READ_BIT(M0P_UART1->ISR, 0x04));
}

/**
 *******************************************************************************
 ** \brief 清除串口异常状态位
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_ClrUartErrStatus(void)
{
    CLEAR_BIT(M0P_UART1->ICR, 0x04);
}

/**
 *******************************************************************************
 ** \brief 获取串口数据接收状态
 **
 ** \param [in] None
 **
 ** \retval 获取串口数据接收标志位
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t HC32_GetUartRCStatus(void)
{
    return (READ_BIT(M0P_UART1->ISR, 0x01));
}

/**
 *******************************************************************************
 ** \brief 清除串口数据接收状态位
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_ClrUartRCStatus(void)
{
    CLEAR_BIT(M0P_UART1->ICR, 0x01);
}

/**
 *******************************************************************************
 ** \brief 获取串口接收数据
 **
 ** \param [in] None
 **
 ** \retval 串口接收buff数据
 **
 ******************************************************************************/
__STATIC_INLINE uint8_t HC32_GetUartBuff()
{
    return (M0P_UART1->SBUF_f.SBUF);
}

#define     GpioPort0 (0x00u)                 ///< GPIO PORT 0
#define     GpioPort1 (0x40u)                ///< GPIO PORT 1
#define     GpioPort2 (0x80u)                ///< GPIO PORT 2
#define     GpioPort3 (0xc0u)                 ///< GPIO PORT 3

#define     GpioPin0    (0u)                 ///< GPIO PIN0 
#define     GpioPin1    (1u)                 ///< GPIO PIN1 
#define     GpioPin2    (2u)                 ///< GPIO PIN2 
#define     GpioPin3    (3u)                ///< GPIO PIN3 
#define     GpioPin4    (4u)                ///< GPIO PIN4 
#define     GpioPin5    (5u)                 ///< GPIO PIN5 
#define     GpioPin6    (6u)                ///< GPIO PIN6 
#define     GpioPin7    (7u)                 ///< GPIO PIN7

#define LORA_RX_PROT        GpioPort2
#define LORA_RX_PIN         GpioPin3
#define LORA_TX_PROT        GpioPort2
#define LORA_TX_PIN         GpioPin4
#define LORA_MD0_PROT       GpioPort1
#define LORA_MD0_PIN        GpioPin5
#define LORA_MD1_PROT       GpioPort1
#define LORA_MD1_PIN        GpioPin4
#define KT_LED_PORT         GpioPort0
#define KT_LED_PIN          GpioPin3
#define LORA_AUX_PROT       GpioPort2
#define LORA_AUX_PIN        GpioPin5



__STATIC_INLINE void HC32_boot_lora_init(void)
{
    M0P_SYSCTRL->PERI_CLKEN_f.GPIO = 1;
    //TX
        //配置为默认值,GPIO功能
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_RX_PROT, LORA_RX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_RX_PROT) + (((uint32_t)LORA_RX_PIN)<<2))) = 0;

    //方向配置
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_RX_PROT), LORA_RX_PIN, 0);
    //驱动能力配置
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_RX_PROT), LORA_RX_PIN, 0);
    //上拉/下拉配置
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_RX_PROT), LORA_RX_PIN, 1);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_RX_PROT), LORA_RX_PIN, 0);
    //开漏输出功能
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_RX_PROT), LORA_RX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_RX_PROT) + (((uint32_t)LORA_RX_PIN)<<2))) = 6u;

    //Rx
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_TX_PROT, LORA_TX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_TX_PROT) + (((uint32_t)LORA_TX_PIN)<<2))) = 0;

    //方向配置
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_TX_PROT), LORA_TX_PIN, 1);
    //驱动能力配置
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_TX_PROT), LORA_TX_PIN, 0);
    //上拉/下拉配置
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_TX_PROT), LORA_TX_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_TX_PROT), LORA_TX_PIN, 0);
    //开漏输出功能
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_TX_PROT), LORA_TX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) +  LORA_TX_PROT) + (((uint32_t)LORA_TX_PIN)<<2))) = 6u;

    //MD0
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_MD0_PROT, LORA_MD0_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_MD0_PROT) + (((uint32_t)LORA_MD0_PIN)<<2))) = 0;
    
        //方向配置
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    //驱动能力配置
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    //上拉/下拉配置
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_MD0_PROT), LORA_MD0_PIN, 1);
    //开漏输出功能
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0OUT + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    
    //MD1
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_MD1_PROT, LORA_MD1_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_MD1_PROT) + (((uint32_t)LORA_MD1_PIN)<<2))) = 0;
    
        //方向配置
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    //驱动能力配置
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    //上拉/下拉配置
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_MD1_PROT), LORA_MD1_PIN, 1);
    //开漏输出功能
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0OUT + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    
    //KT_LED
    SetBit((uint32_t)&M0P_GPIO->P0ADS + KT_LED_PORT, KT_LED_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + KT_LED_PORT) + (((uint32_t)KT_LED_PIN)<<2))) = 0;
    
        //方向配置
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + KT_LED_PORT), KT_LED_PIN, 0);
    //驱动能力配置
    SetBit(((uint32_t)&M0P_GPIO->P0DR + KT_LED_PORT), KT_LED_PIN, 0);
    //上拉/下拉配置
    SetBit(((uint32_t)&M0P_GPIO->P0PU + KT_LED_PORT), KT_LED_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + KT_LED_PORT), KT_LED_PIN, 1);
    //开漏输出功能
    SetBit(((uint32_t)&M0P_GPIO->P0OD + KT_LED_PORT), KT_LED_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0OUT + KT_LED_PORT), KT_LED_PIN, 1);
      
    //LORA_AUX
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_AUX_PROT, LORA_AUX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_AUX_PROT) + (((uint32_t)LORA_AUX_PIN)<<2))) = 0;
    
        //方向配置
    SetBit(((uint32_t)&M0P_GPIO->P0DIR +LORA_AUX_PROT), LORA_AUX_PIN, 1);
    //驱动能力配置
    SetBit(((uint32_t)&M0P_GPIO->P0DR +LORA_AUX_PROT), LORA_AUX_PIN, 0);
    //上拉/下拉配置
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_AUX_PROT), LORA_AUX_PIN, 1);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_AUX_PROT), LORA_AUX_PIN, 0);
    //开漏输出功能
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_AUX_PROT), LORA_AUX_PIN, 0);
    //SetBit(((uint32_t)&M0P_GPIO->P0OUT +LORA_AUX_PROT), LORA_AUX_PIN, 1);
}

/**
 *******************************************************************************
 ** \brief 初始化UART0模块
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_InitUart()
{
    
    HC32_boot_lora_init();

    M0P_SYSCTRL->PERI_CLKEN_f.BASETIM = 1;
    M0P_SYSCTRL->PERI_CLKEN_f.UART1 = 1;

    M0P_UART1->SCON_f.DBAUD = 0;
    uint16_t u16tmload = 0x10000-((SystemClockFreq)/( BR_9600 *32));//单双倍波特率，定时器配置

    M0P_TIM1->CR_f.GATEP = 0;
    M0P_TIM1->CR_f.GATE = 0;
    M0P_TIM1->CR_f.PRS = 0;
    M0P_TIM1->CR_f.TOGEN = 0;
    M0P_TIM1->CR_f.CT = 0u;
    M0P_TIM1->CR_f.MD = 1u;
    M0P_TIM1->ARR_f.ARR = u16tmload;
    M0P_TIM1->CNT_f.CNT = u16tmload;
    M0P_TIM1->CR_f.CTEN = 1;

    M0P_UART1->SCON_f.SM01 = 1u; 
   

    SetBit((uint32_t)(&(M0P_UART1->SCON)), 4, 1);
    SetBit((uint32_t)(&(M0P_UART1->SCON)), 0, 1);
    SetBit((uint32_t)(&(M0P_UART1->ICR)), 0, 0);


    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_SetPriority(UART1_IRQn, 1u);
    NVIC_EnableIRQ(UART1_IRQn);


}

/**
 *******************************************************************************
 ** \brief 复位UART0模块
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_DeInitUart(void)
{
    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_DisableIRQ(UART1_IRQn);
    
    M0P_SYSCTRL->PERI_CLKEN_f.GPIO = 0;              //关闭GPIO外设时钟门控开关
    M0P_SYSCTRL->PERI_CLKEN_f.UART1 = 0;             //关闭UART0外设时钟门控开关
    M0P_RESET->PERI_RESET_f.GPIO = 0;
    M0P_RESET->PERI_RESET_f.GPIO = 1;
    M0P_RESET->PERI_RESET_f.UART1 = 0;
    M0P_RESET->PERI_RESET_f.UART1 = 1;
    

}

/**
 *******************************************************************************
 ** \brief 串口发送一个数据字节
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_UartSendByte(uint8_t u8TxData)
{
     //发送缓存写数据
    M0P_UART1->SBUF_f.SBUF = u8TxData; 
    while ((((M0P_UART1->ISR >> 1) & 1u) > 0) == 0);
        //等待发送完成
    //M0P_UART1->ICR_f.TICLR = 0;     //清除发送完成标志位
    SetBit((uint32_t)(&(M0P_UART1->ICR)), 1, 0);
}


/************************************************************ 定时器操作相关函数 ********************************************************************/

/**
 *******************************************************************************
** \brief 获取定时器1溢出中断标志位
 **
 ** \param [in] None
 **
 ** \retval 标志寄存器值
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t HC32_GetTimUIFStatus()
{
    return (READ_BIT(M0P_TIM2->IFR, 0x01));
    
}

/**
 *******************************************************************************
** \brief 清除定时器1溢出中断标志位
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_ClrTimUIFStatus()
{
    CLEAR_BIT(M0P_TIM2->ICLR, 0x01);
}

/**
 *******************************************************************************
** \brief 初始化定时器1，周期为1ms的定时中断
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_InitTIM()
{
	M0P_SYSCTRL->PERI_CLKEN_f.BASETIM = 1;

	M0P_TIM2->CNT = 65536-((SystemClockFreq/64)/100);  //配置定时器中断周期1ms
	M0P_TIM2->ARR = 65536-((SystemClockFreq/64)/100);

	M0P_TIM2->CR_f.PRS = 6;                           //中断使能；64倍时钟分频；16位重载计数
	M0P_TIM2->CR_f.MD = 1;
    M0P_TIM2->CR_f.UIE = 1;
     
    M0P_TIM2->ICLR = 0;                               //清除中断标志位
	
    NVIC_ClearPendingIRQ(TIM2_IRQn);                        //清除TIMER1中断挂起bit
    NVIC_SetPriority(TIM2_IRQn, 2);                        //设定TIMER1优先级
	NVIC_EnableIRQ(TIM2_IRQn);                              //使能NVIC_TIMER1中断
    
	M0P_TIM2->CR_f.CTEN = 1;                        //使能定时器
}

/**
 *******************************************************************************
** \brief 复位时器1模块
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_DeInitTIM()
{
    
    NVIC_ClearPendingIRQ(TIM2_IRQn);                        //清除TIMER1中断挂起bit
    NVIC_DisableIRQ(TIM2_IRQn);                              //使能NVIC_TIMER1中断
    
    M0P_SYSCTRL->PERI_CLKEN_f.BASETIM = 0;
    M0P_RESET->PERI_RESET_f.BASETIM = 0;
    M0P_RESET->PERI_RESET_f.BASETIM = 1;
}



/************************************************************ flash操作相关函数 ********************************************************************/

/* flash超时计数值宏定义 */
#define FLASH_TIMEOUT_INIT         (0xFFu)
/* flash写序列，每次flash寄存器修改，都需调用此序列 */
#define FLASH_BYPASS()           do{M0P_FLASH->BYPASS = 0x5A5A;\
                                    M0P_FLASH->BYPASS = 0xA5A5; }while(0)
/* Flash 解锁 */
#define Flash_UnlockAll()        do{__disable_irq();\
                                    FLASH_BYPASS();\
                                    M0P_FLASH->SLOCK = 0xFFFFFFFFu;\
                                    __enable_irq();}while(0)
/* Flash 加锁 */
#define Flash_LockAll()          do{__disable_irq();\
                                    FLASH_BYPASS();\
                                    M0P_FLASH->SLOCK = 0;\
                                    __enable_irq();}while(0)
/**
 *******************************************************************************
 ** \brief flash初始化
 **
 ** \param [in] u32CfgFreq         配置擦写频率
 **
 ** \retval Ok                     成功
 ** \retval ErrorUninitialized     初始化失败
 **
 ******************************************************************************/
__STATIC_INLINE en_result_t HC32_InitFlash(uint32_t u32CfgFreq)
{
    en_result_t enResult  = Ok;
    uint32_t    u32PrgTimer_22_12MHz[8] = {(uint32_t)(0.5+8*SystemClockMHz), (uint32_t)(0.5+5.75*SystemClockMHz), (uint32_t)(0.5+6.75*SystemClockMHz), (uint32_t)(0.5+4500*SystemClockMHz),\
                                           (uint32_t)(0.5+35000*SystemClockMHz), (uint32_t)(0.5+6*SystemClockMHz), (uint32_t)(0.5+60*SystemClockMHz), (uint32_t)(0.5+250*SystemClockMHz)}; 
    uint32_t    u32PrgTimer_4MHz[8] = {0x20, 0x17, 0x1B, 0x4650, 0x222E0, 0x18, 0xF0, 0x3E8};
    uint32_t   *pu32PrgTimer;
    volatile uint32_t  *pu32PrgTimerReg = (volatile uint32_t*)M0P_FLASH;
    volatile uint8_t    u8TimeOutCnt;      
    uint32_t            u32Index  = 0;      

    if (u32CfgFreq == FLASH_CONFIG_FREQ_4MHZ)  //flash擦写时间按4MHz配置
    {
        pu32PrgTimer = u32PrgTimer_4MHz;
    }else                                      //flash擦写时间按22.12MHz配置
    {
        pu32PrgTimer = u32PrgTimer_22_12MHz;
    }
    
    for(u32Index=0; u32Index<8; u32Index++)
    {
        u8TimeOutCnt = FLASH_TIMEOUT_INIT;
        while(pu32PrgTimerReg[u32Index]  != pu32PrgTimer[u32Index])
        {
            if(u8TimeOutCnt--)
            {
                FLASH_BYPASS();
                pu32PrgTimerReg[u32Index] = pu32PrgTimer[u32Index];
            }
            else
            {
                return ErrorUninitialized;
            }
        }
    }

    return (enResult);
}

/**
 *******************************************************************************
 ** \brief flash扇区擦除
 **
 ** \param [in] u32SectorAddr     所擦除扇区内的地址
 **
 ** \retval Ok                    擦除成功.
 ** \retval ErrorInvalidParameter FLASH地址无效 
 ** \retval ErrorTimeout          操作超时
 ******************************************************************************/
__STATIC_INLINE en_result_t HC32_FlashEraseSector(uint32_t u32SectorAddr)
{
    en_result_t             enResult = Ok;    
    volatile uint32_t       u32TimeOut;
    
    u32TimeOut = FLASH_TIMEOUT_INIT;
    while(0x02 != M0P_FLASH->CR_f.OP)         //如果flash操作模式不是：页擦除
    {
        if(u32TimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = 0x02;        //配置flash操作为：页擦除
        }
        else
        {
            return ErrorTimeout;              //配置操作超时，错误返回
        }
    }        
    
    Flash_UnlockAll();                        //flash擦写 解锁    
    *((volatile uint32_t*)u32SectorAddr) = 0; //向sector内写任意数据
    u32TimeOut = FLASH_TIMEOUT_INIT;
    while (TRUE == M0P_FLASH->CR_f.BUSY)      //查询是否擦写成功
    {
        if(0 == u32TimeOut--)
        {
            Flash_LockAll();                  //Flash 加锁
            return ErrorTimeout;
        }
    }        
    Flash_LockAll();                          //Flash 加锁
    
    return (enResult);
}

/**
 *******************************************************************************
 ** \brief flash写入数据 按字节写
 **
 ** \param [in] None
 **
 ** \retval Ok             成功
 ** \retval ErrorTimeout   超时错误
 **
 ******************************************************************************/
__STATIC_INLINE en_result_t HC32_FlashWriteBytes(uint32_t u32Addr, const uint8_t *pu8WriteBuff, uint32_t u32ByteLength)
{
    en_result_t             enResult = Ok;    
    volatile uint32_t       u32TimeOut;
    uint32_t                i;    
    
    u32TimeOut = FLASH_TIMEOUT_INIT;
    while(0x01 != M0P_FLASH->CR_f.OP)                    //如果flash操作模式不是：写入模式
    {
        if(u32TimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = 0x01;                   //配置flash操作模式为写入模式
        }
        else
        {
            return ErrorTimeout;                         //超时，写入失败，错误返回
        }
    }
    
    Flash_UnlockAll();                                   //去除擦写保护    
    for (i=0; i<u32ByteLength; i++)
    {
        *((volatile uint8_t*)(u32Addr+i)) = pu8WriteBuff[i]; //写入byte        
        
        u32TimeOut = FLASH_TIMEOUT_INIT;
        while (TRUE == M0P_FLASH->CR_f.BUSY)             //检测是否写入完成
        {
            if(0 == u32TimeOut--)
            {
                return ErrorTimeout;                     //超时，写入失败
            }
        }
    }    
    Flash_LockAll();                                     //Flash 加锁，使能擦写保护
    
    return (enResult);
}

/**
 *******************************************************************************
 ** \brief flash 数据读取
 **
 ** \param [in] u32Addr                  读取起始地址
 ** \param [in] *pu8ReadBuff             数据缓存地址
 ** \param [in] u32ByteLength            读取字节长度
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_FlashReadBytes(uint32_t u32Addr, uint8_t *pu8ReadBuff, uint32_t u32ByteLength)
{
    uint32_t i;
    
    for (i=0; i<u32ByteLength; i++)
    {
        pu8ReadBuff[i] = *((unsigned char *)(u32Addr+i));
    }
}

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
