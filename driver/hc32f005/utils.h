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
/* ?????????????????????. */
#define BR_9600                     (9600ul)                          //?????????9600Hz
#define BR_19200                    (19200ul)                         //?????????19200Hz
#define BR_38400                    (38400ul)                         //?????????38400Hz
#define BR_57600                    (57600ul)                         //?????????57600Hz
#define BR_76800                    (76800ul)                         //?????????76800Hz
#define BR_115200                   (115200ul)                        //?????????115200Hz  
#define BR_256000                   (256000ul)                        //?????????256000Hz 
#define BR_512000                   (512000ul)                        //?????????512000Hz 
#define BR_921600                   (921600ul)                        //?????????921600Hz 
/* ????????????????????????????????? */
#define BAUD_RATE                   BR_9600                         //???????????????????????????
/* ?????????????????????????????? */
#define UART_IRQHandler(void)       UART1_IRQHandler(void)
/* ????????????????????????????????? */
#define Tim_IRQHandler(void)        TIM2_IRQHandler(void)

/* ???????????????????????????. */
#define SystemClockFreq             24000000                          //?????????????????????Hz
#define SystemClockMHz              24                                //?????????????????????MHz
    
/* Flash??????????????? */
#define FLASH_SECTOR_SIZE           0x200ul                           //??????sector?????????
#define FLASH_BASE                  ((uint32_t)0x00000000)            //flash?????????
#define FLASH_SIZE                  (64u * FLASH_SECTOR_SIZE)        //flash??????
/* RAM??????????????? */
#define SRAM_BASE                   ((uint32_t)0x20000000)            //RAM?????????
#define RAM_SIZE                    0x1000ul                          //RAM??????
/* BootLoader flash??????????????? */
#define BOOT_SIZE                   (8*FLASH_SECTOR_SIZE)             //BootLoader flash??????
#define BOOT_PARA_ADDRESS           (FLASH_BASE + BOOT_SIZE - 0x100u) //BootLoader para????????????
/* APP flash??????????????? */
#define APP_FLAG                    ((uint32_t)0x67890123)            //???BootLoader para????????????????????????APP????????????
#define APP_ADDRESS                 (FLASH_BASE + BOOT_SIZE)          //APP?????????????????????
/* Flash?????????????????? */
#define FLASH_CONFIG_FREQ_4MHZ      0x01                              //flash?????????????????????4MHz????????????flash????????????????????????
#define FLASH_CONFIG_FREQ_22_12MHZ  0x02                              //flash?????????????????????22.12MHz????????????flash????????????????????????
#define FLASH_CONFIG_FREQ_24MHZ     0x02                              //flash?????????????????????22.12MHz????????????flash????????????????????????

/************************************************************ ???????????????????????????????????? ********************************************************************/





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
 ** \brief ?????????????????????RCH 12.12MHz.
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
 ** \brief ???????????????????????????RCH 4MHz.
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

/************************************************************ CRC?????????????????? ********************************************************************/

/**
 *******************************************************************************
 ** \brief ?????????CRC??????
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
 ** \brief ??????CRC??????
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
 ** \brief CRC16 ??????(??????????????????)??????????????????????????????CRC16??????.
 **
 ** \param [in]  pu8Data          ?????????????????????????????????????????????
 ** \param [in]  u32Len           ????????????????????????????????????
 ** 
 ** \retval CRC16                 CRC16?????????. 
 ******************************************************************************/
__STATIC_INLINE uint16_t HC32_CRC16Get8(const uint8_t* pu8Data, uint32_t u32Len)
{
    uint32_t u32Index = 0;
    
    //M0P_CRC->CR_f.CR = 0;       //16???
    M0P_CRC->RESULT_f.RESULT = 0xA28C;;                      //????????????????????????????????????
    for(u32Index = 0;u32Index<u32Len;u32Index++)
    {
        *((volatile uint8_t*)(&(M0P_CRC->DATA)))  = pu8Data[u32Index];
    }

    return (M0P_CRC->RESULT_f.RESULT);
}


/************************************************************ ???????????????????????? ********************************************************************/

/**
 *******************************************************************************
 ** \brief ????????????????????????
 **
 ** \param [in] None
 **
 ** \retval ?????????????????????????????????
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t HC32_GetUartErrStatus(void)
{
    return (READ_BIT(M0P_UART1->ISR, 0x04));
}

/**
 *******************************************************************************
 ** \brief ???????????????????????????
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
 ** \brief ??????????????????????????????
 **
 ** \param [in] None
 **
 ** \retval ?????????????????????????????????
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t HC32_GetUartRCStatus(void)
{
    return (READ_BIT(M0P_UART1->ISR, 0x01));
}

/**
 *******************************************************************************
 ** \brief ?????????????????????????????????
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
 ** \brief ????????????????????????
 **
 ** \param [in] None
 **
 ** \retval ????????????buff??????
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
        //??????????????????,GPIO??????
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_RX_PROT, LORA_RX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_RX_PROT) + (((uint32_t)LORA_RX_PIN)<<2))) = 0;

    //????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_RX_PROT), LORA_RX_PIN, 0);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_RX_PROT), LORA_RX_PIN, 0);
    //??????/????????????
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_RX_PROT), LORA_RX_PIN, 1);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_RX_PROT), LORA_RX_PIN, 0);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_RX_PROT), LORA_RX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_RX_PROT) + (((uint32_t)LORA_RX_PIN)<<2))) = 6u;

    //Rx
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_TX_PROT, LORA_TX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_TX_PROT) + (((uint32_t)LORA_TX_PIN)<<2))) = 0;

    //????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_TX_PROT), LORA_TX_PIN, 1);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_TX_PROT), LORA_TX_PIN, 0);
    //??????/????????????
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_TX_PROT), LORA_TX_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_TX_PROT), LORA_TX_PIN, 0);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_TX_PROT), LORA_TX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) +  LORA_TX_PROT) + (((uint32_t)LORA_TX_PIN)<<2))) = 6u;

    //MD0
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_MD0_PROT, LORA_MD0_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_MD0_PROT) + (((uint32_t)LORA_MD0_PIN)<<2))) = 0;
    
        //????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    //??????/????????????
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_MD0_PROT), LORA_MD0_PIN, 1);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0OUT + LORA_MD0_PROT), LORA_MD0_PIN, 0);
    
    //MD1
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_MD1_PROT, LORA_MD1_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_MD1_PROT) + (((uint32_t)LORA_MD1_PIN)<<2))) = 0;
    
        //????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DR + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    //??????/????????????
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_MD1_PROT), LORA_MD1_PIN, 1);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0OUT + LORA_MD1_PROT), LORA_MD1_PIN, 0);
    
    //KT_LED
    SetBit((uint32_t)&M0P_GPIO->P0ADS + KT_LED_PORT, KT_LED_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + KT_LED_PORT) + (((uint32_t)KT_LED_PIN)<<2))) = 0;
    
        //????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DIR + KT_LED_PORT), KT_LED_PIN, 0);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DR + KT_LED_PORT), KT_LED_PIN, 0);
    //??????/????????????
    SetBit(((uint32_t)&M0P_GPIO->P0PU + KT_LED_PORT), KT_LED_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + KT_LED_PORT), KT_LED_PIN, 1);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0OD + KT_LED_PORT), KT_LED_PIN, 0);
    SetBit(((uint32_t)&M0P_GPIO->P0OUT + KT_LED_PORT), KT_LED_PIN, 1);
      
    //LORA_AUX
    SetBit((uint32_t)&M0P_GPIO->P0ADS + LORA_AUX_PROT, LORA_AUX_PIN, 0);
    *((uint32_t*)(((uint32_t)((&(M0P_GPIO->P01_SEL)) - 1u) + LORA_AUX_PROT) + (((uint32_t)LORA_AUX_PIN)<<2))) = 0;
    
        //????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DIR +LORA_AUX_PROT), LORA_AUX_PIN, 1);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0DR +LORA_AUX_PROT), LORA_AUX_PIN, 0);
    //??????/????????????
    SetBit(((uint32_t)&M0P_GPIO->P0PU + LORA_AUX_PROT), LORA_AUX_PIN, 1);
    SetBit(((uint32_t)&M0P_GPIO->P0PD + LORA_AUX_PROT), LORA_AUX_PIN, 0);
    //??????????????????
    SetBit(((uint32_t)&M0P_GPIO->P0OD + LORA_AUX_PROT), LORA_AUX_PIN, 0);
    //SetBit(((uint32_t)&M0P_GPIO->P0OUT +LORA_AUX_PROT), LORA_AUX_PIN, 1);
}

/**
 *******************************************************************************
 ** \brief ?????????UART0??????
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
    uint16_t u16tmload = 0x10000-((SystemClockFreq)/( BR_9600 *32));//????????????????????????????????????

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
 ** \brief ??????UART0??????
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
    
    M0P_SYSCTRL->PERI_CLKEN_f.GPIO = 0;              //??????GPIO????????????????????????
    M0P_SYSCTRL->PERI_CLKEN_f.UART1 = 0;             //??????UART0????????????????????????
    M0P_RESET->PERI_RESET_f.GPIO = 0;
    M0P_RESET->PERI_RESET_f.GPIO = 1;
    M0P_RESET->PERI_RESET_f.UART1 = 0;
    M0P_RESET->PERI_RESET_f.UART1 = 1;
    

}

/**
 *******************************************************************************
 ** \brief ??????????????????????????????
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_UartSendByte(uint8_t u8TxData)
{
     //?????????????????????
    M0P_UART1->SBUF_f.SBUF = u8TxData; 
    while ((((M0P_UART1->ISR >> 1) & 1u) > 0) == 0);
        //??????????????????
    //M0P_UART1->ICR_f.TICLR = 0;     //???????????????????????????
    SetBit((uint32_t)(&(M0P_UART1->ICR)), 1, 0);
}


/************************************************************ ??????????????????????????? ********************************************************************/

/**
 *******************************************************************************
** \brief ???????????????1?????????????????????
 **
 ** \param [in] None
 **
 ** \retval ??????????????????
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t HC32_GetTimUIFStatus()
{
    return (READ_BIT(M0P_TIM2->IFR, 0x01));
    
}

/**
 *******************************************************************************
** \brief ???????????????1?????????????????????
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
** \brief ??????????????????1????????????1ms???????????????
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_InitTIM()
{
	M0P_SYSCTRL->PERI_CLKEN_f.BASETIM = 1;

	M0P_TIM2->CNT = 65536-((SystemClockFreq/64)/100);  //???????????????????????????1ms
	M0P_TIM2->ARR = 65536-((SystemClockFreq/64)/100);

	M0P_TIM2->CR_f.PRS = 6;                           //???????????????64??????????????????16???????????????
	M0P_TIM2->CR_f.MD = 1;
    M0P_TIM2->CR_f.UIE = 1;
     
    M0P_TIM2->ICLR = 0;                               //?????????????????????
	
    NVIC_ClearPendingIRQ(TIM2_IRQn);                        //??????TIMER1????????????bit
    NVIC_SetPriority(TIM2_IRQn, 2);                        //??????TIMER1?????????
	NVIC_EnableIRQ(TIM2_IRQn);                              //??????NVIC_TIMER1??????
    
	M0P_TIM2->CR_f.CTEN = 1;                        //???????????????
}

/**
 *******************************************************************************
** \brief ????????????1??????
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_DeInitTIM()
{
    
    NVIC_ClearPendingIRQ(TIM2_IRQn);                        //??????TIMER1????????????bit
    NVIC_DisableIRQ(TIM2_IRQn);                              //??????NVIC_TIMER1??????
    
    M0P_SYSCTRL->PERI_CLKEN_f.BASETIM = 0;
    M0P_RESET->PERI_RESET_f.BASETIM = 0;
    M0P_RESET->PERI_RESET_f.BASETIM = 1;
}



/************************************************************ flash?????????????????? ********************************************************************/

/* flash???????????????????????? */
#define FLASH_TIMEOUT_INIT         (0xFFu)
/* flash??????????????????flash??????????????????????????????????????? */
#define FLASH_BYPASS()           do{M0P_FLASH->BYPASS = 0x5A5A;\
                                    M0P_FLASH->BYPASS = 0xA5A5; }while(0)
/* Flash ?????? */
#define Flash_UnlockAll()        do{__disable_irq();\
                                    FLASH_BYPASS();\
                                    M0P_FLASH->SLOCK = 0xFFFFFFFFu;\
                                    __enable_irq();}while(0)
/* Flash ?????? */
#define Flash_LockAll()          do{__disable_irq();\
                                    FLASH_BYPASS();\
                                    M0P_FLASH->SLOCK = 0;\
                                    __enable_irq();}while(0)
/**
 *******************************************************************************
 ** \brief flash?????????
 **
 ** \param [in] u32CfgFreq         ??????????????????
 **
 ** \retval Ok                     ??????
 ** \retval ErrorUninitialized     ???????????????
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

    if (u32CfgFreq == FLASH_CONFIG_FREQ_4MHZ)  //flash???????????????4MHz??????
    {
        pu32PrgTimer = u32PrgTimer_4MHz;
    }else                                      //flash???????????????22.12MHz??????
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
 ** \brief flash????????????
 **
 ** \param [in] u32SectorAddr     ???????????????????????????
 **
 ** \retval Ok                    ????????????.
 ** \retval ErrorInvalidParameter FLASH???????????? 
 ** \retval ErrorTimeout          ????????????
 ******************************************************************************/
__STATIC_INLINE en_result_t HC32_FlashEraseSector(uint32_t u32SectorAddr)
{
    en_result_t             enResult = Ok;    
    volatile uint32_t       u32TimeOut;
    
    u32TimeOut = FLASH_TIMEOUT_INIT;
    while(0x02 != M0P_FLASH->CR_f.OP)         //??????flash??????????????????????????????
    {
        if(u32TimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = 0x02;        //??????flash?????????????????????
        }
        else
        {
            return ErrorTimeout;              //?????????????????????????????????
        }
    }        
    
    Flash_UnlockAll();                        //flash?????? ??????    
    *((volatile uint32_t*)u32SectorAddr) = 0; //???sector??????????????????
    u32TimeOut = FLASH_TIMEOUT_INIT;
    while (TRUE == M0P_FLASH->CR_f.BUSY)      //????????????????????????
    {
        if(0 == u32TimeOut--)
        {
            Flash_LockAll();                  //Flash ??????
            return ErrorTimeout;
        }
    }        
    Flash_LockAll();                          //Flash ??????
    
    return (enResult);
}

/**
 *******************************************************************************
 ** \brief flash???????????? ????????????
 **
 ** \param [in] None
 **
 ** \retval Ok             ??????
 ** \retval ErrorTimeout   ????????????
 **
 ******************************************************************************/
__STATIC_INLINE en_result_t HC32_FlashWriteBytes(uint32_t u32Addr, const uint8_t *pu8WriteBuff, uint32_t u32ByteLength)
{
    en_result_t             enResult = Ok;    
    volatile uint32_t       u32TimeOut;
    uint32_t                i;    
    
    u32TimeOut = FLASH_TIMEOUT_INIT;
    while(0x01 != M0P_FLASH->CR_f.OP)                    //??????flash?????????????????????????????????
    {
        if(u32TimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = 0x01;                   //??????flash???????????????????????????
        }
        else
        {
            return ErrorTimeout;                         //????????????????????????????????????
        }
    }
    
    Flash_UnlockAll();                                   //??????????????????    
    for (i=0; i<u32ByteLength; i++)
    {
        *((volatile uint8_t*)(u32Addr+i)) = pu8WriteBuff[i]; //??????byte        
        
        u32TimeOut = FLASH_TIMEOUT_INIT;
        while (TRUE == M0P_FLASH->CR_f.BUSY)             //????????????????????????
        {
            if(0 == u32TimeOut--)
            {
                return ErrorTimeout;                     //?????????????????????
            }
        }
    }    
    Flash_LockAll();                                     //Flash ???????????????????????????
    
    return (enResult);
}

/**
 *******************************************************************************
 ** \brief flash ????????????
 **
 ** \param [in] u32Addr                  ??????????????????
 ** \param [in] *pu8ReadBuff             ??????????????????
 ** \param [in] u32ByteLength            ??????????????????
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
