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
/** \file basic.c
 **
 **   - 2019-05-29  1.0  yangjp  First version for BASIC driver.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "basic.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/


/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief ?????????????????????????????????????????????RCH 22.12MHz.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void SystemClock_Init(void)
{
    HC32_SetSystemClockToRCH24MHz();
}

/**
 *******************************************************************************
 ** \brief ?????????????????????RCH 4MHz.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void SystemClock_DeInit(void)
{
    HC32_RestSystemClockToRCH4MHz();
}

/**
 *******************************************************************************
 ** \brief CPU?????????????????????
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void PreiModule_Init(void)
{
    HC32_SetSystemClockToRCH24MHz();
    HC32_InitUart();
    HC32_InitCRC();
    HC32_InitTIM();
    HC32_InitFlash(FLASH_CONFIG_FREQ_22_12MHZ);
}

/**
 *******************************************************************************
 ** \brief CPU??????????????????
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void PreiModule_DeInit(void)
{
    HC32_RestSystemClockToRCH4MHz();
    HC32_DeInitUart();
    HC32_DeInitCRC();
    HC32_DeInitTIM();
    HC32_InitFlash(FLASH_CONFIG_FREQ_4MHZ);
}

/**
 *******************************************************************************
 ** \brief  ???????????????????????????CRC16???
 **
 ** \param  *pu8Data        ??????????????????
 ** \param  u32Len          ??????????????????
 **
 ** \retval None
 **
 ******************************************************************************/
uint16_t Cal_CRC16(const uint8_t* pu8Data, uint32_t u32Len)
{
    return HC32_CRC16Get8(pu8Data, u32Len);
}

/**
 *******************************************************************************
 ** \brief  ??????????????????
 **
 ** \param  *pu8TxBuff     ?????????????????????
 ** \param  u16Length      ?????????????????????
 **
 ** \retval None
 **
 ******************************************************************************/
void UART_SendData(uint8_t *pu8TxBuff, uint16_t u16Length)
{
    while(GetBit(((uint32_t)&M0P_GPIO->P0IN + LORA_AUX_PROT), LORA_AUX_PIN) == 0);
    while (u16Length--)
    {
        HC32_UartSendByte(*pu8TxBuff);
        pu8TxBuff++;
    }
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
 **
 ******************************************************************************/
en_result_t Flash_EraseSector(uint32_t u32Addr)
{
    en_result_t enRet = Ok;

    if (u32Addr > (FLASH_BASE + FLASH_SIZE))    //?????????????????????
    {
        return ErrorInvalidParameter;
    }

    if ((u32Addr % 4) != 0)
    {
        u32Addr = (u32Addr / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    }
    
    enRet = HC32_FlashEraseSector(u32Addr);

    return enRet;
}

/**
 *******************************************************************************
 ** \brief  flash?????????
 **
 ** \param  u32Addr           ???flash?????????
 ** \param  *pu8WriteBuff     ??????????????????
 ** \param  u32ByteLength     ????????????
 **
 ** \retval Ok                ??????
 ** \retval ErrorTimeout     ????????????
 **
 ******************************************************************************/
en_result_t Flash_WriteBytes(uint32_t u32Addr, const uint8_t *pu8WriteBuff, uint32_t u32ByteLength)
{
    return HC32_FlashWriteBytes(u32Addr, pu8WriteBuff, u32ByteLength);
}

/**
 *******************************************************************************
 ** \brief  flash?????????
 **
 ** \param  u32Addr           ????????????
 ** \param  *pu8ReadBuff      ??????????????????
 ** \param  u32ByteLength     ????????????
 **
 ** \retval None
 **
 ******************************************************************************/
void Flash_ReadBytes(uint32_t u32Addr, uint8_t *pu8ReadBuff, uint32_t u32ByteLength)
{
    HC32_FlashReadBytes(u32Addr, pu8ReadBuff, u32ByteLength);
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
