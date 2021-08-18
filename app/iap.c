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
/** \file iap.c
 **
 **   - 2019-05-29  1.0  yangjp  First version for IAP function.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "iap.h"
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
uint32_t JumpAddress;
func_ptr_t JumpToApplication;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static en_result_t IAP_JumpToApp(uint32_t u32Addr);
static void IAP_ResetConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  IAP 初始化
 **
 ** \param  [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void IAP_Init(void)
{
    PreiModule_Init();
    Modem_RamInit();
}

/**
 *******************************************************************************
 ** \brief  IAP APP程序升级主函数.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void IAP_Main(void)
{
    en_result_t enRet;

    while (1)
    {
        enRet = Modem_Process();                       //APP程序更新处理
        if (Ok == enRet)
        {
            IAP_ResetConfig();                         //复位所有外设模块
            if (Error == IAP_JumpToApp(APP_ADDRESS))   //如果跳转失败
            {
                while(1) {
                };
            }
        }
    }
}

/**
 *******************************************************************************
 ** \brief  检查BootPara标记区数据值，判断是否需要升级APP程序.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void IAP_UpdateCheck(void)
{
    uint32_t u32AppFlag;
    
    u32AppFlag = *(__IO uint32_t *)BOOT_PARA_ADDRESS; //读出BootLoader para区标记值
    

    
    if (APP_FLAG != u32AppFlag)                       //如果标记值不等于APP_FLAG,表示不需要升级APP程序
    {
        IAP_JumpToApp(APP_ADDRESS);                   //则直接跳转至APP
    }    
}

/**
 *******************************************************************************
 ** \brief  IAP跳转函数
 **
 ** \param  [in] u32Addr                    APP 首地址
 **
 ** \retval Error                           APP 地址错误
 **
 ******************************************************************************/
static en_result_t IAP_JumpToApp(uint32_t u32Addr)
{
    uint32_t u32StackTop = *((__IO uint32_t *)u32Addr);  //读取APP程序栈顶地址

    /* 判断栈顶地址有效性 */
    if ((u32StackTop > SRAM_BASE) && (u32StackTop <= (SRAM_BASE + RAM_SIZE)))
    {
        /* 配置跳转到用户程序复位中断入口 */
        JumpAddress = *(__IO uint32_t *)(u32Addr + 4);
        JumpToApplication = (func_ptr_t)JumpAddress;
        /* 初始化用户程序的栈顶指针 */
        __set_MSP(*(__IO uint32_t *)u32Addr);
        JumpToApplication();
    }

    return Error;
}

/**
 *******************************************************************************
 ** \brief BootLoader复位配置
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void IAP_ResetConfig(void)
{
    PreiModule_DeInit();
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
