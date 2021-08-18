/**
 *******************************************************************************
 * @file  hc32_common.h
 * @brief This file contains the common part of the HC32 series.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2019-08-21       Chenw           First version
 @endverbatim
 *******************************************************************************
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
 * of the software.common part.
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
 *******************************************************************************
 */
#ifndef __HC32_COMMON_H__
#define __HC32_COMMON_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>

/**
 * @addtogroup CMSIS
 * @{
 */

/**
 * @addtogroup HC32_Common_Part
 * @{
 */


/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 * @defgroup HC32_Common_Global_Types HC32 Common Global Types
 * @{
 */

/**
 * @brief Single precision floating point number (4 byte)
 */
typedef float float32_t;

/**
 * @brief Double precision floating point number (8 byte)
 */
typedef double float64_t;

/**
 * @brief Function pointer type to void/void function
 */
typedef void (*func_ptr_t)(void);

/**
 * @brief Function pointer type to void/uint8_t function
 */
typedef void (*func_ptr_arg1_t)(uint8_t);

/**
 * @brief Functional state
 */
typedef enum
{
    Disable = 0u,
    Enable  = 1u,
} en_functional_state_t;

/* Check if it is a functional state */
#define IS_FUNCTIONAL_STATE(state)      (((state) == Disable) || ((state) == Enable))

/**
 * @brief boolean state
 */
typedef enum
{
    FALSE  = 0u,
    TRUE   = 1u,
} en_boolean_state_t;

/**
 * @brief Flag status
 */
typedef enum
{
    Reset = 0u,
    Set   = 1u,
} en_flag_status_t, en_int_status_t;

/**
 * @brief Generic error codes
 */
typedef enum
{
    Ok                       = 0u,   /*!< No error */
    Error                    = 1u,   /*!< Non-specific error code */
    ErrorAddressAlignment    = 2u,   /*!< Address alignment does not match */
    ErrorAccessRights        = 3u,   /*!< Wrong mode (e.g. user/system) mode is set */
    ErrorInvalidParameter    = 4u,   /*!< Provided parameter is not valid */
    ErrorOperationInProgress = 5u,   /*!< A conflicting or requested operation is still in progress */
    ErrorInvalidMode         = 6u,   /*!< Operation not allowed in current mode */
    ErrorUninitialized       = 7u,   /*!< Module (or part of it) was not initialized properly */
    ErrorBufferFull          = 8u,   /*!< Circular buffer can not be written because the buffer is full */
    ErrorTimeout             = 9u,   /*!< Time Out error occurred (e.g. I2C arbitration lost, Flash time-out, etc.) */
    ErrorNotReady            = 10u,  /*!< A requested final state is not reached */
    OperationInProgress      = 11u,  /*!< Indicator for operation in progress (e.g. ADC conversion not finished, DMA channel used, etc.) */
} en_result_t;

/**
 * @}
 */

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup HC32_Common_Global_Macros HC32 Common Global Macros
 * @{
 */

/**
 * @brief Compiler Macro Definitions
 */
//#if defined (__ICCARM__)                /*!< IAR Compiler */
//    #define __WEAKDEF                   __weak
//    #define __ALIGN_BEGIN               _Pragma("data_alignment=4")
//    #define __NOINLINE                  _Pragma("optimize = no_inline")
//    #define __RAM_FUNC                  __ramfunc
//#elif defined (__CC_ARM) || (__GNUC__)  /*!< ARM Compiler */
    #define __WEAKDEF                   __attribute__((weak))
    #define __ALIGN_BEGIN               __align(4)
    #define __NOINLINE                  __attribute__((noinline))
    /* RAM functions are defined using the toolchain options. 
    Functions that are executed in RAM should reside in a separate source module.
    Using the 'Options for File' dialog you can simply change the 'Code / Const' 
    area of a module to a memory space in physical RAM. */
//    #define __RAM_FUNC
//#else
//    #error  "unsupported compiler!!"
//#endif

/**
 * @defgroup Extend_Macro_Definitions Extend Macro Definitions
 * @{
 */
/* Decimal to BCD */
#define DEC2BCD(x)                      ((((x) / 10) << 4) + ((x) % 10))

/* BCD to decimal */
#define BCD2DEC(x)                      ((((x) >> 4) * 10) + ((x) & 0x0F))

/* Returns the dimension of an array */
#define ARRAY_SZ(X)                     (sizeof(X) / sizeof(X[0]))
/**
 * @}
 */

/**
 * @defgroup Register_Macro_Definitions Register Macro Definitions
 * @{
 */
#define SET_BIT(REG, BIT)               ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)             ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)              ((REG) & (BIT))

#define CLEAR_REG(REG)                  ((REG) = (0x0))

#define WRITE_REG(REG, VAL)             ((REG) = (VAL))

#define READ_REG(REG)                   ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
/**
 * @}
 */

/**
 * @}
 */

/**
 * @brief HC32 Common Device Include
 */
#if defined(HC32L19x)
    #include "hc32l19x\hc32l19x.h"
    #include "hc32l19x\system_hc32l19x.h"
    #include "hc32l19x\utils.h"
#elif defined(HC32L17x)
    #include "hc32l17x\hc32l17x.h"
    #include "hc32l17x\system_hc32l17x.h"
    #include "hc32l17x\utils.h"
#elif defined(HC32L07x)
    #include "hc32l07x\hc32l07x.h"
    #include "hc32l07x\system_hc32l07x.h"
    #include "hc32l07x\utils.h"
#elif defined(HC32L13x)
    #include "hc32l13x\hc32l13x.h"
    #include "hc32l13x\system_hc32l13x.h"
    #include "hc32l13x\utils.h"
#elif defined(HC32L110)
    #include "hc32l110\hc32l110.h"
    #include "hc32l110\system_hc32l110.h"
    #include "hc32l110\utils.h"
#elif defined(HC32F005)
    #include "hc32f005\hc32f005.h"
    #include "hc32f005\system_hc32f005.h"
    #include "hc32f005\utils.h"
#else
    #error "Please select first the target HC32xxxx device used in your application (in hc32xxxx.h file)"
#endif

#include "modem.h"
#include "basic.h"
#include "iap.h"
/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
 * Global function prototypes (definition in C source)
 ******************************************************************************/

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __HC32_COMMON_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

