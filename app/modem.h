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
/** \file modem.h
 **
 **   - 2019-05-29  1.0  yangjp  First version for modem function.
 **
 ******************************************************************************/
#ifndef __MODEM_H__
#define __MODEM_H__

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
/**
 *******************************************************************************
 ** \brief Packet status enumeration
 ******************************************************************************/
typedef enum
{
    FRAME_RECV_IDLE_STATUS       = 0x00,
    FRAME_RECV_HEADER_STATUS     = 0x01,
    FRAME_RECV_DATA_STATUS       = 0x02,
    FRAME_RECV_PROC_STATUS       = 0x03,
} en_frame_recv_status_t;

/**
 *******************************************************************************
 ** \brief Packet status enumeration
 ******************************************************************************/
typedef enum
{
    PACKET_ACK_OK                = 0x00,
    PACKET_ACK_ERROR             = 0x01,
    PACKET_ACK_ABORT             = 0x02,
    PACKET_ACK_TIMEOUT           = 0x03,
    PACKET_ACK_ADDR_ERROR        = 0x04,
    PACKET_ACK_FLASH_SIZE_ERROR  = 0x05,
} en_packet_status_t;

/**
 *******************************************************************************
 ** \brief Packet command enumeration
 ******************************************************************************/
typedef enum
{
    PACKET_CMD_HANDSHAKE       = 0x20,
    PACKET_CMD_JUMP_TO_APP     = 0x21,
    PACKET_CMD_APP_DOWNLOAD    = 0x22,
    PACKET_CMD_APP_UPLOAD      = 0x23,
    PACKET_CMD_ERASE_FLASH     = 0x24,
    PACKET_CMD_CRC_FLASH       = 0x25,
    PACKET_CMD_START_UPDATE    = 0x26,
} en_packet_cmd_t;

/**
 *******************************************************************************
 ** \brief Packet command type enumeration
 ******************************************************************************/
typedef enum
{
    PACKET_CMD_TYPE_CONTROL     = 0x11,
    PACKET_CMD_TYPE_DATA        = 0x12,
} en_packet_cmd_type_t;

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define FRAME_HEAD_L                        0x6Du
#define FRAME_HEAD_H                        0xACu

/* Frame and packet size */
#define FRAME_SHELL_SIZE                    8
#define PACKET_INSTRUCT_SEGMENT_SIZE        10
#define PACKET_DATA_SEGMENT_SIZE            512
#define FRAME_MIN_SIZE                      PACKET_INSTRUCT_SEGMENT_SIZE
#define FRAME_MAX_SIZE                      (PACKET_DATA_SEGMENT_SIZE + PACKET_INSTRUCT_SEGMENT_SIZE + FRAME_SHELL_SIZE)

/* Frame structure defines */
#define FRAME_HEAD_H_INDEX                  0x00
#define FRAME_HEAD_L_INDEX                  0x01
#define FRAME_NUM_INDEX                     0x02
#define FRAME_XORNUM_INDEX                  0x03
#define FRAME_LENGTH_INDEX                  0x04
#define FRAME_PACKET_INDEX                  0x06

#define FRAME_RECV_TIMEOUT                  500             // ms
#define FRAME_NUM_XOR_BYTE                  0xFF

/* Packet structure defines */
#define PACKET_CMD_INDEX                   (FRAME_PACKET_INDEX + 0x00)
#define PACKET_TYPE_INDEX                  (FRAME_PACKET_INDEX + 0x01)
#define PACKET_RESULT_INDEX                (FRAME_PACKET_INDEX + 0x01)
#define PACKET_ADDRESS_INDEX               (FRAME_PACKET_INDEX + 0x02)
#define PACKET_FLASH_CRC_INDEX             (FRAME_PACKET_INDEX + 0x0A)
#define PACKET_DATA_INDEX                  (FRAME_PACKET_INDEX + PACKET_INSTRUCT_SEGMENT_SIZE)

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
en_result_t Modem_Process(void);
void Modem_RamInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODEM_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
