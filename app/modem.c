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
/** \file modem.c
 **
 **   - 2019-05-29  1.0  yangjp  First version for modem function.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "modem.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint8_t u8FrameData[FRAME_MAX_SIZE];       //帧存储缓存
uint32_t u32FrameDataIndex;                //帧存储缓存索引
uint32_t  u32FrameSize;

uint32_t u32FrameRecvOverTime;             //帧接收超时计数器，在(1ms)定时器中断中计数，在串口中断中清零

en_frame_recv_status_t enFrameRecvStatus;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static uint16_t FLASH_PageNumber(uint32_t u32Size);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief 应答帧处理
 **
 ** \param [out] u8TxBuff               发送缓存指针
 ** \param [in] u16TxLength             待发送数据长度
 **
 ** \retval None
 **
 ******************************************************************************/
void Modem_SendFrame(uint8_t *u8TxBuff, uint16_t u16TxLength)
{
    uint16_t u16Crc16;

    u8TxBuff[FRAME_LENGTH_INDEX] = u16TxLength & 0x00FF;                   //存储数据包长度
    u8TxBuff[FRAME_LENGTH_INDEX + 1] = u16TxLength >> 8;
    u16Crc16 = Cal_CRC16(&u8TxBuff[FRAME_PACKET_INDEX], u16TxLength);      //计算数据包的CRC校验值
    u8TxBuff[FRAME_PACKET_INDEX + u16TxLength] = u16Crc16 & 0x00FF;        //存储CRC至数据包
    u8TxBuff[FRAME_PACKET_INDEX + u16TxLength + 1] = u16Crc16 >> 8;
    UART_SendData(&u8TxBuff[0], FRAME_PACKET_INDEX + u16TxLength + 2);     //发送应答帧
}


/**
 *******************************************************************************
 ** \brief 上位机数据帧解析及处理
 **
 ** \param [in] None             
 **
 ** \retval Ok                          APP程序升级完成，并接受到跳转至APP命令
 ** \retval OperationInProgress         数据处理中
 ** \retval Error                       通讯错误
 **
 ******************************************************************************/
en_result_t Modem_Process(void)
{
    uint8_t  u8Cmd, u8FlashAddrValid, u8Cnt, u8Ret;
    uint16_t u16DataLength, u16PageNum, u16Ret;
    uint32_t u32FlashAddr, u32FlashLength, u32Temp;
    
    if (enFrameRecvStatus == FRAME_RECV_PROC_STATUS)                //有数据帧待处理, enFrameRecvStatus值在串口中断中调整
    {
        u8Cmd = u8FrameData[PACKET_CMD_INDEX];                      //获取帧指令码
        if (PACKET_CMD_TYPE_DATA == u8FrameData[PACKET_TYPE_INDEX]) //如果是数据指令
        {
            u8FlashAddrValid = 0u;
            
            u32FlashAddr = u8FrameData[PACKET_ADDRESS_INDEX] +      //读取地址值
                           (u8FrameData[PACKET_ADDRESS_INDEX + 1] << 8)  +
                           (u8FrameData[PACKET_ADDRESS_INDEX + 2] << 16) +
                           (u8FrameData[PACKET_ADDRESS_INDEX + 3] << 24);
            if ((u32FlashAddr >= (FLASH_BASE + BOOT_SIZE)) && (u32FlashAddr < (FLASH_BASE + FLASH_SIZE)))  //如果地址值在有效范围内
            {
                u8FlashAddrValid = 1u;                              //标记地址有效
            }
        }
        
        switch (u8Cmd)                                              //根据指令码跳转执行
        {
            case  PACKET_CMD_HANDSHAKE    :                         //握手帧 指令码
                u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_OK;   //返回状态为：正确
                Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE);   //发送应答帧给上位机
                break;
            case  PACKET_CMD_ERASE_FLASH  :                         //擦除flash 指令码
                if ((u32FlashAddr % FLASH_SECTOR_SIZE) != 0)        //如果擦除地址不是页首地址
                {
                    u8FlashAddrValid = 0u;                          //标记地址无效
                }

                if (1u == u8FlashAddrValid)                         //如果地址有效
                {
                    u32Temp = u8FrameData[PACKET_DATA_INDEX] +      //获取待擦除flash尺寸
                              (u8FrameData[PACKET_DATA_INDEX + 1] << 8)  +
                              (u8FrameData[PACKET_DATA_INDEX + 2] << 16) +
                              (u8FrameData[PACKET_DATA_INDEX + 3] << 24);
                    u16PageNum = FLASH_PageNumber(u32Temp);          //计算需擦除多少页
                    for (u8Cnt=0; u8Cnt<u16PageNum; u8Cnt++)         //根据需要擦除指定数量的扇区
                    {
                        u8Ret = Flash_EraseSector(u32FlashAddr + (u8Cnt * FLASH_SECTOR_SIZE));
                        if (Ok != u8Ret)                             //如果擦除失败，反馈上位机错误代码
                        {
                            u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_ERROR;
                            break;
                        }
                    }
                    if (Ok == u8Ret)                                 //如果全部擦除成功，反馈上位机成功
                    {
                        u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_OK;
                    }else                                            //如果擦除失败，反馈上位机错误超时标志
                    {
                        u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_TIMEOUT;
                    }
                }
                else                                                 //地址无效，反馈上位机地址错误
                {
                    u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_ADDR_ERROR;
                }
                Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE);             //发送应答帧到上位机
                break;
            case  PACKET_CMD_APP_DOWNLOAD :                          //数据下载 指令码
                if (1u == u8FlashAddrValid)                          //如果地址有效
                {
                    u16DataLength = u8FrameData[FRAME_LENGTH_INDEX] + (u8FrameData[FRAME_LENGTH_INDEX + 1] << 8)
                                     - PACKET_INSTRUCT_SEGMENT_SIZE; //获取数据包中的数据长度(不包含指令码指令类型等等)
                    if (u16DataLength > PACKET_DATA_SEGMENT_SIZE)    //如果数据长度大于最大长度
                    {
                        u16DataLength = PACKET_DATA_SEGMENT_SIZE;    //设置数据最大值
                    }
                    u8Ret = Flash_WriteBytes(u32FlashAddr, (uint8_t *)&u8FrameData[PACKET_DATA_INDEX], u16DataLength); //把所有数据写入flash
                    if (Ok != u8Ret)                                 //如果写数据失败       
                    {
                        u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_ERROR;                //反馈上位机错误 标志
                    }
                    else                                             //如果写数据成功
                    {
                        u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_OK;                   //反馈上位机成功 标志
                    }
                }
                else                                                 //如果地址无效
                {
                    u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_ADDR_ERROR;               //反馈上位机地址错误
                }
                Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE);             //发送应答帧到上位机
                break;
            case  PACKET_CMD_CRC_FLASH    :                          //查询flash校验值 指令码
                if (1u == u8FlashAddrValid)                          //如果地址有效
                {
                    u32FlashLength = u8FrameData[PACKET_DATA_INDEX] +                 
                                    (u8FrameData[PACKET_DATA_INDEX + 1] << 8)  +
                                    (u8FrameData[PACKET_DATA_INDEX + 2] << 16) +
                                    (u8FrameData[PACKET_DATA_INDEX + 3] << 24);             //获取待校验flash大小
                    if ((u32FlashLength + u32FlashAddr) > (FLASH_BASE + FLASH_SIZE))        //如果flash长度超出有效范围
                    {
                        u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_FLASH_SIZE_ERROR;     //反馈上位机flash尺寸错误
                    }else
                    {
                        u16Ret = Cal_CRC16(((unsigned char *)u32FlashAddr), u32FlashLength);//读取flash指定区域的值并计算crc值
                        u8FrameData[PACKET_FLASH_CRC_INDEX] = (uint8_t)u16Ret;              //把crc值存储到应答帧
                        u8FrameData[PACKET_FLASH_CRC_INDEX+1] = (uint8_t)(u16Ret>>8);
                        u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_OK;                   //反馈上位机成功 标志
                    }
                }
                else                                                                        //如果地址无效
                {
                    u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_ADDR_ERROR;               //反馈上位机地址错误
                }
                Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE+2);           //发送应答帧到上位机
                break;
            case  PACKET_CMD_JUMP_TO_APP  :                          //跳转至APP 指令码
                Flash_EraseSector(BOOT_PARA_ADDRESS);                //擦除BOOT parameter 扇区
                u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_OK;    //反馈上位机成功
                Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE);             //发送应答帧到上位机
                return Ok;                                           //APP更新完成，返回OK，接下来执行跳转函数，跳转至APP
            case  PACKET_CMD_APP_UPLOAD   :                          //数据上传
                if (1u == u8FlashAddrValid)                          //如果地址有效
                {
                    u32Temp = u8FrameData[PACKET_DATA_INDEX] +
                              (u8FrameData[PACKET_DATA_INDEX + 1] << 8)  +
                              (u8FrameData[PACKET_DATA_INDEX + 2] << 16) +
                              (u8FrameData[PACKET_DATA_INDEX + 3] << 24);                   //读取上传数据长度
                    if (u32Temp > PACKET_DATA_SEGMENT_SIZE)                                 //如果数据长度大于最大值
                    {
                        u32Temp = PACKET_DATA_SEGMENT_SIZE;                                 //设置数据长度为最大值
                    }
                    Flash_ReadBytes(u32FlashAddr, (uint8_t *)&u8FrameData[PACKET_DATA_INDEX], u32Temp); //读flash数据
                    u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_OK;                       //反馈上位机成功 标志
                    Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE + u32Temp);//发送应答帧到上位机
                }
                else                                                  //如果地址无效
                {
                    u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_ADDR_ERROR;               //反馈上位机地址错误 标志
                    Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE);         //发送应答帧到上位机
                }
                break;
            case  PACKET_CMD_START_UPDATE :                           //启动APP更新(此指令正常在APP程序中调用)
                u8FrameData[PACKET_RESULT_INDEX] = PACKET_ACK_OK;     //反馈上位机成功 标志
                Modem_SendFrame(&u8FrameData[0], PACKET_INSTRUCT_SEGMENT_SIZE);             //发送应答帧到上位机
                break;
        }
        enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;                   //帧数据处理完成，帧接收状态恢复到空闲状态
    }
    
    return OperationInProgress;                                       //返回，APP更新中。。。
}


/**
 *******************************************************************************
 ** \brief 计算有多少sector页待擦除
 **
 ** \param [out] None
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static uint16_t FLASH_PageNumber(uint32_t u32Size)
{
    uint16_t u32PageNum = u32Size / FLASH_SECTOR_SIZE;

    if ((u32Size % FLASH_SECTOR_SIZE) != 0)
    {
        u32PageNum += 1u;
    }

    return u32PageNum;
}


/**
 *******************************************************************************
 ** \brief 串口中断，接收上位机发送帧
 **
 ** \param [out] None
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void UART_IRQHandler(void)
{
    uint8_t   recvData;
    uint16_t  u16Crc16;
    static uint8_t led_falg = 1;
//    led_falg = !led_falg;
//    SetBit(((uint32_t)&M0P_GPIO->P0OUT + KT_LED_PORT), KT_LED_PIN, led_falg);
    if (HC32_GetUartErrStatus())                                        //获取串口异常标志位
    {
        HC32_ClrUartErrStatus();                                        //清除串口异常标志位
    }
    
    if (HC32_GetUartRCStatus())                                         //获取串口数据接收标志位
    {
        HC32_ClrUartRCStatus();                                         //清除串口数据接收标志位
        
        u32FrameRecvOverTime = 0;                                       //帧接收超时计数器，清零
        recvData = HC32_GetUartBuff();                                  //获取串口接收数据
        switch(enFrameRecvStatus)
        {
            case FRAME_RECV_IDLE_STATUS     :                           //当前处于空闲状态
                if (recvData == FRAME_HEAD_L)                           //收到帧头第一个字节
                {
                    u8FrameData[FRAME_HEAD_H_INDEX] = recvData;         //保存数据
                    enFrameRecvStatus = FRAME_RECV_HEADER_STATUS;       //帧接收进入下一状态:  空闲状态
                }
                break;
            case FRAME_RECV_HEADER_STATUS   :                           //当前处于接收帧头状态
                if (recvData == FRAME_HEAD_H)                           //收到帧头第二个字节
                {
                    u8FrameData[FRAME_HEAD_L_INDEX] = recvData;         //保存数据
                    u32FrameDataIndex = FRAME_NUM_INDEX;                //数组下标从帧头的下一位置开始计数
                    enFrameRecvStatus = FRAME_RECV_DATA_STATUS;         //帧接收进入下一状态:  接收帧数据状态
                }else if (recvData == FRAME_HEAD_L)                     //收到帧头第一个字节
                {
                    u8FrameData[FRAME_HEAD_H_INDEX] = recvData;         //保存数据
                    enFrameRecvStatus = FRAME_RECV_HEADER_STATUS;       //帧接收进入下一状态
                }else                                                   //数据错误
                {
                    enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;         //帧接收恢复到初始状态:  空闲状态
                }
                break;
            case FRAME_RECV_DATA_STATUS     :                           //当前处于接收帧数据状态
                u8FrameData[u32FrameDataIndex++] = recvData;
                if (u32FrameDataIndex == (FRAME_NUM_INDEX+2))           //已经接收到数据帧序号及校验值
                {
                    if ((u8FrameData[FRAME_NUM_INDEX] != (u8FrameData[FRAME_XORNUM_INDEX] ^ FRAME_NUM_XOR_BYTE)))                    //数据帧序号及校验值不匹配
                    {
                        enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;     //帧接收恢复到初始状态
                        return;                                         //错误返回
                    }
                }else if (u32FrameDataIndex == (FRAME_LENGTH_INDEX+2))  //已经收到包长度数据
                {
                    u32FrameSize = u8FrameData[FRAME_LENGTH_INDEX] + (u8FrameData[FRAME_LENGTH_INDEX + 1] << 8) + FRAME_SHELL_SIZE;  //计算此帧的长度
                    if ((u32FrameSize < FRAME_MIN_SIZE) || (u32FrameSize > FRAME_MAX_SIZE))  //帧长度不在有效范围内
                    {
                        enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;     //帧接收恢复到初始状态
                        return;                                         //错误返回
                    }
                }else if ((u32FrameDataIndex > (FRAME_LENGTH_INDEX+2)) && (u32FrameDataIndex == u32FrameSize))                       //帧数据接收完毕
                {
                    u16Crc16 = u8FrameData[u32FrameDataIndex-2] + (u8FrameData[u32FrameDataIndex-1]<<8);
                    if (Cal_CRC16(&u8FrameData[FRAME_PACKET_INDEX], (u32FrameSize-FRAME_SHELL_SIZE)) == u16Crc16)                    //如果CRC校验通过
                    {
                        enFrameRecvStatus = FRAME_RECV_PROC_STATUS;     //帧接收进入下一状态:  帧处理状态                    
                    }else                                               //校验失败
                    {
                        enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;     //帧接收恢复到初始状态
                        return;                                         //错误返回
                    }
                }
                break;
            case FRAME_RECV_PROC_STATUS     :                           //当前处于帧处理状态
                break;
        }
    }
}


/**
 *******************************************************************************
 ** \brief 定时器中断函数，1ms中断一次，用于监测串口数据接收超时，及帧数据处理超时
 **
 ** \param [out] None
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Tim_IRQHandler(void)
{
    if (HC32_GetTimUIFStatus())                                         //获取定时器溢出中断标志位
    {
        HC32_ClrTimUIFStatus();                                         //清除定时器溢出中断标志位
        
        u32FrameRecvOverTime++;        
        if ((enFrameRecvStatus == FRAME_RECV_HEADER_STATUS) || (enFrameRecvStatus == FRAME_RECV_DATA_STATUS))   //处于帧接收过程中
        {
            if (u32FrameRecvOverTime++ > 10)                            //超过10ms没有收到到数据，异常
            {
                enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;             //帧接收恢复到初始状态
            }
        }else if (enFrameRecvStatus == FRAME_RECV_PROC_STATUS)          //处于帧处理状态
        {
            if (u32FrameRecvOverTime++ > 4500)                          //超过4.5s没有收到到数据，异常 (上位机超过5s没收到应答帧，会重发数据帧)
            {
                enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;             //帧接收恢复到初始状态
            }
        }
    }
}

/**
 *******************************************************************************
 ** \brief modem文件中相关变量参数初始化
 **
 ** \param [out] None
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Modem_RamInit(void)
{    
    uint32_t i;
    
    enFrameRecvStatus = FRAME_RECV_IDLE_STATUS;                         //帧状态初始化为空闲状态
    
    for (i=0; i<FRAME_MAX_SIZE; i++)
    {
        u8FrameData[i] = 0;                                             //帧数据缓存初始化为零
    }
    
    u32FrameDataIndex = 0;                                              //帧缓存数组索引值初始化为零
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
