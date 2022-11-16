/**
 * @file uartPack.h
 * @brief see uartPack.c for details.
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-19
 *
 * THINK DIFFERENTLY
 */

#ifndef _UART_PACK_H_
#define _UART_PACK_H_
#include "main.h"
// private define
#define _REDIRECT_UART_PORT huart1 //�ض��򴮿�Ŀ��

// typedef
//  typedef char* va_list;

// constants
#define RX_BUFFER_SIZE 128
#define _RX_DEFAILT_TIMEOUT 10
#define _RX_DEFAILT_ENDBIT '\n'

// typedef
typedef struct
{                                    //��ʱ��UART���ƽṹ��
  __IO uint8_t rxFlag;               //���ڽ��ձ�־λ
  __IO uint8_t rxSaveFlag;           //������ɱ�־λ
  uint8_t rxData[2];                 //���յ�������
  uint8_t rxBuf[RX_BUFFER_SIZE];     //���ջ�����
  __IO uint8_t rxBufIndex;           //���ջ���������
  __IO uint8_t rxSaveCounter;        //���ձ�����������
  uint8_t rxSaveBuf[RX_BUFFER_SIZE]; //���ձ��滺����
  __IO uint32_t rxTick;              //���ճ�ʱ��ʱ��
  uint32_t rxTimeout;                //���ճ�ʱʱ��
} uart_o_ctrl_t;

typedef struct
{                                    //������λ��UART���ƽṹ��
  __IO uint8_t rxFlag;               //���ڽ��ձ�־λ
  __IO uint8_t rxSaveFlag;           //������ɱ�־λ
  uint8_t rxData[2];                 //���յ�������
  uint8_t rxBuf[RX_BUFFER_SIZE];     //���ջ�����
  __IO uint8_t rxBufIndex;           //���ջ���������
  __IO uint8_t rxSaveCounter;        //���ձ�����������
  uint8_t rxSaveBuf[RX_BUFFER_SIZE]; //���ձ��滺����
  uint8_t rxEndBit;                  //���ս���λ
} uart_e_ctrl_t;

// public functions

int printft(UART_HandleTypeDef *huart, char *fmt, ...);
void Enable_Uart_O_Control(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl);
void Enable_Uart_E_Control(UART_HandleTypeDef *huart, uart_e_ctrl_t *ctrl);
uint8_t Uart_O_Data_Process(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl);
uint8_t Uart_O_Timeout_Check(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl);
uint8_t Uart_E_Data_Process(UART_HandleTypeDef *huart, uart_e_ctrl_t *ctrl);
#endif
