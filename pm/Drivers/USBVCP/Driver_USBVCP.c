#include "Driver_USBVCP.h"
#include "usbd_cdc_core.c"
#include "usbd_cdc_vcp.c"
#include "usbd_desc.c"
#include "usb_bsp.c"
#include "usbd_usr.c"
#include "string.h"

/******************************�ⲿ���ú���************************************/
void usbVCP_Init(uint8_t PreemptionPriority, uint8_t SubPriority);
void usbVCP_SendByte(unsigned char ucByte);
void usbVCP_SendBuffer(unsigned char* pucBuf,unsigned int ulLength);
void usbVCP_SendString(unsigned char* pucStr);
u32 usbVCP_Printf(const char* format, ...);
__weak void usbVCP_DataRX_Handle(uint8_t* Buf, uint32_t Len);
/*****************************************************************************/

extern USB_STATUS_e usbStatus;	//	USB״̬��USB_STATUS_CONFIGURED,	//������״̬
																				//	USB_STATUS_SUSPENDED,		//��ͣ������״̬
																				//	USB_STATUS_RESUMED,			//�ָ�״̬
																				//	USB_STATUS_CONNECTED,		//����״̬
																				//	USB_STATUS_DISCONNECTED,//�Ͽ�����״̬

/*
***************************************************
��������usbVCP_DataRX_Handle
���ܣ�USB���⴮�ڽ��մ�����
��ڲ�����	Buf�����յ����ݵ�ָ��
						Len�����յ��ĳ���
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע�����԰ѱ���������һ���жϺ��������յ�������ΪBUF������ΪLen
			���ﶨ������������������û��ڱ���ļ��±�дһ���ĺ���
***************************************************
*/
__weak void usbVCP_DataRX_Handle(uint8_t* Buf, uint32_t Len)
{
	//�������ݴ���
//	if(Len){
//		for(uint16_t i = 0;i < Len;i++)
////			ANO_DT_Data_Receive_Prepare(Buf[i]);
//	}
}

/*
***************************************************
��������usbVCP_Init
���ܣ�USB���⴮�ڳ�ʼ��
��ڲ�����PreemptionPriority����ռ���ȼ�
					SubPriority�������ȼ�
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
USB_OTG_CORE_HANDLE  USB_OTG_dev;	//USB�豸
uint8_t usbPreemptionPriority, usbSubPriority;
void usbVCP_Init(uint8_t PreemptionPriority, uint8_t SubPriority)
{
		  USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif  
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
}

/*
***************************************************
��������USB_VCP_SendByte
���ܣ�USB���⴮����ͨ����һ���ֽ�����
��ڲ�����	ucByte������һ���ֽ�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void usbVCP_SendByte(unsigned char ucByte)
{
	APP_Rx_Buffer[APP_Rx_ptr_in] = ucByte;
	APP_Rx_ptr_in++;
  /* To avoid buffer overflow */
  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
  {
    APP_Rx_ptr_in = 0;
  } 
}

/*
***************************************************
��������USB_VCP_SendBuffer
���ܣ�USB���⴮����ͨ����һ������
��ڲ�����	pucBuf���������ݵ�ָ��
					ulLength������
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void usbVCP_SendBuffer(unsigned char* pucBuf,unsigned int ulLength)
{
	unsigned int index = 0;
	while(index < ulLength)
	{
		usbVCP_SendByte(pucBuf[index]);
		index ++;
	}
}

/*
***************************************************
��������USB_VCP_SendString
���ܣ�USB���⴮����ͨ�����ַ���
��ڲ�����	pucBuf�������ַ�����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע���ַ����Ľ�β������ 0
***************************************************
*/
void usbVCP_SendString(unsigned char* pucStr)
{
	unsigned int index = 0;
	while(pucStr[index] != 0)
	{
		usbVCP_SendByte(pucStr[index]);
		index ++;
	}
}

#include "stdarg.h"
#include "stdio.h"
#include "string.h"

/*
***************************************************
��������USB_VCP_Printf
���ܣ�USB���⴮�ڸ�ʽ����������
��ڲ�����	format����ʽ
					...����������
����ֵ��res�����͵ĳ���
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
u32 usbVCP_Printf(const char* format, ...)
{
	
#define MAX_USB_PRINTF_LEN 128
	
	char Array_USART[MAX_USB_PRINTF_LEN];
	u16 index = 0;
	u32 res = 0;
	va_list arg_ptr;
	va_start(arg_ptr, format);
	res = vsnprintf(Array_USART, MAX_USB_PRINTF_LEN, format, arg_ptr);
	while( (index<MAX_USB_PRINTF_LEN) && Array_USART[index] )
	{
		usbVCP_SendByte(Array_USART[index]);
		index++;
	}
	return res;
}
