#include "Driver_USBVCP.h"
#include "usbd_cdc_core.c"
#include "usbd_cdc_vcp.c"
#include "usbd_desc.c"
#include "usb_bsp.c"
#include "usbd_usr.c"
#include "string.h"

/******************************外部调用函数************************************/
void usbVCP_Init(uint8_t PreemptionPriority, uint8_t SubPriority);
void usbVCP_SendByte(unsigned char ucByte);
void usbVCP_SendBuffer(unsigned char* pucBuf,unsigned int ulLength);
void usbVCP_SendString(unsigned char* pucStr);
u32 usbVCP_Printf(const char* format, ...);
__weak void usbVCP_DataRX_Handle(uint8_t* Buf, uint32_t Len);
/*****************************************************************************/

extern USB_STATUS_e usbStatus;	//	USB状态：USB_STATUS_CONFIGURED,	//已配置状态
																				//	USB_STATUS_SUSPENDED,		//暂停、挂起状态
																				//	USB_STATUS_RESUMED,			//恢复状态
																				//	USB_STATUS_CONNECTED,		//连接状态
																				//	USB_STATUS_DISCONNECTED,//断开连接状态

/*
***************************************************
函数名：usbVCP_DataRX_Handle
功能：USB虚拟串口接收处理函数
入口参数：	Buf：接收到数据的指针
						Len：接收到的长度
返回值：无
应用范围：外部调用
备注：可以把本函数当做一个中断函数，接收到的数据为BUF，长度为Len
			这里定义的是弱函数，允许用户在别的文件下编写一样的函数
***************************************************
*/
__weak void usbVCP_DataRX_Handle(uint8_t* Buf, uint32_t Len)
{
	//接收数据处理
//	if(Len){
//		for(uint16_t i = 0;i < Len;i++)
////			ANO_DT_Data_Receive_Prepare(Buf[i]);
//	}
}

/*
***************************************************
函数名：usbVCP_Init
功能：USB虚拟串口初始化
入口参数：PreemptionPriority：抢占优先级
					SubPriority：子优先级
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
USB_OTG_CORE_HANDLE  USB_OTG_dev;	//USB设备
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
函数名：USB_VCP_SendByte
功能：USB虚拟串口普通发送一个字节数据
入口参数：	ucByte：发送一个字节数据
返回值：无
应用范围：外部调用
备注：
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
函数名：USB_VCP_SendBuffer
功能：USB虚拟串口普通发送一串数据
入口参数：	pucBuf：发送数据的指针
					ulLength：长度
返回值：无
应用范围：外部调用
备注：
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
函数名：USB_VCP_SendString
功能：USB虚拟串口普通发送字符串
入口参数：	pucBuf：发送字符串的指针
返回值：无
应用范围：外部调用
备注：字符串的结尾必须是 0
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
函数名：USB_VCP_Printf
功能：USB虚拟串口格式化发送数据
入口参数：	format：格式
					...：不定参数
返回值：res：发送的长度
应用范围：外部调用
备注：
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
