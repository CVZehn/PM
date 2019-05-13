#ifndef __DRIVER_USBVCP_H
#define __DRIVER_USBVCP_H

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

/**********************************USB初始化虚拟串口****************************************/
#define WIRELESS_VCP_PreemptionPriority 2						//WIRELESS_USART中断抢占优先级
#define WIRELESS_VCP_SubPriority 				0						//WIRELESS_USART中断响应优先级

void usbVCP_Init(uint8_t PreemptionPriority, uint8_t SubPriority);
void usbVCP_SendByte(unsigned char ucByte);
void usbVCP_SendBuffer(unsigned char* pucBuf,unsigned int ulLength);
void usbVCP_SendString(unsigned char* pucStr);
u32 usbVCP_Printf(const char* format, ...);
__weak void usbVCP_DataRX_Handle(uint8_t* Buf, uint32_t Len);

extern USB_STATUS_e usbStatus; //USB设备的状态

#endif
