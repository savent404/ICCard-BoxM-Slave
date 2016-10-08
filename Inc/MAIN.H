#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f0xx.h"

/////////////////////////////////////////////////////////////////////
//结构体声明
/////////////////////////////////////////////////////////////////////
struct TranSciveBuffer {
	unsigned char MfCommand;
	unsigned int  MfLength;
	unsigned char MfData[64];
};
/////////////////////////////////////////////////////////////////////
//函数原型
/////////////////////////////////////////////////////////////////////
extern void Usr_Define_Delay(int x);
#define DelayMs(x) HAL_Delay(x)
#define Delay_50us(x) Usr_Define_Delay(x)

extern int BSP_IDCard_Init(void);
extern int BSP_IDCard_Request(void);
extern int BSP_ICCard_UID(unsigned char *pt);
#endif


