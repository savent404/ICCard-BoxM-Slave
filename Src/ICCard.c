#include "main.h"
#include "slrc531.h"
#include "iso14443a.h"
#include "iso14443b.h"

unsigned char g_bIblock;

unsigned char g_cFWI;                                       //
unsigned char g_cReceBuf[64];                         //???????????

static int Rc632Ready(void)
{
    char status;
    DelayMs(100);
    DelayMs(100);
		DelayMs(100);
    DelayMs(100);
    DelayMs(100);
    DelayMs(100);
    DelayMs(100);

    status = PcdReset();
    if(status != MI_OK)
    {
        DelayMs(10);
        status = PcdReset();
    } 
    if(status != MI_OK)
    {
        DelayMs(10);
        status = PcdReset();
    } 
    if(status == MI_OK)
    {
			return 0;
    }
		return -1;
}

int request(void)	   // 寻卡(二代证和其他typeB)
{
	unsigned char status;

	do{
		status = M531PiccRequestB(0, 0, 0, &g_cReceBuf[0]);//?? 
	}
	while(status);

	g_cFWI = 0xFF;
	status = M531PiccAttrib(&g_cReceBuf[1], g_cReceBuf[10]&0x0F, &g_cReceBuf[12]);

	if (status == MI_OK)
	{
		status = Get_UID_TypeB(&g_cReceBuf[0]);
		if (status == MI_OK)  //返回卡号
		{
			return 0;
		}

	}
	return -1;
}

int BSP_IDCard_Init(void) {
	int i = 0;
  i = Rc632Ready();
	if (!i) {
		PcdConfigISOType( 'A' );
		return 0;
	}
	return -1;
}

/**
 * @Retvl: \OK(0) \Err(-1)
 */
int  BSP_IDCard_Request(void) {
	DelayMs(10);
	PcdAntennaOn(); //开启
	DelayMs(10);
	
	PcdConfigISOType( 'B' );
	
	return request();
}

int BSP_ICCard_UID(unsigned char *pt) {
	int i = 10;
	unsigned char *spt = g_cReceBuf;
	while (i--) {
		*pt++ = *spt++;
	}
	return 0;
}
