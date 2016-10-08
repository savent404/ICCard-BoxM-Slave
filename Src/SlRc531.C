#include <string.h>
#include "main.h"
#include "slrc531.h"

#define FSD 64
/*
#define Gpbase	              0x7F00                    //感应模块地址 		P2^7
#define outportb(addr,d)	XBYTE[addr]=d			 //写入addr地址值D	/		绝对地址
#define inportb(addr)		XBYTE[addr]			  ///读出addr地址值
*/


/*
unsigned char ReadRawRC(unsigned char Address)
{
	return inportb(Gpbase+Address);
}


void  WriteRawRC(unsigned char Address, unsigned char value)
{  
	outportb(Gpbase+Address,value);
}
*/






/////////////////////////////////////////////////////////////////////
//复位并初始化RC632
//注意:RC500上电后应延时500ms才能可靠初始化
/////////////////////////////////////////////////////////////////////
int PcdReset(void)
{
   int status = MI_OK;
   char n = 0xFF;
   unsigned int i = 3000;

	 /*
   RC531_CE=0;
   RC531_RST=0;
   DelayMs(50);
   RC531_RST=1;
   DelayMs(5);
   RC531_RST=0;
   DelayMs(5);
	 */
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
	DelayMs(50);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
	DelayMs(5);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
	DelayMs(5);

   while (i!=0 && n)
   {
      n = ReadRawRC(RegCommand);
      i--;
   }

   if (i != 0)
   {
      WriteRawRC(RegPage,0x80);
      n = 0x80;
      while ( (i!=0) && (n&0x80) )
      {
          n = ReadRawRC(RegCommand);
          i--;
      }
      if (i==0 || (n&0xFF))
      {   status = MI_RESETERR;   }
   }
   else
   {    status = MI_RESETERR;     }
   
   if (status == MI_OK)
   {    WriteRawRC(RegPage,0x0);  }
   
   return status;
}

//////////////////////////////////////////////////////////////////////
//设置RC632的工作方式 
//////////////////////////////////////////////////////////////////////
int PcdConfigISOType(unsigned char type)
{
   
   if (type == 'A')                    //ISO14443_A
   { 
       ClearBitMask(RegControl,0x08);

       WriteRawRC(RegClockQControl,0x0);
       WriteRawRC(RegClockQControl,0x40);
       Delay_50us(2);                   // wait approximately 100 us - calibration in progress
       ClearBitMask(RegClockQControl,0x40);
       
       WriteRawRC(RegTxControl,0x5b);
       WriteRawRC(RegCwConductance,0x0F);
       WriteRawRC(RegModConductance,0x3F);       
       WriteRawRC(RegCoderControl,0x19);
       WriteRawRC(RegModWidth,0x13);             
       WriteRawRC(RegModWidthSOF,0x00);          
       WriteRawRC(RegTypeBFraming,0x00);
       
       WriteRawRC(RegRxControl1,0x73);
       WriteRawRC(RegDecoderControl,0x08);
       WriteRawRC(RegBitPhase,0xAD);	
       WriteRawRC(RegRxThreshold,0xAA);
       WriteRawRC(RegBPSKDemControl,0);
       WriteRawRC(RegRxControl2,0x41);

       WriteRawRC(RegRxWait,0x06);
       WriteRawRC(RegChannelRedundancy,0x0F);    
       WriteRawRC(RegCRCPresetLSB,0x63);
       WriteRawRC(RegCRCPresetMSB,0x63);
       WriteRawRC(RegTimeSlotPeriod,0x00);
       WriteRawRC(RegMfOutSelect,0x01);
       WriteRawRC(RFU27,0x00);   	      

       WriteRawRC(RegFIFOLevel,0x3F);
       WriteRawRC(RegTimerClock,0x07);
       WriteRawRC(RegTimerReload,0x0A);
       WriteRawRC(RegTimerControl,0x06);   
       WriteRawRC(RegIRqPinConfig,0x02);      
       WriteRawRC(RFU2E,0x00);
       WriteRawRC(RFU2F,0x00);	  

       PcdSetTmo(106);
       DelayMs(1);
       PcdAntennaOn();	  //开启天线

   }
   else if (type == 'B')
   {
       ClearBitMask(RegControl,0x08);

       WriteRawRC(RegClockQControl,0x0);
       WriteRawRC(RegClockQControl,0x40);
       Delay_50us(2);  
       ClearBitMask(RegClockQControl,0x40);
       
       WriteRawRC(RegTxControl,0x4B);
       WriteRawRC(RegCwConductance,0x17);
       WriteRawRC(RegModConductance,0x06);       
       WriteRawRC(RegCoderControl,0x20);
       WriteRawRC(RegModWidth,0x13);             
       WriteRawRC(RegModWidthSOF,0x3F);          
       WriteRawRC(RegTypeBFraming,0x3B);
       
       WriteRawRC(RegRxControl1,0x73);
       WriteRawRC(RegDecoderControl,0x19);
       WriteRawRC(RegBitPhase,0xAD);	
       WriteRawRC(RegRxThreshold,0x88);
       WriteRawRC(RegBPSKDemControl,0x7E);
       WriteRawRC(RegRxControl2,0x01);

       WriteRawRC(RegRxWait,0x06);
       WriteRawRC(RegChannelRedundancy,0x2C);    
       WriteRawRC(RegCRCPresetLSB,0xFF);
       WriteRawRC(RegCRCPresetMSB,0xFF);
       WriteRawRC(RegTimeSlotPeriod,0x00);
       WriteRawRC(RegMfOutSelect,0x01);
       WriteRawRC(RFU27,0x00);   	      

       WriteRawRC(RegFIFOLevel,0x3F);
       WriteRawRC(RegTimerClock,0x07);
       WriteRawRC(RegTimerReload,0x0A);
       WriteRawRC(RegTimerControl,0x06);  
       WriteRawRC(RegIRqPinConfig,0x02);       
       WriteRawRC(RFU2E,0x00);
       WriteRawRC(RFU2F,0x00);
       PcdSetTmo(106);
       DelayMs(1);
       PcdAntennaOn();
   }

   else{ return -1; }
   return MI_OK;
}


/////////////////////////////////////////////////////////////////////
//置RC632寄存器位
//input:reg=寄存器地址
//      mask=置位值
/////////////////////////////////////////////////////////////////////
void SetBitMask(unsigned char reg,unsigned char mask)  
{
   unsigned char tmp = 0x0;
   tmp = ReadRawRC(reg);
   WriteRawRC(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//清RC632寄存器位
//input:reg=寄存器地址
//      mask=清位值
/////////////////////////////////////////////////////////////////////
void ClearBitMask(unsigned char reg,unsigned char mask)  
{
   unsigned char tmp = 0x0;
   tmp = ReadRawRC(reg);
   WriteRawRC(reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//设置RC632定时器
//input:tmolength=设置值
/////////////////////////////////////////////////////////////////////
void PcdSetTmo(unsigned char tmoLength)
{
   switch(tmoLength)
   {  
      case 0:                         // (0.302 ms) FWI=0
         WriteRawRC(RegTimerClock,0x07); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,0x21);// TReloadVal = 'h21 =33(dec) 
         break;
      case 1:                         // (0.604 ms) FWI=1
         WriteRawRC(RegTimerClock,0x07); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 2:                         // (1.208 ms) FWI=2
         WriteRawRC(RegTimerClock,0x07); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 3:                         // (2.416 ms) FWI=3
         WriteRawRC(RegTimerClock,0x09); // TAutoRestart=0,TPrescale=4*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 4:                         // (4.833 ms) FWI=4
         WriteRawRC(RegTimerClock,0x09); // TAutoRestart=0,TPrescale=4*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 5:                         // (9.666 ms) FWI=5
         WriteRawRC(RegTimerClock,0x0B); // TAutoRestart=0,TPrescale=16*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 6:                         // (19.33 ms) FWI=6
         WriteRawRC(RegTimerClock,0x0B); // TAutoRestart=0,TPrescale=16*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 7:                         // (38.66 ms) FWI=7
         WriteRawRC(RegTimerClock,0x0D); // TAutoRestart=0,TPrescale=64*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 8:                         // (77.32 ms) FWI=8
         WriteRawRC(RegTimerClock,0x0D); // TAutoRestart=0,TPrescale=64*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 9:                         // (154.6 ms) FWI=9
         WriteRawRC(RegTimerClock,0x0F); // TAutoRestart=0,TPrescale=256*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 10:                        // (309.3 ms) FWI=10
         WriteRawRC(RegTimerClock,0x0F); // TAutoRestart=0,TPrescale=256*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 11:                        // (618.6 ms) FWI=11
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x11);// TReloadVal = 'h21 =17(dec) 
         break;
      case 12:                        // (1.2371 s) FWI=12
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x21);// TReloadVal = 'h41 =33(dec) 
         break;
      case 13:                        // (2.4742 s) FWI=13
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h81 =65(dec) 
         break;
      case 14:                        // (4.9485 s) FWI=14
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 15:                        // (4.9485 s) FWI=14
         WriteRawRC(RegTimerClock,0x9); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x0ff);// TReloadVal = 'h81 =129(dec) 
         break;
      default:                       // 
         WriteRawRC(RegTimerClock,0x19); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
         break;
   }     
   WriteRawRC(RegTimerControl,0X06);
}

/////////////////////////////////////////////////////////////////////
//通过RC632和ISO14443卡通讯
//input: pi->MfCommand = RC632命令字
//       pi->MfLength  = 发送的数据长度
//       pi->MfData[]  = 发送数据
//output:status        = 错误字
//       pi->MfLength  = 接收的数据BIT长度
//       pi->MfData[]  = 接收数据
/////////////////////////////////////////////////////////////////////
int PcdComTransceive(struct TranSciveBuffer *pi)
{
   unsigned char recebyte = 0;
   int status;
   unsigned char irqEn   = 0x00;
   unsigned char waitFor = 0x00;
   unsigned char lastBits;
   unsigned char n;
   unsigned int i;
   switch (pi->MfCommand)
   {
      case PCD_IDLE:
         irqEn   = 0x00;
         waitFor = 0x00;
         break;
      case PCD_WRITEE2:
         irqEn   = 0x11;
         waitFor = 0x10;
         break;
      case PCD_READE2:
         irqEn   = 0x07;
         waitFor = 0x04;
         recebyte=1;
         break;
      case PCD_LOADCONFIG:
      case PCD_LOADKEYE2:
      case PCD_AUTHENT1:
         irqEn   = 0x05;
         waitFor = 0x04;
         break;
      case PCD_CALCCRC:
         irqEn   = 0x11;
         waitFor = 0x10;
         break;
      case PCD_AUTHENT2:
         irqEn   = 0x04;
         waitFor = 0x04;
         break;
      case PCD_RECEIVE:
         irqEn   = 0x06;
         waitFor = 0x04;
         recebyte=1;
         break;
      case PCD_LOADKEY:
         irqEn   = 0x05;
         waitFor = 0x04;
         break;
      case PCD_TRANSMIT:
         irqEn   = 0x05;
         waitFor = 0x04;
         break;
      case PCD_TRANSCEIVE:
         irqEn   = 0x3D;
         waitFor = 0x04;
         recebyte=1;
         break;
      default:
         pi->MfCommand = MI_UNKNOWN_COMMAND;
         break;
   }
   
   if (pi->MfCommand != MI_UNKNOWN_COMMAND)
   {
      WriteRawRC(RegPage,0x00);
      WriteRawRC(RegInterruptEn,0x7F);
      WriteRawRC(RegInterruptRq,0x7F);
      WriteRawRC(RegCommand,PCD_IDLE);
      SetBitMask(RegControl,0x01);
      WriteRawRC(RegInterruptEn,irqEn|0x80);
      for (i=0; i<pi->MfLength; i++)
      {
         WriteRawRC(RegFIFOData, pi->MfData[i]);
      }
      WriteRawRC(RegCommand, pi->MfCommand);
      i = 0x3500;
      do
      {
         n = ReadRawRC(RegInterruptRq);
         i--;
      }
      while ((i!=0) && !(n&irqEn&0x20) && !(n&waitFor));
      status = MI_COM_ERR;
      if ((i!=0) && !(n&irqEn&0x20))
      {
         if (!(ReadRawRC(RegErrorFlag)&0x17))
         {
            status = MI_OK;
            if (recebyte)
            {
              	n = ReadRawRC(RegFIFOLength);
              	lastBits = ReadRawRC(RegSecondaryStatus) & 0x07;
                if (lastBits)
                {
                   pi->MfLength = (n-1)*8 + lastBits;
                }
                else
                {
                   pi->MfLength = n*8;
                }
                if (n == 0)
                {
                   n = 1;
                }
                for (i=0; i<n; i++)
                {
                    pi->MfData[i] = ReadRawRC(RegFIFOData);
                }
            }
         }
		 else if (ReadRawRC(RegErrorFlag)&0x01)
         {
		    status = MI_COLLERR;
            if (recebyte)
            {
              	n = ReadRawRC(RegFIFOLength);
              	lastBits = ReadRawRC(RegSecondaryStatus) & 0x07;
                if (lastBits)
                {
                   pi->MfLength = (n-1)*8 + lastBits;
                }
                else
                {
                   pi->MfLength = n*8;
                }
                if (n == 0)
                {
                   n = 1;
                }
                for (i=0; i<n; i++)
                {
                    pi->MfData[i+1] = ReadRawRC(RegFIFOData);
                }
            }
			pi->MfData[0]=ReadRawRC(0x0B);
         }

      }
      else if (n & irqEn & 0x20)
      {   status = MI_NOTAGERR;   }
      else if (!(ReadRawRC(RegErrorFlag)&0x17))
      {   status = MI_ACCESSTIMEOUT;   }
      else
      {   status = MI_COM_ERR;    }
      
      WriteRawRC(RegInterruptEn,0x7F);
      WriteRawRC(RegInterruptRq,0x7F);
      SetBitMask(RegControl,0x04);           // stop timer now
      WriteRawRC(RegCommand,PCD_IDLE); 
   }
   return status;
}


/*
/////////////////////////////////////////////////////////////////////
//读RC632EEPROM
//input :startaddr=EEPROM地址（低位在前）
//       length=读字节数
//output:readdata=读出的数据
/////////////////////////////////////////////////////////////////////
char PcdReadE2(unsigned int startaddr,unsigned char length,unsigned char *readdata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    MfComData.MfCommand = PCD_READE2;
    MfComData.MfLength  = 3;
    MfComData.MfData[0] = startaddr&0xFF;
    MfComData.MfData[1] = (startaddr >> 8) & 0xFF;
    MfComData.MfData[2] = length;

    status = PcdComTransceive(pi);

    if (status == MI_OK)
    {   memcpy(readdata, &MfComData.MfData[0], length);    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//写RC632EEPROM
//input :startaddr=EEPROM地址（低位在前）
//       length=写字节数
//       writedata=要写入的数据
/////////////////////////////////////////////////////////////////////
char PcdWriteE2(unsigned int startaddr,unsigned char length,unsigned char *writedata)
{
    char status;
    struct TranSciveBuffer MfComData;    
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    MfComData.MfCommand = PCD_WRITEE2;
    MfComData.MfLength  = length+2;
    MfComData.MfData[0] = startaddr&0xFF;
    MfComData.MfData[1] = (startaddr >> 8) & 0xFF;
    memcpy(&MfComData.MfData[2], writedata, length);    

    status = PcdComTransceive(pi);
    return status;
}
*/

/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
int PcdAntennaOn()
{
  static unsigned char i;
	while(1){
	    i = ReadRawRC(RegTxControl);
	    if (i & 0x03)
	    {   return MI_OK;	}
	    else
	    {
	        SetBitMask(RegTxControl, 0x03);
	        //return MI_OK;
	    }
	}
}

/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
int PcdAntennaOff()
{
    ClearBitMask(RegTxControl, 0x03);
    return MI_OK;
}


