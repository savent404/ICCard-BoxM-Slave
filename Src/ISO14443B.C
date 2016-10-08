#include <string.h>
#include "main.h"
#include "iso14443b.h" 
#include "slrc531.h"

extern unsigned char g_cFWI;

//////////////////////////////////////////////////////////////////////
//REQUEST B
//////////////////////////////////////////////////////////////////////
char M531PiccRequestB(unsigned char req_code, 
                      unsigned char AFI, 
                      unsigned char N, 
                      unsigned char *ATQB)
{
    char  status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi=&MfComData;

    ClearBitMask(RegControl,0x08);      // disable crypto 1 unit   
    
   // SetBitMask(RegTxControl,0x03);      // Tx2RF-En, Tx1RF-En enable

    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 3;
    MfComData.MfData[0] = ISO14443B_ANTICOLLISION;     	       // APf code
    MfComData.MfData[1] = AFI;                // 
    MfComData.MfData[2] = ((req_code<<3)&0x08) | (N&0x07);  // PARAM
 
    status = PcdComTransceive(pi);

    if (status!=MI_OK && status!=MI_NOTAGERR)
    {   status = MI_COLLERR;   }
    
    if (MfComData.MfLength != 96)
    {   status = MI_COM_ERR;   }
    
    if (status == MI_OK) 
    {	
    	memcpy(ATQB, &MfComData.MfData[0], 16);
        PcdSetTmo(ATQB[11]>>4); // set FWT 
    } 	
    return status;
}                      

//////////////////////////////////////////////////////////////////////
//SLOT-MARKER
//////////////////////////////////////////////////////////////////////
/*char M531PiccSlotMarker(unsigned char N, unsigned char *ATQB)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(5);

    if(!N || N>15) status = MI_WRONG_PARAMETER_VALUE;	
    else
    {
       MfComData.MfCommand=PCD_TRANSCEIVE;
       MfComData.MfLength=1;
       MfComData.MfData[0]=0x05|(N<<4); // APn code
        
       status=PcdComTransceive(pi);

       if (status == MI_CRCERR) status = MI_COLLERR; // collision occurs

       if (status == MI_OK) 
       {	
          memcpy(ATQB, &MfComData.MfData[0], 16);
          PcdSetTmo(ATQB[11]>>4); // set FWT 
       } 	
    }
    return status;
}                      
*/
             

//////////////////////////////////////////////////////////////////////
//ATTRIB
//////////////////////////////////////////////////////////////////////
char M531PiccAttrib(unsigned char *PUPI,unsigned char PARAM3,unsigned char *answer)
{
    char  status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
	PUPI=PUPI;
	PARAM3=PARAM3;

    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 9;
    MfComData.MfData[0] = ISO14443B_ATTRIB;
   // memcpy(&MfComData.MfData[1], PUPI, 4);
	MfComData.MfData[1] = 0x00;
	MfComData.MfData[2] = 0x00;
	MfComData.MfData[3] = 0x00;
	MfComData.MfData[4] = 0x00;
    MfComData.MfData[5] = 0x00;  	    // EOF/SOF required, default TR0/TR1
    MfComData.MfData[6] = 0x08;//FSDI; // Max frame 64 
    MfComData.MfData[7] = 0x01;//PARAM3;  	    // Param3, ISO/IEC 14443-4 compliant?
    MfComData.MfData[8] = 0x08;//1;  	    // CID
    
    status  = PcdComTransceive(pi);

    if (status == MI_OK)
    {	
    	*answer = MfComData.MfData[0];
    } 	
    
    return status;
} 
//////////////////////////////////////////////////////////////////////
//REQUEST B
//////////////////////////////////////////////////////////////////////
char Get_UID_TypeB(unsigned char *ATQB1)
{
    char  status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi=&MfComData;

    ClearBitMask(RegControl,0x08);      // disable crypto 1 unit   
    
   // SetBitMask(RegTxControl,0x03);      // Tx2RF-En, Tx1RF-En enable

    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  =5;
    MfComData.MfData[0] =0x00; //ISO14443B_ANTICOLLISION;     	       // APf code
    MfComData.MfData[1] =0x36;// AFI;                // 
    MfComData.MfData[2] =0x00; //((req_code<<3)&0x08) | (N&0x07);  // PARAM
	MfComData.MfData[3] =0x00;
	MfComData.MfData[4] =0x08;
 
    status = PcdComTransceive(pi);

 /*  if (status!=MI_OK && status!=MI_NOTAGERR)
    {   status = MI_COLLERR;   }
    
    if (MfComData.MfLength != 96)
    {   status = MI_COM_ERR;   }		   		*/
    
    if (status == MI_OK) 
    {	
    	memcpy(ATQB1, &MfComData.MfData[0], 10);
  //      PcdSetTmo(ATQB[11]>>4); // set FWT 
    } 	
    return status;
}       

//////////////////////////////////////////////////////////////////////
//HLTB
//////////////////////////////////////////////////////////////////////
char M531PiccHltb(unsigned char *PUPI)
{
    char  status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
  
    PcdSetTmo(g_cFWI);
				                               // disable, ISO/IEC3390 enable	
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = ISO14443B_ATTRIB;
    memcpy(&MfComData.MfData[1], PUPI, 4);
    
    status = PcdComTransceive(pi);

    return status;
}                      

/////////////////////////////////////////////////////////////////////
//AT88RF020验证密码
//input:password=8字节密码
/////////////////////////////////////////////////////////////////////
char At88rf020Check(unsigned char *password)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(g_cFWI);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 10;
    MfComData.MfData[0] = AT88RF020_CHECK_PASSWORD|0x10;
    MfComData.MfData[1] = 0;
    memcpy(&MfComData.MfData[2], password, 8);   

    status = PcdComTransceive(pi);
    
    if ((MfComData.MfData[1]&0x01) || (MfComData.MfLength!=0x10))
    {    status = MI_COM_ERR;    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//读AT88RF020一页数据
//input :addr=页地址
//output:readdata=读出的8字节数据
/////////////////////////////////////////////////////////////////////
char At88rf020Read(unsigned char addr, unsigned char *readdata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(g_cFWI);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 10;
    MfComData.MfData[0] = AT88RF020_READ|0x10;
    MfComData.MfData[1] = addr;
    
    status = PcdComTransceive(pi);
    
    if ((status==MI_OK) && (MfComData.MfLength==0x50))
    {   memcpy(readdata, &MfComData.MfData[2], 8);    }
    else
    {   status = MI_COM_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//写AT88RF020一页数据
//input :addr=页地址
//       writedata=要写入的8字节数据
/////////////////////////////////////////////////////////////////////
char At88rf020Write(unsigned char addr,unsigned char *writedata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(g_cFWI);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 10;
    MfComData.MfData[0] = AT88RF020_WRITE|0x10;
    MfComData.MfData[1] = addr;
    memcpy(&MfComData.MfData[2], writedata, 8);    

    status = PcdComTransceive(pi);
    
    if ((MfComData.MfData[1]&0x01) || (MfComData.MfLength!=0x10))
    {    status = MI_COM_ERR;    }
    
    return status;
}
/////////////////////////////////////////////////////////////////////
//AT88RF020一LOCK
//input :lockdata=4字节数据
/////////////////////////////////////////////////////////////////////
char At88rf020Lock(unsigned char *lockdata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(g_cFWI);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 10;
    MfComData.MfData[0] = AT88RF020_LOCK|0x10;
    MfComData.MfData[1] = 0;
    memcpy(&MfComData.MfData[2], lockdata, 4);    

    status = PcdComTransceive(pi);
    
    if ((MfComData.MfData[1]&0x01) || (MfComData.MfLength!=0x10))
    {    status = MI_COM_ERR;    }
    
    return status;
}
/////////////////////////////////////////////////////////////////////
//AT88RF020计数操作
//input :signature = 6字节签名信息
/////////////////////////////////////////////////////////////////////
char At88rf020Count(unsigned char *signature)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(g_cFWI);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 10;
    MfComData.MfData[0] = AT88RF020_COUNT|0x10;
    MfComData.MfData[1] = 0;
    memcpy(&MfComData.MfData[4], signature, 6);
    
    status = PcdComTransceive(pi);
    
    if((MfComData.MfData[1]&0x01) || (MfComData.MfLength!=0x10))
    {    status = MI_COM_ERR;    }
    
    return status;
}
/////////////////////////////////////////////////////////////////////
//AT88RF020进入HALT状态
/////////////////////////////////////////////////////////////////////
char At88rf020Deselect()
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(g_cFWI);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 10;
    MfComData.MfData[0] = AT88RF020_DESELECT|0x10;
    
    status = PcdComTransceive(pi);
    if((MfComData.MfData[1]&0x01) || (MfComData.MfLength!=0x10))
    {    status = MI_COM_ERR;    }
    
    return status;
}

//////////////////////////////////////////////////////////////////////
//激活
//////////////////////////////////////////////////////////////////////
char SelectSR(unsigned char *Chip_ID)
{
    char  status = MI_OK;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    ClearBitMask(RegControl,0x08);         // disable crypto 1 unit   
    
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = 6;     	       
    MfComData.MfData[1] = 0;                
    
    status = PcdComTransceive(pi);

    if (status!=MI_OK && status!=MI_NOTAGERR) 
    {   status = MI_COLLERR;   }          // collision occurs
    
    if(MfComData.MfLength != 8)
    {   status = MI_COM_ERR;   }
    
    if (status == MI_OK)
    {	
         PcdSetTmo(5);
         MfComData.MfCommand = PCD_TRANSCEIVE;
         MfComData.MfLength  = 2;
         MfComData.MfData[1] = MfComData.MfData[0];     	       
         MfComData.MfData[0] = 0x0E;                 
         
         status = PcdComTransceive(pi); 
         
         if (status!=MI_OK && status!=MI_NOTAGERR)  // collision occurs
         {   status = MI_COLLERR;   }               // collision occurs
         if (MfComData.MfLength != 8) 
         {   status = MI_COM_ERR;     }
         if (status == MI_OK)
         {  *Chip_ID = MfComData.MfData[0];  }
    } 	
    
    return status;
}  

//////////////////////////////////////////////////////////////////////
//SR176卡读块
//////////////////////////////////////////////////////////////////////
char ReadSR176(unsigned char addr,unsigned char *readdata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = 8;
    MfComData.MfData[1] = addr;
  
    status = PcdComTransceive(pi);
  
    if ((status==MI_OK) && (MfComData.MfLength!=16))
    {   status = MI_BITCOUNTERR;    }
    if (status == MI_OK)
    {
        *readdata     = MfComData.MfData[0];
        *(readdata+1) = MfComData.MfData[1];
    }
    return status;  
}  
//////////////////////////////////////////////////////////////////////
//SR176卡写块
//////////////////////////////////////////////////////////////////////
char WriteSR176(unsigned char addr,unsigned char *writedata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSMIT;
    MfComData.MfLength  = 4;
    MfComData.MfData[0] = 9;
    MfComData.MfData[1] = addr;
    MfComData.MfData[2] = *writedata;
    MfComData.MfData[3] = *(writedata+1);
    status = PcdComTransceive(pi);
    return status;  
}      

//////////////////////////////////////////////////////////////////////
//取SR176卡块锁定状态
//////////////////////////////////////////////////////////////////////                            
char GetProtSR176(unsigned char lockreg)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSMIT;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = 8;
    MfComData.MfData[1] = 0x0f;
    status = PcdComTransceive(pi);
    if (status == MI_OK) { lockreg = MfComData.MfData[1];  }
    return status;  
}   	
//////////////////////////////////////////////////////////////////////
//SR176卡块锁定
//////////////////////////////////////////////////////////////////////
char ProtectSR176(unsigned char lockreg)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSMIT;
    MfComData.MfLength  = 4;
    MfComData.MfData[0] = 9;
    MfComData.MfData[1] = 0x0F;
    MfComData.MfData[2] = 0;
    MfComData.MfData[3] = lockreg;
    status = PcdComTransceive(pi);
    return status;  
}   

//////////////////////////////////////////////////////////////////////
//COMPLETION ST
//////////////////////////////////////////////////////////////////////
char CompletionSR()
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSMIT;
    MfComData.MfLength  = 1;
    MfComData.MfData[0] = 0x0F;
    status = PcdComTransceive(pi);
    return status;  
}                                          

//////////////////////////////////////////////////////////////////////
//SRIX4K卡读块
//////////////////////////////////////////////////////////////////////
char ReadSR4K(unsigned char addr,unsigned char *readdata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = 8;
    MfComData.MfData[1] = addr;
    status = PcdComTransceive(pi);
    if (status!=MI_OK) status = MI_COLLERR; // collision occurs
    if (MfComData.MfLength!=32) status = MI_COM_ERR;
    if (status == MI_OK)
    {
        *readdata     = MfComData.MfData[0];
        *(readdata+1) = MfComData.MfData[1];
        *(readdata+2) = MfComData.MfData[2];
        *(readdata+3) = MfComData.MfData[3];
    }
    return status;  
}

//////////////////////////////////////////////////////////////////////
//SR176卡写块
//////////////////////////////////////////////////////////////////////
char WriteSR4K(unsigned char addr,unsigned char *writedata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSMIT;
    MfComData.MfLength  = 6;
    MfComData.MfData[0] = 9;
    MfComData.MfData[1] = addr;
    MfComData.MfData[2] = *writedata;
    MfComData.MfData[3] = *(writedata+1);
    MfComData.MfData[4] = *(writedata+2);
    MfComData.MfData[5] = *(writedata+3);
    status = PcdComTransceive(pi);
    return status;  
} 

//////////////////////////////////////////////////////////////////////
//SR176卡鉴别
//////////////////////////////////////////////////////////////////////
char AuthSR4K(unsigned char *rnd,unsigned char *sig)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    PcdSetTmo(9);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 7;
    MfComData.MfData[0] = 0x0A;
    memcpy(&MfComData.MfData[1], rnd, 6);
    
    status = PcdComTransceive(pi);
    
    if (status!=MI_OK) status = MI_COLLERR; // collision occurs
    if (MfComData.MfLength!=24) status=MI_COM_ERR;
    if (status == MI_OK)
    {
        * sig    = MfComData.MfData[0];
        *(sig+1) = MfComData.MfData[1];
        *(sig+2) = MfComData.MfData[2];
    }
    return status;  
}  

//////////////////////////////////////////////////////////////////////
//SR176卡读UID
//////////////////////////////////////////////////////////////////////
char GetUIDSR4K(unsigned char *UID)
{
    char status;
    unsigned char i;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 1;
    MfComData.MfData[0] = 0x0B;
 
    status = PcdComTransceive(pi);

    if (MfComData.MfLength!=64) status = MI_COM_ERR;
    if (status == MI_OK)
    {
        for(i=0;i<8;i++)
        {  *(UID+7-i) = MfComData.MfData[i];  }
    }
    return status;  
}                                         



//////////////////////////////////////////////////////////////////////
//ISO14443 DESELECT
//////////////////////////////////////////////////////////////////////
char CL_Deselect(unsigned char CID)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(4);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = 0xca;
    MfComData.MfData[1] = CID;
    status = PcdComTransceive(pi);
    return status;
}


