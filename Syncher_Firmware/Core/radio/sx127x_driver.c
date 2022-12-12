#include "sx127x_driver.h"
#include "sx1276-LoRa.h"
#include "sx127x_hal.h"
#include "string.h"

/*
 * Copyright (c) 2019-2020 AIThinker.yangbin All rights reserved.
 *
 * ±¾¹¤³ÌÖ»ÊÇSX127XµÄÇı¶¯demo£¬½ö¹©²Î¿¼£¬²»±£Ö¤ÉÌÓÃÎÈ¶¨ĞÔ¡£
 *
 * author     Specter
 */

#define DEFAUTL_TIMEOUT 1000
static void Sx127xStartRx(uint32_t timeoutSystick);
static void SX127xSetTxPacket( const void *buffer, uint16_t size,uint32_t timeoutSystick);
static void Sx127xReadRxPackage( void *buffer, uint16_t *size );
static void Sx127xStartCADCheck(void);

tRadioDriver g_Radio ={sx127xInit,Sx127xRestart,Sx127xStartRx,Sx127xReadRxPackage,SX127xSetTxPacket,Sx127xStartCADCheck,SX127xProcess};

static void SX1278ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t size);
static void SX1278WriteBuffer(uint8_t addr,uint8_t *buffer,uint8_t size);
static uint8_t u8_SFList[]={RFLR_MODEMCONFIG2_SF_6,RFLR_MODEMCONFIG2_SF_7,RFLR_MODEMCONFIG2_SF_8,RFLR_MODEMCONFIG2_SF_9,RFLR_MODEMCONFIG2_SF_10,RFLR_MODEMCONFIG2_SF_11,RFLR_MODEMCONFIG2_SF_12}; 
static uint8_t u8_CRList[]={RFLR_MODEMCONFIG1_CODINGRATE_4_5,RFLR_MODEMCONFIG1_CODINGRATE_4_6,RFLR_MODEMCONFIG1_CODINGRATE_4_7,RFLR_MODEMCONFIG1_CODINGRATE_4_8};
static uint8_t u8_BWList[]={RFLR_MODEMCONFIG1_BW_7_81_KHZ,RFLR_MODEMCONFIG1_BW_10_41_KHZ,RFLR_MODEMCONFIG1_BW_15_62_KHZ,RFLR_MODEMCONFIG1_BW_20_83_KHZ,RFLR_MODEMCONFIG1_BW_31_25_KHZ,RFLR_MODEMCONFIG1_BW_41_66_KHZ,RFLR_MODEMCONFIG1_BW_62_50_KHZ,RFLR_MODEMCONFIG1_BW_125_KHZ,RFLR_MODEMCONFIG1_BW_250_KHZ,RFLR_MODEMCONFIG1_BW_500_KHZ};
//è½¯ä»¶è¶…æ—¶æ—¶é—´(è¿™ä¸ªä½¿ç”¨systickè®¡æ—¶çš„ï¼Œå•ä½æ˜¯1systick)
static uint32_t softTimeout=DEFAUTL_TIMEOUT;	
//æœ¬åœ°ä¿å­˜çš„é…ç½®ä¿¡æ¯ï¼Œç”¨äºåœ¨å¤ä½åé‡æ–°åˆå§‹åŒ–
static tLoRaSettings localSettingSave={435000000,20,7,7,1,0x000f};	
//å½“å‰å°„é¢‘çŠ¶æ€æœº(ä¸æ˜¯è¿”å›ç»™ç”¨æˆ·çš„tRFProcessReturnCodesï¼Œè¿™ä¸ªæ¯”ç”¨æˆ·ç»™ç”¨æˆ·çš„çŠ¶æ€å¤š)
volatile static tRFLRStates  loraStatus=RFLR_STATE_IDLE;	

/**
 * åˆå§‹åŒ–LoRaé…ç½®å‡½æ•°
 * å‚æ•°
 *     sttingï¼šé…ç½®ç»“æ„ä½“ï¼Œä¼ å…¥éœ€è¦è®¾ç½®çš„å‚æ•°ï¼Œeg{435000000,20,8,7,1}ï¼Œå…·ä½“å†…å®¹æŸ¥çœ‹ tLoRaSettings å®šä¹‰,å¦‚æœä¼ å…¥NULLï¼Œè¡¨ç¤ºåŠ è½½ä¸Šæ¬¡çš„è®¾ç½®(ä¸Šæ¬¡æ²¡æœ‰å°±åŠ è½½é»˜è®¤è®¾ç½®)
 */
void sx127xInit(tLoRaSettings *stting){
	SX1276HALInit();
	Sx127xRestart();
	while(0x6c!=Read127xReg(0x06)){
		DEBUG("[ERROR %s()-%d]spi error\r\n",__func__,__LINE__);
		Soft_delay_ms(100);
	}
	DEBUG("spi init ok\r\n");
	
	SX127xSetLoRaMode();
	
	if(NULL!=stting){
		memcpy(&localSettingSave,stting,sizeof(tLoRaSettings));	//å¤åˆ¶é…ç½®ä¿¡æ¯
	}
	stting=&localSettingSave;	//settingæŒ‡å‘å¤‡ä»½æ•°æ®ï¼Œé¿å…ä¿®æ”¹å¯¼è‡´settingåŸå€¼æ”¹å˜
	
	if(stting->SignalBw>9){
		DEBUG("[WARRING %s()-%d]setting error,auto fix\r\n",__func__,__LINE__);
		stting->SignalBw=9;
	}
	if(stting->ErrorCoding>4){
		DEBUG("[WARRING %s()-%d]setting error,auto fix\r\n",__func__,__LINE__);
		stting->ErrorCoding=4;
	}
	if(stting->ErrorCoding<1){
		DEBUG("[WARRING %s()-%d]setting error,auto fix\r\n",__func__,__LINE__);
		stting->ErrorCoding=1;
	}
	if(stting->SpreadingFactor>12){
		DEBUG("[WARRING %s()-%d]setting error,auto fix\r\n",__func__,__LINE__);
		stting->SpreadingFactor=12;
	}
	if(stting->SpreadingFactor<6){
		DEBUG("[WARRING %s()-%d]setting error,auto fix\r\n",__func__,__LINE__);
		stting->SpreadingFactor=6;
	}
	if(stting->Power>20){
		DEBUG("[WARRING %s()-%d]setting error,auto fix\r\n",__func__,__LINE__);
		stting->Power=20;
	}
	
	SX127xSetFrf(stting->RFFrequency);//è®¾ç½®é¢‘ç‡
	Write127xReg(REG_LR_MODEMCONFIG1,u8_BWList[stting->SignalBw]|u8_CRList[stting->ErrorCoding -1]|RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF);//ÉèÖÃ´ø¿í¡¢¾À´í±àÂëÂÊ
	Write127xReg(REG_LR_MODEMCONFIG2,u8_SFList[stting->SpreadingFactor-6] | RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF|RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON|0x03);//ÉèÖÃSD£¬CRC£¬³¬Ê±Ê±¼ä
	Write127xReg(REG_LR_SYMBTIMEOUTLSB,0xFF);//è®¾ç½®è¶…æ—¶æ—¶é—´
	Write127xReg(REG_LR_MODEMCONFIG3,0x0C);//è®¾ç½®ä½é€Ÿç‡(åŒ…é•¿è¶…è¿‡16mså¿…é¡»æ‰“å¼€)
	
	if(stting->Power>17){
		Write127xReg(REG_LR_PACONFIG,0x80+stting->Power-5);//è®¾ç½®åŠŸç‡
		Write127xReg(0x4d,0x87);//è®¾ç½®æ›´å¤§åŠŸç‡
	}else{
		Write127xReg(REG_LR_PACONFIG,0x80+stting->Power-2);//è®¾ç½®åŠŸç‡
		Write127xReg(0x4d,0x84);//å…³é—­æ›´å¤§åŠŸç‡
	}
	Write127xReg(REG_LR_OCP,0x3B);//è¿‡æµä¿æŠ¤
	//è®¾ç½®å‰å¯¼ç é•¿åº¦ REG_LR_PREAMBLEMSB
	Write127xReg(REG_LR_PREAMBLEMSB,stting->PreambleLength>>8);	//å‰å¯¼ç é«˜æœ‰æ•ˆä½
	Write127xReg(REG_LR_PREAMBLELSB,stting->PreambleLength&0x00ff);	//å‰å¯¼ç ä½æœ‰æ•ˆä½

}

void Sx127xRestart(void){
	SX127X_ResetPinControl(0);
	Soft_delay_ms(10);
	SX127X_ResetPinControl(1);
	Soft_delay_ms(10);
	loraStatus=RFLR_STATE_IDLE;
}

//ÉèÖÃÎªLoRaÄ£Ê½
void SX127xSetLoRaMode(void)
{
  if(0!=(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_LONGRANGEMODE_ON)){
		return;//µ±Ç°´¦ÓÚLoRaÄ£Ê½
	}
	SX127xSetOpMode(LORA_OPMODE_SLEEP);
	Write127xReg(REG_LR_OPMODE,Read127xReg(REG_LR_OPMODE)|RFLR_OPMODE_LONGRANGEMODE_ON);//ÉèÖÃÎªLoRaÄ£Ê½£¨Ö»ÓĞÔÚ LORA_OPMODE_SLEEP Ä£Ê½ÏÂ²ÅÄÜ²Ù×÷£©
}

//Ğ´sx1278¼Ä´æÆ÷
void Write127xReg(uint8_t addr,uint8_t data){
	SX1278WriteBuffer( addr,&data, 1 );
}

//¶Ásx1278¼Ä´æÆ÷
uint8_t Read127xReg(uint8_t addr){
	uint8_t u8_recive;

	SX1278ReadBuffer( addr, &u8_recive, 1 );

	return u8_recive;
}

//Ğ´sx1278 fifo
void SX127xWriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1278WriteBuffer( 0, buffer, size );
}

//¶Ásx1278 fifo
void SX127xReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1278ReadBuffer( 0, buffer, size );
}

//è®¾ç½®OpMode
void SX127xSetOpMode(LoRaOpModeType opMode)
{
	if(opMode==SX127xGetOpMode()){
		return;//µ±Ç°Ä£Ê½ÕıÈ·£¬²»ÓÃÇĞ»»
	}
  Write127xReg(REG_LR_OPMODE,(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_MASK)|opMode|RFLR_OPMODE_FREQMODE_ACCESS_LF );
	Soft_delay_ms(1);
}

//è·å–OpMode
LoRaOpModeType SX127xGetOpMode(void)
{
    return (LoRaOpModeType)(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_MASK);
}

static void SX1278ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t size)
{
    uint8_t i;

    //NSS = 0;
		SpiNSSEnable(0);	//ç‰‡é€‰spi1

    SpiInOut(addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut(0x00); //è¯»å–æ•°æ®
    }

    //NSS = 1;
		SpiNSSEnable(1);
}

static void SX1278WriteBuffer(uint8_t addr,uint8_t *buffer,uint8_t size)
{
    uint8_t i;

    //NSS = 0;
		SpiNSSEnable(0);

    SpiInOut(addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
			SpiInOut(buffer[i]);//å†™å…¥æ•°æ®
    }

    //NSS = 1;
		SpiNSSEnable(1);
}

//è®¾ç½®è½½æ³¢é¢‘ç‡
void SX127xSetFrf(uint32_t fr)
{
	uint8_t frfBuf[4];
	
	fr=fr*0.016384;//æ ¹æ®æ•°æ®æ‰‹å†Œè®¡ç®—å¯„å­˜å™¨è¦è®¾ç½®çš„å€¼
	memcpy(frfBuf,&fr,4);
	SX127xSetOpMode(LORA_OPMODE_SLEEP);

	Write127xReg(REG_LR_FRFMSB,frfBuf[2]);
	Write127xReg(REG_LR_FRFMID,frfBuf[1]);
	Write127xReg(REG_LR_FRFLSB,frfBuf[0]);
}


/**
 * å‘é€æ•°æ®ï¼ˆé˜»å¡å‘é€ï¼‰
 * å‚æ•°ï¼š
 *     dataï¼šè¦å‘é€çš„æ•°æ®
 *     lenï¼šdataçš„é•¿åº¦
 *     timeoutMsï¼šè¶…æ—¶æ—¶é—´ï¼ˆè¿™ä¸ªæ—¶é—´ä½¿ç”¨systickï¼‰
 * è¿”å›å€¼ï¼š
 *     0æˆåŠŸ
 *     1è¶…æ—¶
 */
uint8_t sx127xSend(uint8_t *data,uint8_t len,uint32_t timeoutMs){
	uint32_t systickBak=GET_TICK_COUNT(),currTick;
	
	Write127xReg( REG_LR_PAYLOADLENGTH, len );	//è®¾ç½®è´Ÿè½½é•¿åº¦

	Write127xReg( REG_LR_FIFOTXBASEADDR, 0 );//å°†å‘é€bufçš„åŸºåœ°å€æŒ‡å‘0x00ï¼Œæ­¤æ—¶æ•´ä¸ªfifoéƒ½å¯ä»¥ç”¨æ¥å‘é€æ•°æ®
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//å°†fifiè¯»å†™æŒ‡é’ˆæ‰§è¡Œ0x00ï¼Œæ­¤æ—¶å‘å¯„å­˜å™¨0x00è¯»å†™æ•°æ®æŒ‡é’ˆä¼šè‡ªå¢çš„å°†æ•°æ®å†™å…¥fifo
	SX127xSetOpMode(LORA_OPMODE_STANDBY);//LORA_OPMODE_SLEEP æ¨¡å¼ä¸èƒ½è¯»å†™fifo
	
	SX127xWriteFifo(data,len);	//å°†è¦å‘é€çš„æ•°æ®å†™å…¥fifo
	//å¼€å¯ä¸­æ–­å±è”½ä½(åªç•™ä¸‹äº† RFLR_IRQFLAGS_TXDONE ä¸­æ–­æ²¡æœ‰å±è”½æ‰)ï¼Œè®¾ç½®ä¸ºå‘é€æ¨¡å¼åä»»æ„è¯»å†™ä¸€ä¸ªå¯„å­˜å™¨å°±ä¼šè§¦å‘å‘é€ï¼Œè¿™é‡Œå†™å…¥è¿™ä¸ªå¼€ä¸­æ–­æ­£å¥½è§¦å‘
	Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
	Write127xReg( REG_LR_DIOMAPPING1, ( Read127xReg( REG_LR_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );//DIO0è®¾ç½®ä¸ºTXdoneä¸­æ–­
	//DEBUG("DIO0=%d\r\n0x06:%02x\r\n",SX1276ReadDio0(),Read127xReg(0x06));
	SX127xSetOpMode(LORA_OPMODE_TRANSMITTER);	//è®¾ç½®ä¸ºå‘é€æ¨¡å¼
	Read127xReg(REG_LR_IRQFLAGS);//è®¾ç½®å‘é€åè¯»å†™ä»»æ„å¯„å­˜å™¨å¯ä»¥å¼€å§‹å‘é€
	
	while(1){
		if(1==SX1276ReadDio0()){
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//æ¸…é™¤ä¸­æ–­æ ‡å¿—ä½
			return 0;
		}
		currTick=GET_TICK_COUNT();
		if(currTick>=systickBak){
			if(currTick-systickBak>timeoutMs){
				return 1;
			}
		}else{
			//currTickæº¢å‡ºäº†
			if(currTick+(~systickBak)>timeoutMs){
				return 1;
			}
		}
	}
}

/**
 * æ¥æ”¶æ•°æ®ï¼ˆé˜»å¡æ¥æ”¶ï¼‰
 * å‚æ•°
 *     bufï¼šæ¥æ”¶æ•°æ®çš„bufç©ºé—´
 *     lenï¼šä¼ å…¥bufçš„å¤§å°ï¼Œè¿”å›åä¼šä¿®æ”¹ä¸ºæ¥æ”¶åˆ°æ•°æ®çš„é•¿åº¦ï¼ˆæ¥æ”¶å¤±è´¥ä¸å˜ï¼‰
 *     timeoutMSï¼šè½¯ä»¶è¶…æ—¶æ—¶é—´
 * è¿”å›å€¼
 *     0ï¼šæ¥æ”¶æˆåŠŸ
 *     1ï¼šDIO1è¶…æ—¶ï¼ˆDIO1è¶…æ—¶æ˜¯ç¡¬ä»¶è¿”å›çš„ï¼Œå’ŒtimeoutMSæ— å…³ï¼‰
 *     2ï¼šè½¯ä»¶è¶…æ—¶
 */
uint8_t sx127xRx(uint8_t *buf,uint8_t *len,uint32_t timeoutMs){
	uint32_t systickBak=GET_TICK_COUNT(),currTick;
	uint8_t u8_reciveLength;
	
	SX127xSetOpMode(LORA_OPMODE_STANDBY);
	Write127xReg( REG_LR_FIFORXBASEADDR, 0 );//å°†æ¥æ”¶bufçš„åŸºåœ°å€æŒ‡å‘0x00ï¼Œæ­¤æ—¶æ•´ä¸ªfifoéƒ½å¯ä»¥ç”¨æ¥æ¥æ”¶æ•°æ®
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//å°†fifiè¯»å†™æŒ‡é’ˆæ‰§è¡Œ0x00ï¼Œæ­¤æ—¶å‘å¯„å­˜å™¨0x00è¯»å†™æ•°æ®æŒ‡é’ˆä¼šè‡ªå¢çš„ä»fifoä¸­è¯»å–æ•°æ®
	Write127xReg( REG_LR_SYMBTIMEOUTLSB, 0xFF );//é…ç½® 0x1f rxè¶…æ—¶
	
	//é…ç½®ä¸­æ–­
	Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
	// DIO0=RxDone 0x00, DIO2=RxTimeout 0x00
  Write127xReg( REG_LR_DIOMAPPING1, ( Read127xReg( REG_LR_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00);
	SX127xSetOpMode(LORA_OPMODE_RECEIVER);//è¿ç»­æ¥æ”¶
	//SX127xSetOpMode( LORA_OPMODE_RECIVER_SINGLE );//å•æ¬¡æ¥æ”¶
	Read127xReg(REG_LR_IRQFLAGS);//è®¾ç½®æ¥æ”¶åè¯»å†™ä»»æ„å¯„å­˜å™¨å¯ä»¥å¼€å§‹æ¥æ”¶
	
	while(1){
		if(1==SX1276ReadDio0()){
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE);//æ¸…é™¤ä¸­æ–­æ ‡å¿—ä½
			u8_reciveLength=Read127xReg(REG_LR_NBRXBYTES);
			if(u8_reciveLength > *len){
				u8_reciveLength=*len;
			}else{
				*len=u8_reciveLength;
			}
			SX127xReadFifo(buf,u8_reciveLength);
			return 0;
		}
		if(1==SX1276ReadDio1()){
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXTIMEOUT);//æ¸…é™¤ä¸­æ–­æ ‡å¿—ä½
			return 1;
		}
		currTick=GET_TICK_COUNT();
		if(currTick>=systickBak){
			if(currTick-systickBak>timeoutMs){
				return 1;
			}
		}else{
			//currTickæº¢å‡ºäº†
			if(currTick+(~systickBak)>timeoutMs){
				return 1;
			}
		}
	}
}

//·Ç×èÈû×´Ì¬ÂÖÑ¯º¯Êı
tRFProcessReturnCodes SX127xProcess( void )
{
	uint32_t currTick=0;
	static uint32_t systickBak=0;
	
	switch(loraStatus){
		case RFLR_STATE_IDLE:
			return RF_IDLE;
		case RFLR_STATE_RX_INIT:
			SX127xSetOpMode(LORA_OPMODE_STANDBY);
			Write127xReg( REG_LR_FIFORXBASEADDR, 0 );//½«½ÓÊÕbufµÄ»ùµØÖ·Ö¸Ïò0x00£¬´ËÊ±Õû¸öfifo¶¼¿ÉÒÔÓÃÀ´½ÓÊÕÊı¾İ
			Write127xReg( REG_LR_FIFOADDRPTR, 0 );//½«fifi¶ÁĞ´Ö¸ÕëÖ´ĞĞ0x00£¬´ËÊ±Ïò¼Ä´æÆ÷0x00¶ÁĞ´Êı¾İÖ¸Õë»á×ÔÔöµÄ´ÓfifoÖĞ¶ÁÈ¡Êı¾İ
			Write127xReg( REG_LR_SYMBTIMEOUTLSB, 0xFF );//ÅäÖÃ 0x1f rx³¬Ê±
			
			//ÅäÖÃÖĞ¶Ï
			Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
																											//RFLR_IRQFLAGS_RXDONE |
																											RFLR_IRQFLAGS_PAYLOADCRCERROR |
																											RFLR_IRQFLAGS_VALIDHEADER |
																											RFLR_IRQFLAGS_TXDONE |
																											RFLR_IRQFLAGS_CADDONE |
																											RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
																											RFLR_IRQFLAGS_CADDETECTED );
			// DIO0=RxDone 0x00, DIO2=RxTimeout 0x00
			Write127xReg( REG_LR_DIOMAPPING1, ( Read127xReg( REG_LR_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00);
			SX127xSetOpMode(LORA_OPMODE_RECEIVER);//Á¬Ğø½ÓÊÕ
			//SX127xSetOpMode( LORA_OPMODE_RECIVER_SINGLE );//µ¥´Î½ÓÊÕ
			Read127xReg(REG_LR_IRQFLAGS);//ÉèÖÃ½ÓÊÕºó¶ÁĞ´ÈÎÒâ¼Ä´æÆ÷¿ÉÒÔ¿ªÊ¼½ÓÊÕ
			systickBak=GET_TICK_COUNT();
			loraStatus=RFLR_STATE_RX_RUNNING;
			return RF_BUSY;
    case RFLR_STATE_RX_RUNNING:
			if(1==SX1276ReadDio0()){
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE);//Çå³ıÖĞ¶Ï±êÖ¾Î»
				loraStatus=RFLR_STATE_RX_DONE;
				return RF_BUSY;
			}
			if(1==SX1276ReadDio1()){
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXTIMEOUT);//Çå³ıÖĞ¶Ï±êÖ¾Î»
				loraStatus=RFLR_STATE_RX_TIMEOUT;
				return RF_BUSY;
			}
			currTick=GET_TICK_COUNT();
			if(currTick>=systickBak){
				if(currTick-systickBak>softTimeout){
					loraStatus=RFLR_STATE_RX_TIMEOUT;
					return RF_BUSY;
				}
			}else{
				//currTickÒç³öÁË
				if(currTick+(~systickBak)>softTimeout){
					loraStatus=RFLR_STATE_RX_TIMEOUT;
					return RF_BUSY;
				}
			}
			return RF_BUSY;
    case RFLR_STATE_RX_DONE:
			return RF_RX_DONE;
    case RFLR_STATE_RX_TIMEOUT:
			return RF_RX_TIMEOUT;
    case RFLR_STATE_TX_INIT:
			systickBak=GET_TICK_COUNT();
			return RF_BUSY;
    case RFLR_STATE_TX_RUNNING:
			if(1==SX1276ReadDio0()){
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//Çå³ıÖĞ¶Ï±êÖ¾Î»
				loraStatus=RFLR_STATE_TX_DONE;
				return RF_BUSY;
			}
			if(1==SX1276ReadDio0()){
				//½ÓÊÕÍê³É
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//Çå³ıÖĞ¶Ï±êÖ¾Î»
				loraStatus=RFLR_STATE_TX_DONE;
				return RF_BUSY;
			}
			currTick=GET_TICK_COUNT();
			if(currTick>=systickBak){
				if(currTick-systickBak>softTimeout){
					loraStatus=RFLR_STATE_TX_TIMEOUT;
				}
			}else{
				//currTickÒç³öÁË
				if(currTick+(~systickBak)>softTimeout){
					loraStatus=RFLR_STATE_TX_TIMEOUT;
				}
			}
			return RF_BUSY;
    case RFLR_STATE_TX_DONE:
			return RF_TX_DONE;
    case RFLR_STATE_TX_TIMEOUT:
			return RF_TX_TIMEOUT;
    case RFLR_STATE_CAD_INIT:
			SX127xSetOpMode(LORA_OPMODE_STANDBY);
			//ÅäÖÃÖĞ¶Ï
			Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
																											RFLR_IRQFLAGS_RXDONE |
																											RFLR_IRQFLAGS_PAYLOADCRCERROR |
																											RFLR_IRQFLAGS_VALIDHEADER |
																											RFLR_IRQFLAGS_TXDONE |
																											//RFLR_IRQFLAGS_CADDONE |
																											RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL //|
																											//RFLR_IRQFLAGS_CADDETECTED 
																											);
		                                   // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
      Write127xReg( REG_LR_DIOMAPPING1,RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00);
			SX127xSetOpMode(LORA_OPMODE_CAD);//¿ªÆôCAD¼ì²â
			//Read127xReg(REG_LR_IRQFLAGS);//ÉèÖÃ½ÓÊÕºó¶ÁĞ´ÈÎÒâ¼Ä´æÆ÷¿ÉÒÔ¿ªÊ¼Ö´ĞĞÃüÁî
			systickBak=GET_TICK_COUNT();
			loraStatus=RFLR_STATE_CAD_RUNNING;
			return RF_BUSY;
    case RFLR_STATE_CAD_RUNNING:
			if( 1 == SX1276ReadDio3() ){ 
				// Clear Irq
        Write127xReg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
				
        if( 1==SX1276ReadDio4() ){ // CAD Detected interrupt
					// Clear Irq
          Write127xReg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
          // CAD detected, we have a LoRa preamble
          loraStatus=RFLR_STATE_CAD_DETECTED;
					return RF_BUSY;
        }else{    
					// The device goes in Standby Mode automatically    
          loraStatus=RFLR_STATE_CAD_EMPTY;
					return RF_BUSY;
        }
      }	//end of if( 1 == SX1276ReadDio3() ){
			currTick=GET_TICK_COUNT();
			if(currTick>=systickBak){
				if(currTick-systickBak>100){
					loraStatus=RFLR_STATE_CAD_TIMEOUT;
				}
			}else{
				//currTickÒç³öÁË
				if(currTick+(~systickBak)>100){
					loraStatus=RFLR_STATE_CAD_TIMEOUT;
				}
			}
			return RF_BUSY;	//RF_CAD_DETECTED
		case RFLR_STATE_CAD_DETECTED:
			return RF_CAD_DETECTED;
		case RFLR_STATE_CAD_EMPTY:
			return RF_CAD_EMPTY;
		case RFLR_STATE_CAD_TIMEOUT:
			return RF_CAD_TIMEOUT;
		default:
			return RF_UNKNOW_STATUS;
	}
	
	//return RF_UNKNOW_STATUS;
}

//·Ç×èÈû·¢ËÍÊı¾İ(ĞèÒªÅäºÏ ÂÖÑ¯ SX127xProcess() Ê¹ÓÃ)
//buffer:Òª·¢ËÍµÄÊı¾İ
//size£º·¢ËÍ³¤¶È
//timeoutSystick:Èí¼ş³¬Ê±Ê±¼ä(ÓÃsystick¼ÆÊ±£¬µ¥Î»ÊÇ1systick)£¬0±íÊ¾Ê¹ÓÃÄ¬ÈÏÖµ
static void SX127xSetTxPacket( const void *buffer, uint16_t size ,uint32_t timeoutSystick){
	loraStatus=RFLR_STATE_TX_INIT;
	if(size>255){
		size=255;
	}
	if(timeoutSystick>0){
		softTimeout=timeoutSystick;
	}else{
		softTimeout=DEFAUTL_TIMEOUT;
	}
	
	Write127xReg( REG_LR_PAYLOADLENGTH, size );	//ÉèÖÃ¸ºÔØ³¤¶È

	Write127xReg( REG_LR_FIFOTXBASEADDR, 0 );//½«·¢ËÍbufµÄ»ùµØÖ·Ö¸Ïò0x00£¬´ËÊ±Õû¸öfifo¶¼¿ÉÒÔÓÃÀ´·¢ËÍÊı¾İ
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//½«fifi¶ÁĞ´Ö¸ÕëÖ´ĞĞ0x00£¬´ËÊ±Ïò¼Ä´æÆ÷0x00¶ÁĞ´Êı¾İÖ¸Õë»á×ÔÔöµÄ½«Êı¾İĞ´Èëfifo
	SX127xSetOpMode(LORA_OPMODE_STANDBY);//LORA_OPMODE_SLEEP Ä£Ê½²»ÄÜ¶ÁĞ´fifo
	
	SX127xWriteFifo((uint8_t *)buffer,size);	//½«Òª·¢ËÍµÄÊı¾İĞ´Èëfifo
	//¿ªÆôÖĞ¶ÏÆÁ±ÎÎ»(Ö»ÁôÏÂÁË RFLR_IRQFLAGS_TXDONE ÖĞ¶ÏÃ»ÓĞÆÁ±Îµô)
	Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
	Write127xReg( REG_LR_DIOMAPPING1, ( Read127xReg( REG_LR_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );//DIO0ÉèÖÃÎªTXdoneÖĞ¶Ï
	//DEBUG("DIO0=%d\r\n0x06:%02x\r\n",SX1276ReadDio0(),Read127xReg(0x06));
	SX127xProcess();	//µ÷ÓÃÕâ¸öÊÇÎªÁË¸üĞÂsystick
	SX127xSetOpMode(LORA_OPMODE_TRANSMITTER);	//ÉèÖÃÎª·¢ËÍÄ£Ê½
	Read127xReg(REG_LR_IRQFLAGS);//ÉèÖÃ·¢ËÍºó¶ÁĞ´ÈÎÒâ¼Ä´æÆ÷¿ÉÒÔ¿ªÊ¼·¢ËÍ
	loraStatus=RFLR_STATE_TX_RUNNING;
}

//·Ç×èÈû½ÓÊÕº¯Êı(ĞèÒªÅäºÏ ÂÖÑ¯ SX127xProcess() Ê¹ÓÃ)
//timeoutSystick:Èí¼ş³¬Ê±Ê±¼ä(ÓÃsystick¼ÆÊ±£¬µ¥Î»ÊÇ1systick)£¬0±íÊ¾Ê¹ÓÃÄ¬ÈÏÖµ
static void Sx127xStartRx(uint32_t timeoutSystick){
	if(timeoutSystick>0){
		softTimeout=timeoutSystick;
	}else{
		softTimeout=DEFAUTL_TIMEOUT;
	}
	loraStatus=RFLR_STATE_RX_INIT;
}

//¶ÁÈ¡fifoÖĞ½ÓÊÕµ½µÄÊı¾İ
static void Sx127xReadRxPackage( void *buffer, uint16_t *size ){
	//¶ÁÈ¡Êı¾İ
	uint16_t tmpReciveLength;
	
	tmpReciveLength=Read127xReg(REG_LR_NBRXBYTES);
	if(tmpReciveLength > *size){
		tmpReciveLength=*size;
	}else{
		*size=tmpReciveLength;
	}
	SX127xReadFifo(buffer,tmpReciveLength);
}

//·Ç×èÈû ¿ªÆôÒ»´ÎCAD¼ì²â(ĞèÒªÅäºÏ ÂÖÑ¯ SX127xProcess() Ê¹ÓÃ)
static void Sx127xStartCADCheck(void){
	loraStatus=RFLR_STATE_CAD_INIT;
}
