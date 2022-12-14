#include "sx127x_driver.h"
#include "sx1276-LoRa.h"
#include "sx127x_hal.h"
#include "string.h"

/*
 * Copyright (c) 2019-2020 AIThinker.yangbin All rights reserved.
 *
 * 本工程只是SX127X的驱动demo，仅供参考，不保证商用稳定性。
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
//杞欢瓒呮椂鏃堕棿(杩欎釜浣跨敤systick璁℃椂鐨勶紝鍗曚綅鏄?1systick)
static uint32_t softTimeout=DEFAUTL_TIMEOUT;	
//鏈湴淇濆瓨鐨勯厤缃俊鎭紝鐢ㄤ簬鍦ㄥ浣嶅悗閲嶆柊鍒濆鍖?
static tLoRaSettings localSettingSave={435000000,20,7,7,1,0x000f};	
//褰撳墠灏勯鐘舵?佹満(涓嶆槸杩斿洖缁欑敤鎴风殑tRFProcessReturnCodes锛岃繖涓瘮鐢ㄦ埛缁欑敤鎴风殑鐘舵?佸)
volatile static tRFLRStates  loraStatus=RFLR_STATE_IDLE;	

/**
 * 鍒濆鍖朙oRa閰嶇疆鍑芥暟
 * 鍙傛暟
 *     stting锛氶厤缃粨鏋勪綋锛屼紶鍏ラ渶瑕佽缃殑鍙傛暟锛宔g{435000000,20,8,7,1}锛屽叿浣撳唴瀹规煡鐪? tLoRaSettings 瀹氫箟,濡傛灉浼犲叆NULL锛岃〃绀哄姞杞戒笂娆＄殑璁剧疆(涓婃娌℃湁灏卞姞杞介粯璁よ缃?)
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
		memcpy(&localSettingSave,stting,sizeof(tLoRaSettings));	//澶嶅埗閰嶇疆淇℃伅
	}
	stting=&localSettingSave;	//setting鎸囧悜澶囦唤鏁版嵁锛岄伩鍏嶄慨鏀瑰鑷磗etting鍘熷?兼敼鍙?
	
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
	
	SX127xSetFrf(stting->RFFrequency);//璁剧疆棰戠巼
	Write127xReg(REG_LR_MODEMCONFIG1,u8_BWList[stting->SignalBw]|u8_CRList[stting->ErrorCoding -1]|RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF);//设置带宽、纠错编码率
	Write127xReg(REG_LR_MODEMCONFIG2,u8_SFList[stting->SpreadingFactor-6] | RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF|RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON|0x03);//设置SD，CRC，超时时间
	Write127xReg(REG_LR_SYMBTIMEOUTLSB,0xFF);//璁剧疆瓒呮椂鏃堕棿
	Write127xReg(REG_LR_MODEMCONFIG3,0x0C);//璁剧疆浣庨?熺巼(鍖呴暱瓒呰繃16ms蹇呴』鎵撳紑)
	
	if(stting->Power>17){
		Write127xReg(REG_LR_PACONFIG,0x80+stting->Power-5);//璁剧疆鍔熺巼
		Write127xReg(0x4d,0x87);//璁剧疆鏇村ぇ鍔熺巼
	}else{
		Write127xReg(REG_LR_PACONFIG,0x80+stting->Power-2);//璁剧疆鍔熺巼
		Write127xReg(0x4d,0x84);//鍏抽棴鏇村ぇ鍔熺巼
	}
	Write127xReg(REG_LR_OCP,0x3B);//杩囨祦淇濇姢
	//璁剧疆鍓嶅鐮侀暱搴? REG_LR_PREAMBLEMSB
	Write127xReg(REG_LR_PREAMBLEMSB,stting->PreambleLength>>8);	//鍓嶅鐮侀珮鏈夋晥浣?
	Write127xReg(REG_LR_PREAMBLELSB,stting->PreambleLength&0x00ff);	//鍓嶅鐮佷綆鏈夋晥浣?

}

void Sx127xRestart(void){
	SX127X_ResetPinControl(0);
	Soft_delay_ms(10);
	SX127X_ResetPinControl(1);
	Soft_delay_ms(10);
	loraStatus=RFLR_STATE_IDLE;
}

//设置为LoRa模式
void SX127xSetLoRaMode(void)
{
  if(0!=(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_LONGRANGEMODE_ON)){
		return;//当前处于LoRa模式
	}
	SX127xSetOpMode(LORA_OPMODE_SLEEP);
	Write127xReg(REG_LR_OPMODE,Read127xReg(REG_LR_OPMODE)|RFLR_OPMODE_LONGRANGEMODE_ON);//设置为LoRa模式（只有在 LORA_OPMODE_SLEEP 模式下才能操作）
}

//写sx1278寄存器
void Write127xReg(uint8_t addr,uint8_t data){
	SX1278WriteBuffer( addr,&data, 1 );
}

//读sx1278寄存器
uint8_t Read127xReg(uint8_t addr){
	uint8_t u8_recive;

	SX1278ReadBuffer( addr, &u8_recive, 1 );

	return u8_recive;
}

//写sx1278 fifo
void SX127xWriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1278WriteBuffer( 0, buffer, size );
}

//读sx1278 fifo
void SX127xReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1278ReadBuffer( 0, buffer, size );
}

//璁剧疆OpMode
void SX127xSetOpMode(LoRaOpModeType opMode)
{
	if(opMode==SX127xGetOpMode()){
		return;//当前模式正确，不用切换
	}
  Write127xReg(REG_LR_OPMODE,(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_MASK)|opMode|RFLR_OPMODE_FREQMODE_ACCESS_LF );
	Soft_delay_ms(1);
}

//鑾峰彇OpMode
LoRaOpModeType SX127xGetOpMode(void)
{
    return (LoRaOpModeType)(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_MASK);
}

static void SX1278ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t size)
{
    uint8_t i;

    //NSS = 0;
		SpiNSSEnable(0);	//鐗囬?塻pi1

    SpiInOut(addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut(0x00); //璇诲彇鏁版嵁
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
			SpiInOut(buffer[i]);//鍐欏叆鏁版嵁
    }

    //NSS = 1;
		SpiNSSEnable(1);
}

//璁剧疆杞芥尝棰戠巼
void SX127xSetFrf(uint32_t fr)
{
	uint8_t frfBuf[4];
	
	fr=fr*0.016384;//鏍规嵁鏁版嵁鎵嬪唽璁＄畻瀵勫瓨鍣ㄨ璁剧疆鐨勫??
	memcpy(frfBuf,&fr,4);
	SX127xSetOpMode(LORA_OPMODE_SLEEP);

	Write127xReg(REG_LR_FRFMSB,frfBuf[2]);
	Write127xReg(REG_LR_FRFMID,frfBuf[1]);
	Write127xReg(REG_LR_FRFLSB,frfBuf[0]);
}


/**
 * 鍙戦?佹暟鎹紙闃诲鍙戦?侊級
 * 鍙傛暟锛?
 *     data锛氳鍙戦?佺殑鏁版嵁
 *     len锛歞ata鐨勯暱搴?
 *     timeoutMs锛氳秴鏃舵椂闂达紙杩欎釜鏃堕棿浣跨敤systick锛?
 * 杩斿洖鍊硷細
 *     0鎴愬姛
 *     1瓒呮椂
 */
uint8_t sx127xSend(uint8_t *data,uint8_t len,uint32_t timeoutMs){
	uint32_t systickBak=GET_TICK_COUNT(),currTick;
	
	Write127xReg( REG_LR_PAYLOADLENGTH, len );	//璁剧疆璐熻浇闀垮害

	Write127xReg( REG_LR_FIFOTXBASEADDR, 0 );//灏嗗彂閫乥uf鐨勫熀鍦板潃鎸囧悜0x00锛屾鏃舵暣涓猣ifo閮藉彲浠ョ敤鏉ュ彂閫佹暟鎹?
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//灏唂ifi璇诲啓鎸囬拡鎵ц0x00锛屾鏃跺悜瀵勫瓨鍣?0x00璇诲啓鏁版嵁鎸囬拡浼氳嚜澧炵殑灏嗘暟鎹啓鍏ifo
	SX127xSetOpMode(LORA_OPMODE_STANDBY);//LORA_OPMODE_SLEEP 妯″紡涓嶈兘璇诲啓fifo
	
	SX127xWriteFifo(data,len);	//灏嗚鍙戦?佺殑鏁版嵁鍐欏叆fifo
	//寮?鍚腑鏂睆钄戒綅(鍙暀涓嬩簡 RFLR_IRQFLAGS_TXDONE 涓柇娌℃湁灞忚斀鎺?)锛岃缃负鍙戦?佹ā寮忓悗浠绘剰璇诲啓涓?涓瘎瀛樺櫒灏变細瑙﹀彂鍙戦?侊紝杩欓噷鍐欏叆杩欎釜寮?涓柇姝ｅソ瑙﹀彂
	Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
	Write127xReg( REG_LR_DIOMAPPING1, ( Read127xReg( REG_LR_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );//DIO0璁剧疆涓篢Xdone涓柇
	//DEBUG("DIO0=%d\r\n0x06:%02x\r\n",SX1276ReadDio0(),Read127xReg(0x06));
	SX127xSetOpMode(LORA_OPMODE_TRANSMITTER);	//璁剧疆涓哄彂閫佹ā寮?
	Read127xReg(REG_LR_IRQFLAGS);//璁剧疆鍙戦?佸悗璇诲啓浠绘剰瀵勫瓨鍣ㄥ彲浠ュ紑濮嬪彂閫?
	
	while(1){
		if(1==SX1276ReadDio0()){
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//娓呴櫎涓柇鏍囧織浣?
			return 0;
		}
		currTick=GET_TICK_COUNT();
		if(currTick>=systickBak){
			if(currTick-systickBak>timeoutMs){
				return 1;
			}
		}else{
			//currTick婧㈠嚭浜?
			if(currTick+(~systickBak)>timeoutMs){
				return 1;
			}
		}
	}
}

/**
 * 鎺ユ敹鏁版嵁锛堥樆濉炴帴鏀讹級
 * 鍙傛暟
 *     buf锛氭帴鏀舵暟鎹殑buf绌洪棿
 *     len锛氫紶鍏uf鐨勫ぇ灏忥紝杩斿洖鍚庝細淇敼涓烘帴鏀跺埌鏁版嵁鐨勯暱搴︼紙鎺ユ敹澶辫触涓嶅彉锛?
 *     timeoutMS锛氳蒋浠惰秴鏃舵椂闂?
 * 杩斿洖鍊?
 *     0锛氭帴鏀舵垚鍔?
 *     1锛欴IO1瓒呮椂锛圖IO1瓒呮椂鏄‖浠惰繑鍥炵殑锛屽拰timeoutMS鏃犲叧锛?
 *     2锛氳蒋浠惰秴鏃?
 */
uint8_t sx127xRx(uint8_t *buf,uint8_t *len,uint32_t timeoutMs){
	uint32_t systickBak=GET_TICK_COUNT(),currTick;
	uint8_t u8_reciveLength;
	
	SX127xSetOpMode(LORA_OPMODE_STANDBY);
	Write127xReg( REG_LR_FIFORXBASEADDR, 0 );//灏嗘帴鏀禸uf鐨勫熀鍦板潃鎸囧悜0x00锛屾鏃舵暣涓猣ifo閮藉彲浠ョ敤鏉ユ帴鏀舵暟鎹?
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//灏唂ifi璇诲啓鎸囬拡鎵ц0x00锛屾鏃跺悜瀵勫瓨鍣?0x00璇诲啓鏁版嵁鎸囬拡浼氳嚜澧炵殑浠巉ifo涓鍙栨暟鎹?
	Write127xReg( REG_LR_SYMBTIMEOUTLSB, 0xFF );//閰嶇疆 0x1f rx瓒呮椂
	
	//閰嶇疆涓柇
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
	SX127xSetOpMode(LORA_OPMODE_RECEIVER);//杩炵画鎺ユ敹
	//SX127xSetOpMode( LORA_OPMODE_RECIVER_SINGLE );//鍗曟鎺ユ敹
	Read127xReg(REG_LR_IRQFLAGS);//璁剧疆鎺ユ敹鍚庤鍐欎换鎰忓瘎瀛樺櫒鍙互寮?濮嬫帴鏀?
	
	while(1){
		if(1==SX1276ReadDio0()){
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE);//娓呴櫎涓柇鏍囧織浣?
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
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXTIMEOUT);//娓呴櫎涓柇鏍囧織浣?
			return 1;
		}
		currTick=GET_TICK_COUNT();
		if(currTick>=systickBak){
			if(currTick-systickBak>timeoutMs){
				return 1;
			}
		}else{
			//currTick婧㈠嚭浜?
			if(currTick+(~systickBak)>timeoutMs){
				return 1;
			}
		}
	}
}

//非阻塞状态轮询函数
tRFProcessReturnCodes SX127xProcess( void )
{
	uint32_t currTick=0;
	static uint32_t systickBak=0;
	
	switch(loraStatus){
		case RFLR_STATE_IDLE:
			return RF_IDLE;
		case RFLR_STATE_RX_INIT:
			SX127xSetOpMode(LORA_OPMODE_STANDBY);
			Write127xReg( REG_LR_FIFORXBASEADDR, 0 );//将接收buf的基地址指向0x00，此时整个fifo都可以用来接收数据
			Write127xReg( REG_LR_FIFOADDRPTR, 0 );//将fifi读写指针执行0x00，此时向寄存器0x00读写数据指针会自增的从fifo中读取数据
			Write127xReg( REG_LR_SYMBTIMEOUTLSB, 0xFF );//配置 0x1f rx超时
			
			//配置中断
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
			SX127xSetOpMode(LORA_OPMODE_RECEIVER);//连续接收
			//SX127xSetOpMode( LORA_OPMODE_RECIVER_SINGLE );//单次接收
			Read127xReg(REG_LR_IRQFLAGS);//设置接收后读写任意寄存器可以开始接收
			systickBak=GET_TICK_COUNT();
			loraStatus=RFLR_STATE_RX_RUNNING;
			return RF_BUSY;
    case RFLR_STATE_RX_RUNNING:
			if(1==SX1276ReadDio0()){
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE);//清除中断标志位
				loraStatus=RFLR_STATE_RX_DONE;
				return RF_BUSY;
			}
			if(1==SX1276ReadDio1()){
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXTIMEOUT);//清除中断标志位
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
				//currTick溢出了
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
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//清除中断标志位
				loraStatus=RFLR_STATE_TX_DONE;
				return RF_BUSY;
			}
			if(1==SX1276ReadDio0()){
				//接收完成
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//清除中断标志位
				loraStatus=RFLR_STATE_TX_DONE;
				return RF_BUSY;
			}
			currTick=GET_TICK_COUNT();
			if(currTick>=systickBak){
				if(currTick-systickBak>softTimeout){
					loraStatus=RFLR_STATE_TX_TIMEOUT;
				}
			}else{
				//currTick溢出了
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
			//配置中断
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
			SX127xSetOpMode(LORA_OPMODE_CAD);//开启CAD检测
			//Read127xReg(REG_LR_IRQFLAGS);//设置接收后读写任意寄存器可以开始执行命令
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
				//currTick溢出了
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

//非阻塞发送数据(需要配合 轮询 SX127xProcess() 使用)
//buffer:要发送的数据
//size：发送长度
//timeoutSystick:软件超时时间(用systick计时，单位是1systick)，0表示使用默认值
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
	
	Write127xReg( REG_LR_PAYLOADLENGTH, size );	//设置负载长度

	Write127xReg( REG_LR_FIFOTXBASEADDR, 0 );//将发送buf的基地址指向0x00，此时整个fifo都可以用来发送数据
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//将fifi读写指针执行0x00，此时向寄存器0x00读写数据指针会自增的将数据写入fifo
	SX127xSetOpMode(LORA_OPMODE_STANDBY);//LORA_OPMODE_SLEEP 模式不能读写fifo
	
	SX127xWriteFifo((uint8_t *)buffer,size);	//将要发送的数据写入fifo
	//开启中断屏蔽位(只留下了 RFLR_IRQFLAGS_TXDONE 中断没有屏蔽掉)
	Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
	Write127xReg( REG_LR_DIOMAPPING1, ( Read127xReg( REG_LR_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );//DIO0设置为TXdone中断
	//DEBUG("DIO0=%d\r\n0x06:%02x\r\n",SX1276ReadDio0(),Read127xReg(0x06));
	SX127xProcess();	//调用这个是为了更新systick
	SX127xSetOpMode(LORA_OPMODE_TRANSMITTER);	//设置为发送模式
	Read127xReg(REG_LR_IRQFLAGS);//设置发送后读写任意寄存器可以开始发送
	loraStatus=RFLR_STATE_TX_RUNNING;
}

//非阻塞接收函数(需要配合 轮询 SX127xProcess() 使用)
//timeoutSystick:软件超时时间(用systick计时，单位是1systick)，0表示使用默认值
static void Sx127xStartRx(uint32_t timeoutSystick){
	if(timeoutSystick>0){
		softTimeout=timeoutSystick;
	}else{
		softTimeout=DEFAUTL_TIMEOUT;
	}
	loraStatus=RFLR_STATE_RX_INIT;
}

//读取fifo中接收到的数据
static void Sx127xReadRxPackage( void *buffer, uint16_t *size ){
	//读取数据
	uint16_t tmpReciveLength;
	
	tmpReciveLength=Read127xReg(REG_LR_NBRXBYTES);
	if(tmpReciveLength > *size){
		tmpReciveLength=*size;
	}else{
		*size=tmpReciveLength;
	}
	SX127xReadFifo(buffer,tmpReciveLength);
}

//非阻塞 开启一次CAD检测(需要配合 轮询 SX127xProcess() 使用)
static void Sx127xStartCADCheck(void){
	loraStatus=RFLR_STATE_CAD_INIT;
}
