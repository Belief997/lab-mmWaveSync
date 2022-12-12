#include "sx127x_driver.h"
#include "sx1276-LoRa.h"
#include "sx127x_hal.h"
#include "string.h"

/*
 * Copyright (c) 2019-2020 AIThinker.yangbin All rights reserved.
 *
 * ������ֻ��SX127X������demo�������ο�������֤�����ȶ��ԡ�
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
//软件超时时间(这个使用systick计时的，单位是1systick)
static uint32_t softTimeout=DEFAUTL_TIMEOUT;	
//本地保存的配置信息，用于在复位后重新初始化
static tLoRaSettings localSettingSave={435000000,20,7,7,1,0x000f};	
//当前射频状态机(不是返回给用户的tRFProcessReturnCodes，这个比用户给用户的状态多)
volatile static tRFLRStates  loraStatus=RFLR_STATE_IDLE;	

/**
 * 初始化LoRa配置函数
 * 参数
 *     stting：配置结构体，传入需要设置的参数，eg{435000000,20,8,7,1}，具体内容查看 tLoRaSettings 定义,如果传入NULL，表示加载上次的设置(上次没有就加载默认设置)
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
		memcpy(&localSettingSave,stting,sizeof(tLoRaSettings));	//复制配置信息
	}
	stting=&localSettingSave;	//setting指向备份数据，避免修改导致setting原值改变
	
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
	
	SX127xSetFrf(stting->RFFrequency);//设置频率
	Write127xReg(REG_LR_MODEMCONFIG1,u8_BWList[stting->SignalBw]|u8_CRList[stting->ErrorCoding -1]|RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF);//���ô������������
	Write127xReg(REG_LR_MODEMCONFIG2,u8_SFList[stting->SpreadingFactor-6] | RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF|RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON|0x03);//����SD��CRC����ʱʱ��
	Write127xReg(REG_LR_SYMBTIMEOUTLSB,0xFF);//设置超时时间
	Write127xReg(REG_LR_MODEMCONFIG3,0x0C);//设置低速率(包长超过16ms必须打开)
	
	if(stting->Power>17){
		Write127xReg(REG_LR_PACONFIG,0x80+stting->Power-5);//设置功率
		Write127xReg(0x4d,0x87);//设置更大功率
	}else{
		Write127xReg(REG_LR_PACONFIG,0x80+stting->Power-2);//设置功率
		Write127xReg(0x4d,0x84);//关闭更大功率
	}
	Write127xReg(REG_LR_OCP,0x3B);//过流保护
	//设置前导码长度 REG_LR_PREAMBLEMSB
	Write127xReg(REG_LR_PREAMBLEMSB,stting->PreambleLength>>8);	//前导码高有效位
	Write127xReg(REG_LR_PREAMBLELSB,stting->PreambleLength&0x00ff);	//前导码低有效位

}

void Sx127xRestart(void){
	SX127X_ResetPinControl(0);
	Soft_delay_ms(10);
	SX127X_ResetPinControl(1);
	Soft_delay_ms(10);
	loraStatus=RFLR_STATE_IDLE;
}

//����ΪLoRaģʽ
void SX127xSetLoRaMode(void)
{
  if(0!=(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_LONGRANGEMODE_ON)){
		return;//��ǰ����LoRaģʽ
	}
	SX127xSetOpMode(LORA_OPMODE_SLEEP);
	Write127xReg(REG_LR_OPMODE,Read127xReg(REG_LR_OPMODE)|RFLR_OPMODE_LONGRANGEMODE_ON);//����ΪLoRaģʽ��ֻ���� LORA_OPMODE_SLEEP ģʽ�²��ܲ�����
}

//дsx1278�Ĵ���
void Write127xReg(uint8_t addr,uint8_t data){
	SX1278WriteBuffer( addr,&data, 1 );
}

//��sx1278�Ĵ���
uint8_t Read127xReg(uint8_t addr){
	uint8_t u8_recive;

	SX1278ReadBuffer( addr, &u8_recive, 1 );

	return u8_recive;
}

//дsx1278 fifo
void SX127xWriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1278WriteBuffer( 0, buffer, size );
}

//��sx1278 fifo
void SX127xReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1278ReadBuffer( 0, buffer, size );
}

//设置OpMode
void SX127xSetOpMode(LoRaOpModeType opMode)
{
	if(opMode==SX127xGetOpMode()){
		return;//��ǰģʽ��ȷ�������л�
	}
  Write127xReg(REG_LR_OPMODE,(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_MASK)|opMode|RFLR_OPMODE_FREQMODE_ACCESS_LF );
	Soft_delay_ms(1);
}

//获取OpMode
LoRaOpModeType SX127xGetOpMode(void)
{
    return (LoRaOpModeType)(Read127xReg(REG_LR_OPMODE)&RFLR_OPMODE_MASK);
}

static void SX1278ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t size)
{
    uint8_t i;

    //NSS = 0;
		SpiNSSEnable(0);	//片选spi1

    SpiInOut(addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut(0x00); //读取数据
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
			SpiInOut(buffer[i]);//写入数据
    }

    //NSS = 1;
		SpiNSSEnable(1);
}

//设置载波频率
void SX127xSetFrf(uint32_t fr)
{
	uint8_t frfBuf[4];
	
	fr=fr*0.016384;//根据数据手册计算寄存器要设置的值
	memcpy(frfBuf,&fr,4);
	SX127xSetOpMode(LORA_OPMODE_SLEEP);

	Write127xReg(REG_LR_FRFMSB,frfBuf[2]);
	Write127xReg(REG_LR_FRFMID,frfBuf[1]);
	Write127xReg(REG_LR_FRFLSB,frfBuf[0]);
}


/**
 * 发送数据（阻塞发送）
 * 参数：
 *     data：要发送的数据
 *     len：data的长度
 *     timeoutMs：超时时间（这个时间使用systick）
 * 返回值：
 *     0成功
 *     1超时
 */
uint8_t sx127xSend(uint8_t *data,uint8_t len,uint32_t timeoutMs){
	uint32_t systickBak=GET_TICK_COUNT(),currTick;
	
	Write127xReg( REG_LR_PAYLOADLENGTH, len );	//设置负载长度

	Write127xReg( REG_LR_FIFOTXBASEADDR, 0 );//将发送buf的基地址指向0x00，此时整个fifo都可以用来发送数据
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//将fifi读写指针执行0x00，此时向寄存器0x00读写数据指针会自增的将数据写入fifo
	SX127xSetOpMode(LORA_OPMODE_STANDBY);//LORA_OPMODE_SLEEP 模式不能读写fifo
	
	SX127xWriteFifo(data,len);	//将要发送的数据写入fifo
	//开启中断屏蔽位(只留下了 RFLR_IRQFLAGS_TXDONE 中断没有屏蔽掉)，设置为发送模式后任意读写一个寄存器就会触发发送，这里写入这个开中断正好触发
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
	SX127xSetOpMode(LORA_OPMODE_TRANSMITTER);	//设置为发送模式
	Read127xReg(REG_LR_IRQFLAGS);//设置发送后读写任意寄存器可以开始发送
	
	while(1){
		if(1==SX1276ReadDio0()){
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//清除中断标志位
			return 0;
		}
		currTick=GET_TICK_COUNT();
		if(currTick>=systickBak){
			if(currTick-systickBak>timeoutMs){
				return 1;
			}
		}else{
			//currTick溢出了
			if(currTick+(~systickBak)>timeoutMs){
				return 1;
			}
		}
	}
}

/**
 * 接收数据（阻塞接收）
 * 参数
 *     buf：接收数据的buf空间
 *     len：传入buf的大小，返回后会修改为接收到数据的长度（接收失败不变）
 *     timeoutMS：软件超时时间
 * 返回值
 *     0：接收成功
 *     1：DIO1超时（DIO1超时是硬件返回的，和timeoutMS无关）
 *     2：软件超时
 */
uint8_t sx127xRx(uint8_t *buf,uint8_t *len,uint32_t timeoutMs){
	uint32_t systickBak=GET_TICK_COUNT(),currTick;
	uint8_t u8_reciveLength;
	
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
	
	while(1){
		if(1==SX1276ReadDio0()){
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE);//清除中断标志位
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
			Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXTIMEOUT);//清除中断标志位
			return 1;
		}
		currTick=GET_TICK_COUNT();
		if(currTick>=systickBak){
			if(currTick-systickBak>timeoutMs){
				return 1;
			}
		}else{
			//currTick溢出了
			if(currTick+(~systickBak)>timeoutMs){
				return 1;
			}
		}
	}
}

//������״̬��ѯ����
tRFProcessReturnCodes SX127xProcess( void )
{
	uint32_t currTick=0;
	static uint32_t systickBak=0;
	
	switch(loraStatus){
		case RFLR_STATE_IDLE:
			return RF_IDLE;
		case RFLR_STATE_RX_INIT:
			SX127xSetOpMode(LORA_OPMODE_STANDBY);
			Write127xReg( REG_LR_FIFORXBASEADDR, 0 );//������buf�Ļ���ַָ��0x00����ʱ����fifo������������������
			Write127xReg( REG_LR_FIFOADDRPTR, 0 );//��fifi��дָ��ִ��0x00����ʱ��Ĵ���0x00��д����ָ��������Ĵ�fifo�ж�ȡ����
			Write127xReg( REG_LR_SYMBTIMEOUTLSB, 0xFF );//���� 0x1f rx��ʱ
			
			//�����ж�
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
			SX127xSetOpMode(LORA_OPMODE_RECEIVER);//��������
			//SX127xSetOpMode( LORA_OPMODE_RECIVER_SINGLE );//���ν���
			Read127xReg(REG_LR_IRQFLAGS);//���ý��պ��д����Ĵ������Կ�ʼ����
			systickBak=GET_TICK_COUNT();
			loraStatus=RFLR_STATE_RX_RUNNING;
			return RF_BUSY;
    case RFLR_STATE_RX_RUNNING:
			if(1==SX1276ReadDio0()){
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE);//����жϱ�־λ
				loraStatus=RFLR_STATE_RX_DONE;
				return RF_BUSY;
			}
			if(1==SX1276ReadDio1()){
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXTIMEOUT);//����жϱ�־λ
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
				//currTick�����
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
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//����жϱ�־λ
				loraStatus=RFLR_STATE_TX_DONE;
				return RF_BUSY;
			}
			if(1==SX1276ReadDio0()){
				//�������
				Write127xReg(REG_LR_IRQFLAGS,RFLR_IRQFLAGS_TXDONE);//����жϱ�־λ
				loraStatus=RFLR_STATE_TX_DONE;
				return RF_BUSY;
			}
			currTick=GET_TICK_COUNT();
			if(currTick>=systickBak){
				if(currTick-systickBak>softTimeout){
					loraStatus=RFLR_STATE_TX_TIMEOUT;
				}
			}else{
				//currTick�����
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
			//�����ж�
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
			SX127xSetOpMode(LORA_OPMODE_CAD);//����CAD���
			//Read127xReg(REG_LR_IRQFLAGS);//���ý��պ��д����Ĵ������Կ�ʼִ������
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
				//currTick�����
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

//��������������(��Ҫ��� ��ѯ SX127xProcess() ʹ��)
//buffer:Ҫ���͵�����
//size�����ͳ���
//timeoutSystick:�����ʱʱ��(��systick��ʱ����λ��1systick)��0��ʾʹ��Ĭ��ֵ
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
	
	Write127xReg( REG_LR_PAYLOADLENGTH, size );	//���ø��س���

	Write127xReg( REG_LR_FIFOTXBASEADDR, 0 );//������buf�Ļ���ַָ��0x00����ʱ����fifo������������������
	Write127xReg( REG_LR_FIFOADDRPTR, 0 );//��fifi��дָ��ִ��0x00����ʱ��Ĵ���0x00��д����ָ��������Ľ�����д��fifo
	SX127xSetOpMode(LORA_OPMODE_STANDBY);//LORA_OPMODE_SLEEP ģʽ���ܶ�дfifo
	
	SX127xWriteFifo((uint8_t *)buffer,size);	//��Ҫ���͵�����д��fifo
	//�����ж�����λ(ֻ������ RFLR_IRQFLAGS_TXDONE �ж�û�����ε�)
	Write127xReg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
	Write127xReg( REG_LR_DIOMAPPING1, ( Read127xReg( REG_LR_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );//DIO0����ΪTXdone�ж�
	//DEBUG("DIO0=%d\r\n0x06:%02x\r\n",SX1276ReadDio0(),Read127xReg(0x06));
	SX127xProcess();	//���������Ϊ�˸���systick
	SX127xSetOpMode(LORA_OPMODE_TRANSMITTER);	//����Ϊ����ģʽ
	Read127xReg(REG_LR_IRQFLAGS);//���÷��ͺ��д����Ĵ������Կ�ʼ����
	loraStatus=RFLR_STATE_TX_RUNNING;
}

//���������պ���(��Ҫ��� ��ѯ SX127xProcess() ʹ��)
//timeoutSystick:�����ʱʱ��(��systick��ʱ����λ��1systick)��0��ʾʹ��Ĭ��ֵ
static void Sx127xStartRx(uint32_t timeoutSystick){
	if(timeoutSystick>0){
		softTimeout=timeoutSystick;
	}else{
		softTimeout=DEFAUTL_TIMEOUT;
	}
	loraStatus=RFLR_STATE_RX_INIT;
}

//��ȡfifo�н��յ�������
static void Sx127xReadRxPackage( void *buffer, uint16_t *size ){
	//��ȡ����
	uint16_t tmpReciveLength;
	
	tmpReciveLength=Read127xReg(REG_LR_NBRXBYTES);
	if(tmpReciveLength > *size){
		tmpReciveLength=*size;
	}else{
		*size=tmpReciveLength;
	}
	SX127xReadFifo(buffer,tmpReciveLength);
}

//������ ����һ��CAD���(��Ҫ��� ��ѯ SX127xProcess() ʹ��)
static void Sx127xStartCADCheck(void){
	loraStatus=RFLR_STATE_CAD_INIT;
}
