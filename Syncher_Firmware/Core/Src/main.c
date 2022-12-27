/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "sx127x_driver.h"
#include "stm32l0xx_it.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE                                 20 // Define the payload size here
#define MAX_USER                                    5  
#define TIME_GAP                                    1000 //ms
#define TIME_WAIT									4 //s
#define RX_TIMMER_SYNC                              2 // byte position of sync time 
#define SENSOR_BYTE                                 7
#define SENSOR_TIMEOUT								500 //ms
#define MSG_TIMEOUT_CNT                             40000
#define NOISE_THRE 									50 
#define TIMER_CNT_BIT 								2500
#define REPORT_CIRCLE                               10000


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
enum cap_state{
    CAPTURE_FALLING = 0,
    CAPTURE_RISING,
    CAPTURE_IDLE,
	CAPTURE_READY,
	CAPTURE_FIRSTCAP,
	CAPTURE_FINISHED
} ;

enum tag_state{
    TAG_READY = 0,
    TAG_RECEIVE,
    TAG_START,
    TAG_SENDDING
};

enum master_state{
    M_READY=0,
    M_START,
    M_RECEIVE,
    M_SENDDING
};

extern __IO uint32_t uwTick;
uint8_t pulse_cnt = 0;
uint8_t pulse_max = 8;

extern volatile uint32_t uwTick;
static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
//static uint8_t Buffer[BUFFER_SIZE];					// RF buffer
tRadioDriver *Radio = NULL;

volatile uint8_t flag;//���ݽ����ı�־λ������ʱtag=1

#define LEN_RXBUF 128
u8 txBuf1[20];  //9.30
u8 rxBuf[LEN_RXBUF];

uint16_t Prescal_list[] = {500,1000,2000,5000,10000};

volatile u32 MasterDelayMs = 27;
volatile u32 MasterDelayUs = 50;


extern tRadioDriver g_Radio;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM22_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int Key_timer_cb(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//	if(huart == &huart1){
//		//HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuf1, SENSOR_BYTE);
//		read_over = 1;
//	}
////	else if(huart == &huart2){
////		HAL_UART_Transmit(&huart2, (uint8_t *)rxBuf2, strlen(txBuf2), 0xffff);
////		memset(rxBuf2,0, strlen(txBuf1));
////		HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuf2, 10);
////	}
//}
void delay_0(void){
	u32 i=0;
	u32 j=0;
    
	for(i=0;i<1000;i++){
		for(j=0;j<1000;j++){
		}
	}
}

void delay_us(u32 time){
	u32 i=0;
	u32 j=0;
    
	for(i=0;i<time;i++){
		for(j=0;j<1000;j++){
		}
	}
}


void delayMsBySystick(uint32_t timeoutMs){
	uint32_t systickBak,currTick;
	systickBak = GetTick();
	while(1){
		currTick=GetTick();
		if(currTick>=systickBak){
			if(currTick-systickBak>timeoutMs){
				return;
			}
		}else{
			//currTick 溢出
			if(currTick+(~systickBak)>timeoutMs){
				return;
			}
		}
	}
}

void MasterDelayMsAdd(void)
{
    MasterDelayMs++;
}
void MasterDelayMsMinus(void)
{
    if(MasterDelayMs > 0)
    {
        MasterDelayMs--;
    }
}

bool GetMSMode(void)
{
    // M/S mode confirm: read GPIO value
    // GPIO_PIN_SET -> op_mode true -> Master
    return (GPIO_PIN_SET == HAL_GPIO_ReadPin(MS_Select_GPIO_Port, MS_Select_Pin));
}

void SetKeyLED(bool val)
{
    HAL_GPIO_WritePin(LED_KEY_GPIO_Port, LED_KEY_Pin, val?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void toggle_led(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
void tri_mmwave(void)
{
    int i = 0;

    // 产生�?个正脉冲
//            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin,GPIO_PIN_SET);	   
//    GPIO_TypeDef * GPIOx = Pulse_GPIO_Port;
//    GPIOx->BSRR = Pulse_Pin;

    // 控制高电平时�?
//	while(i < 1)
    { //delay 
        i = i+1;
        i = i+1;
        i = i+1;
        i = i+1;
    }
	HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin,GPIO_PIN_RESET);
//    GPIOx->BRR = Pulse_Pin ;

	pulse_cnt++;

    // Slaver Passive trig timer reset and start
    HAL_TIM_Base_Stop_IT(&htim22);
    HAL_TIM_Base_Start_IT(&htim22);
}

// 按键 IO 边沿触发中断，重置定时器延迟获取按键状�??
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
}


void SendTrigPkg(void)
{
    u16 i=0; // for debug
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    if(0==sx127xSend((uint8_t *)"t",strlen("t"),1000))
    {   
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//                toggle_led();
    }else
    {
        i = 2;
//              DEBUG("send timeout\r\n");
    }

    // trig pulse timer
    delayMsBySystick(MasterDelayMs);
    delay_us(MasterDelayUs);
//    HAL_TIM_Base_Start_IT(&htim22);
    tri_mmwave();
}

// 对按下的按键执行响应动作
int Key_handle(uint32_t key)
{
    static uint8_t prelist_cnt = 0;
    if(key == BTN1_Pin){
        SendTrigPkg();

    }
    
    if(key == BTN2_Pin){
//        MasterDelayMsAdd();
//        MasterDelayMsMinus();

    }
        
    if(key == BTN3_Pin){
//        MasterDelayMsAdd();
//        MasterDelayMsMinus();
        
    }

    return 0;
}


int Key_timer_cb(void)
{  
    HAL_TIM_Base_Stop_IT(&htim2);
    // 依次判断哪个按键按下
    if(0 == HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin))
    {

//        HAL_GPIO_TogglePin(LED_KEY_GPIO_Port, LED_KEY_Pin);
        HAL_GPIO_WritePin(LED_KEY_GPIO_Port, LED_KEY_Pin,GPIO_PIN_RESET);
//        toggle_led();
        Key_handle(BTN1_Pin);

    }
    else if(0 == HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin))
    {

        toggle_led();
        Key_handle(BTN2_Pin);

    }
    else if(0 == HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin))
    {

        toggle_led();
        Key_handle(BTN3_Pin);
    }

    return 0;
}


void OnSlave( void )
{
    u8 ret, rxCount=255;
    while (1)
    {
        rxCount = 255;
//      memset(rxBuf,0,LEN_RXBUF);
        ret = sx127xRx(rxBuf,&rxCount,2000);    
        switch(ret){
            case 0:
                toggle_led();
                if((rxBuf[0]=='t') && (rxCount != 255))
                {
                    //HAL_UART_Transmit(&huart1, (uint8_t *)rxBuf1, strlen(rxBuf1), 0xffff);
//                  toggle_led();
                    // Slaver 立即触发
                    tri_mmwave();
                }
                
                break;
            case 1:
            case 2:
//                  DEBUG("rx timeout ret:%d\r\n",u8_ret);
                break;
            default:
//                  DEBUG("unknow ret:%d\r\n",u8_ret);
                break;
        }
    }
}
/*
 * Manages the master operation (board on the plane).
 */
void OnMaster( void )
{
    while(1){};


    
	static uint32_t time_now;
	static int master_state = TAG_READY;
	uint32_t slot_time_100ms=1;
	uint8_t rf_state = Radio->Process();
	
    delay_0();
    
	switch (master_state)
    {
    case M_READY:
      /* code */
      //memset(start_buf, 0, BUFFER_SIZE);
      master_state = M_START;
	  //刚到，不�?要等
      break;

    case M_START: //发�?�同步帧
		//sprintf(txBuf1, "T%d", TIME_WAIT);//(uwTick / 1000)%TIME_WAIT
		//printf("start_buf : %s\r\n", start_buf);  //usart
		//sprintf(txBuf1, "T%d", 1);//(uwTick / 1000)%TIME_WAIT
		txBuf1[0] = 'T';
//		txBuf1[1] = TIME_WAIT-1; //number of slot of a circle
//		txBuf1[2] = slot_time_100ms; //one slot time = slot_time_100ms*100ms
//		txBuf1[3] = '\r';
//		txBuf1[4] = '\n';
//		Radio->SetTxPacket(txBuf1, strlen(txBuf1));
		master_state = M_SENDDING;
		time_now = uwTick;
			
      break;
	case M_SENDDING:
		if(rf_state==RF_TX_DONE){
			master_state = M_RECEIVE;
//			Radio->StartRx();
		}
        else
        {
            delay_0();
            delay_0();
            delay_0();
            delay_0();
        }
		break;
    case M_RECEIVE:
      //等待�?个周期并进行接收
		
      if (uwTick - time_now < TIME_WAIT * slot_time_100ms *100)
      {
        if(rf_state==RF_RX_DONE){
			Radio->GetRxPacket( rxBuf, ( uint16_t* )&BufferSize );
			//
			if(rxBuf[0]>='0' && rxBuf[4]<='9'){
				rxBuf[6] = '\r';
				rxBuf[7] = '\n';
				HAL_UART_Transmit(&huart1, (uint8_t *)rxBuf, strlen(rxBuf), 0xffff);//Usart send to server
				toggle_led();
			}
			memset(rxBuf,0,BufferSize);
		}
		else{
			master_state = M_RECEIVE;
		}
      }      
	  else{
		master_state = M_READY;
	  }
      break;
    default:
      break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	bool op_mode = false;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM22_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//	HAL_UART_Transmit(&huart1, (uint8_t *)txBuf1, strlen(txBuf1), 0xffff);
//	HAL_UART_Transmit(&huart2, (uint8_t *)txBuf2, strlen(txBuf2), 0xffff);
//	HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuf1, SENSOR_BYTE);
//	HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuf2, 10);

    // radio init
    //435M, 20dbm, 125kHz, SF=12, 4/5,0x0005
    tLoRaSettings setting={435000000,20,7,12,1,0x0005};  
    g_Radio.Init(&setting);  
   
    // M/S mode confirm: read GPIO value
    // GPIO_PIN_SET -> op_mode == true -> Master
    op_mode = GetMSMode();

    // 按键灯标识主从
    SetKeyLED(op_mode);

    if(op_mode)
   {
        //master
        OnMaster();
	}
    else
    {
		//slave
        OnSlave();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 31999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 99;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 15999;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_KEY_GPIO_Port, LED_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Lora_Rst_GPIO_Port, Lora_Rst_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Lora_CS_GPIO_Port, Lora_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO0_Pin DIO1_Pin DIO2_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin|DIO1_Pin|DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO3_Pin DIO4_Pin DIO5_Pin */
  GPIO_InitStruct.Pin = DIO3_Pin|DIO4_Pin|DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_KEY_Pin */
  GPIO_InitStruct.Pin = LED_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Pulse_Pin Lora_Rst_Pin */
  GPIO_InitStruct.Pin = Pulse_Pin|Lora_Rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MS_Select_Pin */
  GPIO_InitStruct.Pin = MS_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MS_Select_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Lora_CS_Pin */
  GPIO_InitStruct.Pin = Lora_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Lora_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM21 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM21) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
      if (htim->Instance == TIM22) 
        {
            if(pulse_cnt < pulse_max)
            {
                int i = 0;

                // 产生�?个正脉冲
                //            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin,GPIO_PIN_SET);    
//                GPIO_TypeDef * GPIOx = Pulse_GPIO_Port;
//                GPIOx->BSRR = Pulse_Pin;

                // 控制高电平时�?
//                while(i < 1)
                { //delay 
                    i = i+1;
                    i = i+1;
                    i = i+1;
                    i = i+1;

                }
                HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin,GPIO_PIN_RESET);
//                GPIOx->BRR = Pulse_Pin ;

                pulse_cnt++;

            }
            else
            {
                pulse_cnt = 0;
                HAL_TIM_Base_Stop_IT(&htim22);
                HAL_GPIO_WritePin(LED_KEY_GPIO_Port, LED_KEY_Pin,GPIO_PIN_SET);
            }
    	}
    else if(htim->Instance == TIM2)
    {
        Key_timer_cb();
    }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
