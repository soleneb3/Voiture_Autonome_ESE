/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   STM32F4xx HAL API Template project 
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Board_LED.h"                  // ::Board Support:LED
#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "Driver_CAN.h"               // ARM::CMSIS Driver:USART:Custom

#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS2                  // when RTE component CMSIS RTOS2 is used
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#endif

#ifdef RTE_CMSIS_RTOS2_RTX5






/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}

#endif

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	

extern ARM_DRIVER_USART Driver_USART2;
osThreadId id_CANthreadT;
osThreadId id_CANthreadR;

void CANThreadR(void const *argument);
void CANThreadT(void const *argument);
void myUSART_callback(uint32_t obj_idx, uint32_t event); // arguments imposés





void sendCommand(char cmd, char P1,char P2)
{

	unsigned char tab[10] = {0x7E ,0xFF, 0x06, 0x00,0x00,0x00,0x00,0x00,0x00,0xEF};
	short checksum=0;
	
	tab[3] = cmd;
	tab[5] = P1;
	tab[6]= P2;
	checksum = 0-(tab[1]+tab[2]+tab[3]+tab[4]+tab[5]+tab[6]);
	tab[7] = ((checksum & 0xFF00) >>8 );
	tab[8] = checksum & 0x00FF;
	while(Driver_USART2.GetStatus().tx_busy == 1); // attente buffer TX vide
	Driver_USART2.Send(tab,10);
	osDelay(100);
	
}

void chooseSoundFromFile(char file, char sound)
{
	sendCommand(0x0F,file,sound); // ne fonctionne pas ATM
	
}

void chooseSound(char sound)
{
	sendCommand(0x03,0x00,sound);

}


void Init_UART(void){
	Driver_USART2.Initialize(myUSART_callback);
	Driver_USART2.PowerControl(ARM_POWER_FULL);
	Driver_USART2.Control(	ARM_USART_MODE_ASYNCHRONOUS |
							ARM_USART_DATA_BITS_8		|
							ARM_USART_STOP_BITS_1		|
							ARM_USART_PARITY_NONE		|
							ARM_USART_FLOW_CONTROL_NONE,
							9600);
	Driver_USART2.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART2.Control(ARM_USART_CONTROL_RX,1);

	
}
void Init_CAN(void);
void setFiltreCAN(void);
void sendCAN(short id,char *data);
void receiveCAN(void);

extern ARM_DRIVER_CAN Driver_CAN1;
void Init_CAN(void)
{

	Driver_CAN1.Initialize(NULL,NULL);
	Driver_CAN1.PowerControl(ARM_POWER_FULL);
	Driver_CAN1.SetMode(ARM_CAN_MODE_INITIALIZATION);
	Driver_CAN1.SetBitrate( ARM_CAN_BITRATE_NOMINAL, // débit fixe
	125000, // 125 kbits/s (LS)
	ARM_CAN_BIT_PROP_SEG(5U) | // prop. seg = 5 TQ
	ARM_CAN_BIT_PHASE_SEG1(1U) | // phase seg1 = 1 TQ
	ARM_CAN_BIT_PHASE_SEG2(1U) | // phase seg2 = 1 TQ
	ARM_CAN_BIT_SJW(1U)); // Resync. Seg = 1 TQ
	
	Driver_CAN1.ObjectConfigure(1,ARM_CAN_OBJ_TX); // Objet 1 pour émission
	Driver_CAN1.ObjectConfigure(0,ARM_CAN_OBJ_RX); // Objet 0 pour réception
	Driver_CAN1.SetMode(ARM_CAN_MODE_NORMAL); // fin initialisation
	
	setFiltreCAN();
}

void setFiltreCAN(void)
{
	Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD ,
	ARM_CAN_STANDARD_ID(0x5F6),
	0) ; 
}

void sendCAN(short id,char *data)
{
	unsigned char data_buf[10];
	ARM_CAN_MSG_INFO tx_msg_info;
	tx_msg_info.id = ARM_CAN_STANDARD_ID (id);
	tx_msg_info.rtr = 0; // 0 = trame DATA
	data_buf [0] = 0xFA; // data à envoyer à placer dans un tableau de char
	Driver_CAN1.MessageSend(1, &tx_msg_info, data_buf, 1); // 1 data à envoyer
		
	
}

void receiveCAN(void)
{
	unsigned char data_buf[10];
	short identifiant =0;
	char retour =0;
	char taille=0;
	ARM_CAN_MSG_INFO rx_msg_info;
	Driver_CAN1.MessageRead(0, &rx_msg_info, data_buf, 8); // 8 data max
	identifiant = rx_msg_info.id; // (int)
	retour = data_buf [0] ; // 1ère donnée de la trame récupérée (char)
	taille = rx_msg_info.dlc; // nb data (char)
		
	
}

void myUSART_callback(uint32_t obj_idx, uint32_t event) // arguments imposés
{

switch (event)
{

case ARM_USART_EVENT_SEND_COMPLETE:
	osSignalSet(id_CANthreadT, 0x01);
	LED_On (4);
break;
case ARM_USART_EVENT_RX_BREAK: 
	LED_Off(1);
	osSignalSet(id_CANthreadR, 0x01);
case 	ARM_USART_EVENT_RECEIVE_COMPLETE :
	LED_Off(1);
	osSignalSet(id_CANthreadR, 0x01);
break;
}
}
void CANThreadT(void const *argument)
{
	char data[10]="333";
	osEvent evt;
	while(1)
	{
		
		Driver_USART2.Send(data,3);
		evt=osSignalWait(0x01,osWaitForever);
		LED_On (2);
		
		osDelay(400);
		LED_Off (2);
		osDelay(400);
	}
	
}

void CANThreadR(void const *argument)
{
	
	char receive[50];
	osEvent evt;
	while(1)
	{
		
		Driver_USART2.Receive(receive,50);
		
		evt=osSignalWait(0x01,osWaitForever);
		if(receive[0]==0x31)
		{
			LED_On(4);
		}
		else if(receive[0]==0x32)
		{
			LED_Off(1);
		}
	}

}


//void const HAL_USART_T2ProcessCpltCallback
osThreadDef (CANThreadR, osPriorityNormal, 1, 0); // 1 instance, taille pile par défaut
osThreadDef (CANThreadT, osPriorityNormal, 1, 0); // 1 instance, taille pile par défaut





int main(void)
{
	uint8_t tab[13];
	uint8_t soluce[13] = {0x30,0x38,0x30,0x30,0x38,0x43,0x32,0x33,0x45,0x39,0x34,0x45,0x03};
	char i=0,j=0,error=0;
	
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();
	
  /* Configure the system clock to 168 MHz */
	SystemClock_Config();
  SystemCoreClockUpdate();

  /* Add your application code here
     */


  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();

  /* Create thread functions that start executing, 
  Example: osThreadNew(app_main, NULL, NULL); */

  /* Start thread execution */
  osKernelStart();

	
	
	
	

	LED_Initialize();
	Init_UART();
	Init_CAN();
		
	//init ampli///
	id_CANthreadR = osThreadCreate ( osThread ( CANThreadR ), NULL ) ;
	id_CANthreadT = osThreadCreate ( osThread ( CANThreadT ), NULL ) ;
	

	
	/*sendCommand(0x09,0x00,0x02); // set SD(data 2 = 1 pour TF) comme Source des fichiers 
	osDelay(300);
	sendCommand(0x10,0x01,0x01); // set Volume Open (data1 =1) et volume ) 15/31 (data 2 =1)*/
	

	LED_On (3);
	LED_On (1);		

	osDelay(osWaitForever);
	while (1)
  {
		/*LED_On (2);
		sendCAN(0x12F,"000");
		osDelay(400);
		LED_Off (2);
		osDelay(400);*/
		
		/*
		Driver_USART2.Receive(tab,1);
		while(Driver_USART2.GetRxCount()<1);
		
		Driver_USART2.Receive(tab,13);
		while(Driver_USART2.GetRxCount()<13);
		
		

		
		error =0;
		for (i=0;i<13;i++)
		{
			if (tab[i] != soluce[i])
			{
				LED_Off (1); //////////////////////MESSAGE D'ERREUR////////////////////////////////
				i=13;
			}
			if(i==12)
			{
				sendCommand(0x03,0x00,0x04); //////////////////////OUVRIR LA PORTE ////////////////////////////////
				LED_Off(1);
				osDelay(500);
				for(j=0;j<13;j++)
				{
					tab[j]=0;
				}
				LED_On(1);
			}
		}*/
		
  }
	
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
