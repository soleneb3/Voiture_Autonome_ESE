/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "LPC17xx.h"                    // Device header

extern ARM_DRIVER_USART Driver_USART1;

void tache1(void const * argument);
void tache2(void const * argument);

osThreadId ID_tache1;
osThreadId ID_tache2;

void myUART_callback(uint32_t event);


void tache1(void const * argument){

	char A = 0xA5;
	char Commande = 0x20;
	int i; 
	char tab[15];
	while(1) {
			while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
			Driver_USART1.Send(&A,1);
			while(Driver_USART1.GetStatus().tx_busy == 1); // attente buffer TX vide
			Driver_USART1.Send(&Commande,1);
	}
}

void tache2(void const * argument){
	char tab[15];
	while(1) {
	osSignalWait(0x01, osWaitForever);
		Driver_USART1.Receive(tab,12);
		while(Driver_USART1.GetRxCount()<1);
	}
}

void myUART_callback(uint32_t event) {
switch(event) {
	case ARM_USART_EVENT_RECEIVE_COMPLETE:
	osSignalSet(ID_tache1, 0x01);
	break;

}
}


void Init_UART(void){
	Driver_USART1.Initialize(NULL);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(	ARM_USART_MODE_ASYNCHRONOUS |
							ARM_USART_DATA_BITS_8		|
							ARM_USART_STOP_BITS_1		|
							ARM_USART_PARITY_NONE		|
							ARM_USART_FLOW_CONTROL_NONE,
							115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);
}

//Interruption sur 25kHz, 40us
void Interruption_TIMER0(void)
{

	LPC_TIM0->PR=0; // PR à 0
	LPC_TIM0->MR0 = 999; // Match register
	LPC_TIM0->MCR |= (3<<0); // RAZ du compteur + interruption
	
	NVIC_SetPriority(TIMER0_IRQn,1); // Timer0 : interruption de priorité 1
	NVIC_EnableIRQ(TIMER0_IRQn); // active les interruptions TIMER0
  LPC_TIM0->TCR = 1; // Lancement Timer
}

//Interruption sur 60.9kHz, 16.5us
void Interruption_TIMER1(void) 
{

	LPC_TIM1->PR=0; // PR à 0
	LPC_TIM1->MR0 = 409 ; // Match register
	LPC_TIM1->MCR |= (3<<0); // RAZ du compteur + interruption
	
	NVIC_SetPriority(TIMER1_IRQn,0); // Timer1 : interruption de priorité 0
	NVIC_EnableIRQ(TIMER1_IRQn); // active les interruptions TIMER1
	
}

void TIMER0_IRQHandler(void)
{
	LPC_TIM0->IR |= (1<<0); // Baisse le drapeau
	LPC_GPIO3->FIOPIN3 = LPC_GPIO3->FIOPIN3 | (1<<2); //Mise à 1 de P3.25
	LPC_TIM1->TCR = 1; // Lancement Timer 1
}

void TIMER1_IRQHandler(void)
{
	LPC_TIM1->IR |= (1<<0); // Baisse le drapeau	
	LPC_GPIO3->FIOPIN3 = LPC_GPIO3->FIOPIN3 & (0<<2);//Mise à 0 de P3.25
	LPC_TIM1->TCR = 0; // Arret Timer 1
}

void Init_GPIO(void)
{
	LPC_GPIO3->FIODIR3|=(1<<2); // P3.25 en sortie
	LPC_GPIO2->FIODIR0&=(0<<2);
}

osThreadDef(tache1,osPriorityNormal,1,0);
osThreadDef(tache2,osPriorityAboveNormal,1,0);

int main (void) {

	Init_UART();
	Init_GPIO(); 
	Interruption_TIMER0(); 
  Interruption_TIMER1();
  osKernelInitialize ();                    // initialize CMSIS-RTOS


  ID_tache1 = osThreadCreate (osThread(tache1), NULL);
	ID_tache2 = osThreadCreate (osThread(tache2), NULL);

  osKernelStart ();                         // start thread execution
osDelay(osWaitForever);	
}
