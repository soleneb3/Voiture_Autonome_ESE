/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "LPC17xx.h"
#include "TIMER.h"

extern ARM_DRIVER_USART Driver_USART1;

int Yrecep, Xrecep;

void myUSART_callback(uint32_t obj_idx, uint32_t event);

void tache1(void const * argument);
void tache2(void const * argument);
void USARTthreadR(void const * argument);

osThreadId ID_tache1, ID_USARTthreadR, ID_tache2;


void Init_GPIO(void)
{
	LPC_GPIO3->FIODIR3|=(1<<2); // P3.25 en sortie
	LPC_GPIO2->FIODIR0&=(0<<2);	
	LPC_GPIO0->FIODIR2 |= (5<<0);
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


void Initialise_PWM1(void)
{
	LPC_SC->PCONP |= (1<<6); //enable PWM1
	LPC_PWM1->MCR |= (1<<1); // Compteur relancé quand MR0 repasse à 0
	
	LPC_PWM1->PR=0; // Prescaler
	LPC_PWM1->MR0=1249; // Match register 
	
	LPC_PINCON->PINSEL7|=(3<<18); // Validation des sorties PWM1.2
	
	LPC_PWM1->LER |= 0x0000000F; // Compteurs des PWM relancés quand MR passe à 0
	LPC_PWM1->PCR |= 0x00000E00; // Autorise les sorties PWM1/2/3
	
	LPC_PWM1->TCR=1; //validation timer
}

//Interruption sur 50Hz
void Interruption_TIMER0(void)
{
	LPC_TIM0->PR=0; // PR à 0
	LPC_TIM0->MR0 = 499999; // Match register
	LPC_TIM0->MCR |= (3<<0); // RAZ du compteur + interruption
	
	NVIC_SetPriority(TIMER0_IRQn,1); // Timer0 : interruption de priorité 1
	NVIC_EnableIRQ(TIMER0_IRQn); // active les interruptions TIMER0
  LPC_TIM0->TCR = 1; // Lancement Timer
}


//Interruption pour temps à état haut (5% ou 7,5% ou 10%)
void Interruption_TIMER1(void) 
{
	LPC_TIM1->PR=0; // PR à 0
	LPC_TIM1->MR0 = 37499  ; // Match register
	LPC_TIM1->MCR |= (3<<0); // RAZ du compteur + interruption
	
	NVIC_SetPriority(TIMER1_IRQn,0); // Timer1 : interruption de priorité 0
	NVIC_EnableIRQ(TIMER1_IRQn); // active les interruptions TIMER1
}

void Choix_Vitesse(int vitesse)
{
	LPC_PWM1->MR2=vitesse; // Choix du rapport cyclique entre 1 et 1249
}

void TIMER0_IRQHandler(void)
{
	LPC_TIM0->IR |= (1<<0); // Baisse le drapeau
	LPC_GPIO3->FIOPIN3 = LPC_GPIO3->FIOPIN3 | (1<<2); //Mise à 1 de P2.4
	LPC_TIM1->TCR = 1; // Lancement Timer 1
}


void TIMER1_IRQHandler(void)
{
	LPC_TIM1->IR |= (1<<0); // Baisse le drapeau	
	LPC_GPIO3->FIOPIN3 = LPC_GPIO3->FIOPIN3 & (0<<2);//Mise à 0 de P2.4
	LPC_TIM1->TCR = 0; // Arret Timer 1
}



osThreadId ID_tache1,ID_USARTthreadR, ID_tache2;


void myUSART_callback(uint32_t obj_idx, uint32_t event)
{
	if(event&ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
			osSignalSet(ID_USARTthreadR, 0x01);
	}
}

void USARTthreadR(void const * argument)
{
	char tab[2];
	int Yrecep;
	while (1)
	{
		
		Driver_USART1.Receive(tab,1);
		osSignalWait(0x01, osWaitForever); //mise en sommeil + attente EV 0
		Yrecep = tab[1];
		Yrecep = (Yrecep - 130) * 11.05;
		if(Yrecep <= -1249) Yrecep = -1249;  //saturation
		if(Yrecep >= 1249) Yrecep = 1249;
		if((Xrecep<38100)&&(Xrecep>34000)) Xrecep = 37499;
		if((Yrecep<300)&&(Yrecep>-30)) Yrecep = 0;
		
		Xrecep = tab[0];
		Xrecep = -98*Xrecep + 49999; 
			
		
		if (Yrecep>0) osSignalSet(ID_tache1, 0x02); //mise à un EV1
		else if (Yrecep>0) osSignalSet(ID_tache2, 0x04); //mise à un EV2

	}
}
	

//Avancer
void tache1(void const * argument)
{
	
	while (1)
	{
		osSignalWait(0x02, osWaitForever);//mise en sommeil + attente EV1
		
		//moteur tourne au sens positif
		LPC_GPIO0->FIOPIN2 |= 0x01; //mise à 1 de P0.16
	  LPC_GPIO0->FIOPIN2 &= 0xFB; //mise à 0 de P0.18
		
		Choix_Vitesse(Yrecep); //Propulsion
		LPC_TIM1->MR0 = Xrecep; //Direction
		
	}
}


//Reculer
void tache2(void const * argument)
{
  osSignalWait(0x04, osWaitForever);//mise en sommeil + attente EV2

	while (1)
	{
   //moteur tourne au sens inverse
		LPC_GPIO0->FIOPIN2 &= 0xFE; //mise à 0 de P0.16
	  LPC_GPIO0->FIOPIN2 |= 0x04; //mise à 1 de P0.18

   Choix_Vitesse(Yrecep); //Propulsion
	 LPC_TIM1->MR0 = Xrecep; //Direction

   osSignalWait(0x04, osWaitForever);//mise en sommeil + attente EV2

	}
}


/*
 * main: initialize and start the system
 */

osThreadDef(tache1, osPriorityNormal,1,0);
osThreadDef(tache2, osPriorityNormal,1,0);
osThreadDef(USARTthreadR, osPriorityNormal,1,0);

int main (void) {
  osKernelInitialize ();                    // initialize CMSIS-RTOS

  // initialize peripherals here
	Init_GPIO();
	Init_UART();
	Initialise_PWM1();
	Interruption_TIMER0();
	Interruption_TIMER1();

  // create 'thread' functions that start executing,
  // example: tid_name = osThreadCreate (osThread(name), NULL);
	ID_tache1 = osThreadCreate(osThread(tache1),NULL);
	ID_tache2 = osThreadCreate(osThread(tache2),NULL);
	ID_USARTthreadR = osThreadCreate(osThread(USARTthreadR),NULL);

  osKernelStart ();                         // start thread execution 
	osDelay(osWaitForever); //mise en sommeil infinie
}
