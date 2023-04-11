/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "LPC17xx.h"
#include "TIMER.h"

void tache1(void const * argument);
//void tache2(void const * argument);

osThreadId ID_tache1;// ID_tache2;

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

void Init_GPIO(void)
{
	LPC_GPIO3->FIODIR3|=(1<<2); // P3.25 en sortie
	LPC_GPIO2->FIODIR0&=(0<<2);	
	LPC_GPIO0->FIODIR2 |= (5<<0);
}


void Choix_Vitesse(int vitesse)
{
	LPC_PWM1->MR2=vitesse; // Choix du rapport cyclique entre 1 et 1249
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


//Avancer
void tache1(void const * argument)
{
	while (1)
	{
		//moteur tourne au sens positif
		LPC_GPIO0->FIOPIN2 |= 0x01; //mise à 1 de P0.16
	  LPC_GPIO0->FIOPIN2 &= 0xFB; //mise à 0 de P0.18
		
		if((LPC_GPIO1->FIOPIN3 |= 0xFD) == 0xFD ) //a droite
		{
		Choix_Vitesse(624);
		LPC_TIM1->MR0 = 24999  ; // Match register
		}
		else if((LPC_GPIO1->FIOPIN3 |= 0xFE) == 0xFE ) //a gauche
		{
			Choix_Vitesse(624);
			LPC_TIM1->MR0 = 49999  ; // Match register
		}
		else //tout droit
		{
			Choix_Vitesse(624);
			LPC_TIM1->MR0 = 37499  ; // Match register
		}
	}
}


//Reculer
/*void tache2(void const * argument)
{
	while (1)
	{
   //moteur tourne au sens inverse
		LPC_GPIO0->FIOPIN2 &= 0xFE; //mise à 0 de P0.16
	  LPC_GPIO0->FIOPIN2 |= 0x04; //mise à 1 de P0.18

		if((LPC_GPIO2->FIOPIN1 |= 0xFB) == 0xFB ) //a droite
		{
		Choix_Vitesse(624);
		LPC_TIM1->MR0 = 24999  ; // Match register
		}
		else //((LPC_GPIO1->FIOPIN3 |= (1<<1)) == 1 ) //a gauche
		{
			Choix_Vitesse(624);
			LPC_TIM1->MR0 = 49999  ; // Match register
		}	
	}
}*/


/*
 * main: initialize and start the system
 */

osThreadDef(tache1, osPriorityNormal,1,0);
//osThreadDef(tache2, osPriorityNormal,1,0);

int main (void) {
  osKernelInitialize ();                    // initialize CMSIS-RTOS

  // initialize peripherals here
	Init_GPIO();
	Initialise_PWM1();
	Interruption_TIMER0();
	Interruption_TIMER1();

  // create 'thread' functions that start executing,
  // example: tid_name = osThreadCreate (osThread(name), NULL);
	ID_tache1 = osThreadCreate(osThread(tache1),NULL);
	//ID_tache2 = osThreadCreate(osThread(tache2),NULL);

  osKernelStart ();                         // start thread execution 
	osDelay(osWaitForever); //mise en sommeil infinie
}
