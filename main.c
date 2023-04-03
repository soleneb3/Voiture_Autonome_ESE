#include "LPC17xx.h"
#include "TIMER.h"




void Initialise_PWM1(void)
{
	LPC_SC->PCONP |= (1<<6); //enable PWM1
	LPC_PWM1->MCR |= (1<<1); // Compteur relancé quand MR0 repasse à 0
	
	LPC_PWM1->PR=0; // Prescaler
	LPC_PWM1->MR0=1249; // Match register 
	
	LPC_PINCON->PINSEL7|=(3<<18); // Validation des sorties PWM1.1 .2 .3
	
	LPC_PWM1->LER |= 0x0000000F; // Compteurs des PWM relancés quand MR passe à 0
	LPC_PWM1->PCR |= 0x00000E00; // Autorise les sorties PWM1/2/3
	
	LPC_PWM1->TCR=1; //validation timer
}


void Choix_Vitesse(int vitesse)
{
	LPC_PWM1->MR2=vitesse; // Choix du rapport cyclique entre 1 et 1249
}

void Init_GPIO(void)
{
	LPC_GPIO3->FIODIR3|=(1<<2); // P3.25 en sortie
	LPC_GPIO2->FIODIR0&=(0<<2);
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

//Interruption sur 1ms
void Interruption_TIMER1(void) 
{

	LPC_TIM1->PR=0; // PR à 0
	LPC_TIM1->MR0 = 37499 ; // Match register
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


int main()
{
	char vitesse;
	
	Init_GPIO(); 
	Initialise_PWM1(); //PWM1 sur P2.0
	Interruption_TIMER0(); 
  Interruption_TIMER1();	

	while (1)
	{
 	 Choix_Vitesse(312); // rapport cyclique à 50%                                                                                                                                                                
	}
	
	return 0;
}