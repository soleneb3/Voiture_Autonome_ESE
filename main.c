/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "LPC17xx.h"                    // Device header
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "GLCD_Config.h"                // Keil.MCB1700::Board Support:Graphic LCD
#include "stdio.h"

extern ARM_DRIVER_USART Driver_USART1;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

void tache1(void const * argument);
void tache2(void const * argument);
void tache3(void const * argument);

osThreadId ID_tache1;
osThreadId ID_tache2;
osThreadId ID_tache3;
osMutexId ID_mut_GLCD;
osMailQId ID_bal;


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
			osDelay(2000);
	}
}

void tache2(void const * argument){
	char tab[15];
	char message[15];
	char message2[15];
	short q6, q2;
	char q6_1, q6_2, q2_1, q2_2;
	short *ptr;
	
	while(1) {
		Driver_USART1.Receive(tab,5);
		osSignalWait(0x01, osWaitForever);
			q6_1 = tab[1]>>1;
			q6_2 = tab[2];
			q6 = (short)(q6_2<<7|q6_1);
			q2_1= tab[3];
			q2_2 = tab[4];
			q2 = (short)(q2_2<<8|q2_1);	
			
			//q6 = (q6&0xFE00)+(q6&01FF)*2^-10;
			
			ptr = osMailAlloc(ID_bal, osWaitForever);
			*ptr = q2;
			osMailPut(ID_bal, ptr);
			
			ptr = osMailAlloc(ID_bal, osWaitForever);
			*ptr = q6;
			osMailPut(ID_bal, ptr);
			
			
	}
}

void tache3(void const * argument){
	char message[15];
	char message2[15];
	int i;
	short *recep, distance, angle;
	double x, y;
	osEvent EVretour;
	while(1) {
		EVretour = osMailGet(ID_bal, osWaitForever);
		recep = EVretour.value.p;
		distance = *recep;
		osMailFree(ID_bal, recep);
//		osMutexWait(ID_mut_GLCD, osWaitForever);
//		sprintf(message2, " distance = %d ",distance) ; //on stocke dans message 
//		GLCD_DrawString(1,1,message2); //colonne, ligne, message
//		osMutexRelease(ID_mut_GLCD);
		distance = (distance&0xA000) + (distance&0x3FFF)*(2^-14);
		
		EVretour = osMailGet(ID_bal, osWaitForever);
		recep = EVretour.value.p;
		angle = *recep;
		osMailFree(ID_bal, recep);
		
		angle = (angle&0xFE00)+(angle&0x1FF)*2^-10;
		
		y = cos(angle) * distance;
		x = sin(angle) * distance;
		
	}
}


void myUART_callback(uint32_t event) {
if (event&ARM_USART_EVENT_RECEIVE_COMPLETE) {
		osSignalSet(ID_tache2, 0x01);
		} 
if (event&ARM_USART_EVENT_SEND_COMPLETE) {
		//osSignalSet(ID_tache2, 0x01);
		
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


void Initialise_PWM1(void)
{
	LPC_SC->PCONP |= (1<<6); //enable PWM1
	LPC_PWM1->MCR |= (1<<1); // Compteur relancé quand MR0 repasse à 0
	
	LPC_PWM1->PR=0; // Prescaler
	LPC_PWM1->MR0=999; // Match register pour 25kHz
	
	LPC_PINCON->PINSEL7|=(3<<18); // Validation des sorties PWM1.2
	
	LPC_PWM1->LER |= 0x0000000F; // Compteurs des PWM relancés quand MR passe à 0
	LPC_PWM1->PCR |= 0x00000E00; // Autorise les sorties PWM2
	
	LPC_PWM1->TCR=1; //validation timer
}

void Choix_Vitesse(void)
{
	LPC_PWM1->MR2=409; // Choix du rapport cyclique entre 1 et 999, 60.9kHz
}


osThreadDef(tache1,osPriorityAboveNormal,1,0);
osThreadDef(tache2,osPriorityNormal,1,0);
osThreadDef(tache3,osPriorityNormal,1,0);
osMutexDef(mut_GLCD);
osMailQDef(bal,1,short);

int main (void) {

	Init_UART();
	Initialise_PWM1();
	Choix_Vitesse();
	GLCD_Initialize() ;
	GLCD_ClearScreen();
	GLCD_SetFont(&GLCD_Font_16x24) ;
	//Init_GPIO(); 
	//Interruption_TIMER0(); 
  //Interruption_TIMER1();
  osKernelInitialize ();                    // initialize CMSIS-RTOS


  ID_tache1 = osThreadCreate (osThread(tache1), NULL);
	ID_tache2 = osThreadCreate (osThread(tache2), NULL);
	ID_tache3 = osThreadCreate (osThread(tache3), NULL);
	ID_mut_GLCD = osMutexCreate(osMutex(mut_GLCD));
	ID_bal = osMailCreate(osMailQ(bal), NULL);

  osKernelStart ();                         // start thread execution
	osDelay(osWaitForever);	
}
