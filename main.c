#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "GLCD_Config.h"                // Keil.MCB1700::Board Support:Graphic LCD
#include "stdio.h"


extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;
extern ARM_DRIVER_USART Driver_USART1;


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

int main (void)
	{
	char message[15] ;
	char tab[2];
	int Yrecep;
	int Xrecep;
	GLCD_Initialize() ;
	GLCD_ClearScreen();
	GLCD_SetFont(&GLCD_Font_16x24) ;

	Init_UART();
	while (1)
		{	
			Driver_USART1.Receive(tab,2);
			while(Driver_USART1.GetRxCount()<1);
			Yrecep = tab[1];
			Yrecep = (Yrecep - 130) * 11.05;
			if(Yrecep <= -1249) Yrecep = -1249;  //saturation
			if(Yrecep >= 1249) Yrecep = 1249;
			if((Yrecep<300)&&(Yrecep>-30)) Yrecep = 0;
			sprintf(message, " Y = %d ",Yrecep) ; //on stocke dans message 
			GLCD_DrawString(1,1,(char*)message) ; //colonne, ligne, message
			
					
			Xrecep = tab[0];
			Xrecep = -98*Xrecep + 49999; 
			if((Xrecep<38100)&&(Xrecep>34000)) Xrecep = 37499;
			sprintf(message, " X = %d ",Xrecep) ; //on stocke dans message 
			GLCD_DrawString(1,50,(char*)message) ; //colonne, ligne, message
		}
	}