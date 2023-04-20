#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "Board_LED.h"                  // ::Board Support:LED

#include "stdio.h"



extern ARM_DRIVER_USART Driver_USART3;
extern ARM_DRIVER_USART Driver_USART2;


void Init_UART(void){
	Driver_USART3.Initialize(NULL);
	Driver_USART3.PowerControl(ARM_POWER_FULL);
	Driver_USART3.Control(	ARM_USART_MODE_ASYNCHRONOUS |
							ARM_USART_DATA_BITS_8		|
							ARM_USART_STOP_BITS_1		|
							ARM_USART_PARITY_NONE		|
							ARM_USART_FLOW_CONTROL_NONE,
							9600);
	Driver_USART3.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART3.Control(ARM_USART_CONTROL_RX,1);
	
	Driver_USART2.Initialize(NULL);
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

int main (void)
{
	uint8_t tab[50],crnl[2]="\r\n",recup[50];
	int i,j,k;
	Init_UART();

	while (1)
	{	
			Driver_USART3.Receive(tab,50);
			while(Driver_USART3.GetRxCount()<1);
			
			for(i=0;i<50;i++)
			{
				if(tab[i]=='$')
				{
					for(j=i,k=0;tab[j]!='\n';j++,k++)
					{
						recup[k]=tab[j];
					}
					
					while(Driver_USART2.GetStatus().tx_busy == 1);
					Driver_USART2.Send(recup,k);
					
				}
			
			
			}
	}
}