// Biblioth�que UART - IUT de Cachan
// Avril 2022

#include <LPC17xx.h>
#include "UART.h"

#define	SANS			0x00
#define	IMPAIRE		0x01
#define PAIRE			0x02

/********************************************************************************
* 	Initialisation p�riph�rique UART1 - 115200 bauds                           	*
*   Arguments :																																	*
*		Entree : parite:   SANS pas de parit�																				*
*														IMPAIRE parit� impaire															*
*														PAIRE parit� paire                             			*
*   Retour : /                                                                  *
********************************************************************************/

void initUART1 (char parite)
{
	LPC_PINCON->PINSEL4 = 0x0A;		// Broche P2.0 pour TX et P2.1 pour RX 

	switch (parite)
	{
		case SANS:		LPC_UART1->LCR = 0x03;	// 8 bits, sans parit�, 1 bit de stop
									break;
		
		case IMPAIRE: LPC_UART1->LCR = 0x0B;	// 8 bits, parit� impaire, 1 bit de stop
									break;
		
		case PAIRE: 	LPC_UART1->LCR = 0x1B;	// 8 bits, parit� paire, 1 bit de stop
									break;
		
		default:	break;
	}
	
	// R�glage de la vitesse de transmission
	LPC_UART1->LCR |= 0x80;				// For�age bit DLAB=1 (Demande autorisation de modification)
	LPC_UART1->DLM = 0;						// Pas de sur-division de PCLK
	LPC_UART1->DLL = 9;						// Division principale par 9 de PCLK
	LPC_UART1->FDR = 0x21;				// Division fractionnaire par 1,5 (DIVADDVAL=1 et MULVAL=2) 
	LPC_UART1->LCR &= 0x7F;				// For�age bit DLAB=0 (Fin d'autorisation de modification)
}

/********************************************************************************
* 	Envoi data UART1                     									                    	*
*   Arguments Entree : donnee   	l'octet � envoyer																		*
*   Retour : /                                                                  *
********************************************************************************/

void envoiUART1 (char donnee)
{
	short i;
	LPC_UART1->THR = donnee;	// Envoi de la donn�e
	for(i=0;i<5000;i++);			// Laisser un peu de temps avant un autre envoi
}


/********************************************************************************
* 	R�ception data UART1                     									                 	*
*   Arguments Entree : /																															*
*   Retour : l'octet � r�cup�rer						                                    *
********************************************************************************/

char recepUART1 (void)
{
	char retour;
	while ((LPC_UART1->LSR & 0x01)==0); 	// Attente caract�re re�u (attente tant que bit RDR=0)
	retour = LPC_UART1->RBR;    					// Lecture registre r�ception (remet automatiquement le bit RDR � 0)
	return retour;
}
