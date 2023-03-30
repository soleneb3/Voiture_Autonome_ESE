// Bibliothèque UART - IUT de Cachan
// Avril 2022

#include <LPC17xx.h>
#include "UART.h"

#define	SANS			0x00
#define	IMPAIRE		0x01
#define PAIRE			0x02

/********************************************************************************
* 	Initialisation périphérique UART1 - 115200 bauds                           	*
*   Arguments :																																	*
*		Entree : parite:   SANS pas de parité																				*
*														IMPAIRE parité impaire															*
*														PAIRE parité paire                             			*
*   Retour : /                                                                  *
********************************************************************************/

void initUART1 (char parite)
{
	LPC_PINCON->PINSEL4 = 0x0A;		// Broche P2.0 pour TX et P2.1 pour RX 

	switch (parite)
	{
		case SANS:		LPC_UART1->LCR = 0x03;	// 8 bits, sans parité, 1 bit de stop
									break;
		
		case IMPAIRE: LPC_UART1->LCR = 0x0B;	// 8 bits, parité impaire, 1 bit de stop
									break;
		
		case PAIRE: 	LPC_UART1->LCR = 0x1B;	// 8 bits, parité paire, 1 bit de stop
									break;
		
		default:	break;
	}
	
	// Réglage de la vitesse de transmission
	LPC_UART1->LCR |= 0x80;				// Forçage bit DLAB=1 (Demande autorisation de modification)
	LPC_UART1->DLM = 0;						// Pas de sur-division de PCLK
	LPC_UART1->DLL = 9;						// Division principale par 9 de PCLK
	LPC_UART1->FDR = 0x21;				// Division fractionnaire par 1,5 (DIVADDVAL=1 et MULVAL=2) 
	LPC_UART1->LCR &= 0x7F;				// Forçage bit DLAB=0 (Fin d'autorisation de modification)
}

/********************************************************************************
* 	Envoi data UART1                     									                    	*
*   Arguments Entree : donnee   	l'octet à envoyer																		*
*   Retour : /                                                                  *
********************************************************************************/

void envoiUART1 (char donnee)
{
	short i;
	LPC_UART1->THR = donnee;	// Envoi de la donnée
	for(i=0;i<5000;i++);			// Laisser un peu de temps avant un autre envoi
}


/********************************************************************************
* 	Réception data UART1                     									                 	*
*   Arguments Entree : /																															*
*   Retour : l'octet à récupérer						                                    *
********************************************************************************/

char recepUART1 (void)
{
	char retour;
	while ((LPC_UART1->LSR & 0x01)==0); 	// Attente caractère reçu (attente tant que bit RDR=0)
	retour = LPC_UART1->RBR;    					// Lecture registre réception (remet automatiquement le bit RDR à 0)
	return retour;
}
