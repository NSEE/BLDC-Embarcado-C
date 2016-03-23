//parte_saida

//projeto GENIUS
 
//bibliotecas
#include <stdlib.h>
#include <p18f452.h>
#include <stdio.h>
#include <timers.h>
#include <math.h>
#include <delays.h>
#include <stdio.h>
#include <stdlib.h>

//configuracoes do hardware
#pragma config WDT = OFF, LVP = OFF, OSC = XT, PWRT = ON, BOR = ON

//botoes
#define	botao1 PORTBbits.RB0 //botoes 1,2,3,4
#define	botao2 PORTBbits.RB1
#define	botao3 PORTBbits.RB2
#define	botao4 PORTBbits.RB3

//I/Os
void io(){	
	//entradas
	TRISB=0b00001111;		
	//saidas
	TRISD=0b00000000;
	PORTB=0b10000000;
}

//variaveis globais
int acertos;
unsigned char apontador[14];

void incializacao()
{
	acertos=0;
	apontador[acertos]=1+(rand()%4);
}

//funcoes
void mostra()
{
	int i=0;
	while(i<=acertos)
	{
		if (apontador[i]==1)
			PORTD=0b00000110;
		else if (apontador[i]==2)
			PORTD=0b01011011;
		else if (apontador[i]==3)
			PORTD=0b01001111;
		else
			PORTD=0b01100110;
		
		Delay10KTCYx(200);
		i++;
	}
}

//main loop
void main()
{
	io();
	incializacao();
	mostra();
	while(acertos==0){}
}