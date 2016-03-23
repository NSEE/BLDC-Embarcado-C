//parte_entrada

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

void io();
int tecla,botao();


void io()
{
	//entradas
	TRISB=0b00001111;		
	//saidas
	TRISD=0b00000000;
	PORTB=0b10000000;
}

int botao()
{
	if (botao1==0)
	{
		Delay10KTCYx(10);
		if (botao1==0)
			return 1;
	}
	else if (botao2==0)
	{
		Delay10KTCYx(10);
		if (botao2==0)
			return 2;
	}
	else if (botao3==0)
	{
		Delay10KTCYx(10);
		if (botao1==0)
			return 3;
	}
	else if (botao4==0)
	{
		Delay10KTCYx(10);
		if (botao4==0)
			return 4;
	}
	else
		return 0;
}

void main()
{
	io();
	while(tecla==0)
	{
		tecla=botao();
	}
}

