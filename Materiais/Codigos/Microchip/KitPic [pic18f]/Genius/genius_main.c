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
int apontador[14], acertos, tecla[14], botao(), checa(),fim;
void mostra(),inicializacao(),io(),limpa_tecla();

void inicializacao()
{
	int i;
	for(i=0;i<=14;i++)
	{
		apontador[i]=1+(rand()%4);
		tecla[i]=0;
	}
	fim=0;
	acertos=0;
}

//funcoes
void mostra()
{
	// declarando variavel auxiliar para contar quantidade de acerto
	int i;
	i=0;
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
	PORTD=0b10000000;
}

int botao()
{
	if (botao1==0)
	{
		return 1;
	}
	else if (botao2==0)
	{
		return 2;
	}
	else if (botao3==0)
	{
		return 3;
	}
	else if (botao4==0)
	{
		return 4;
	}
	else
		return 0;
}

int checa()
{
	int i;
	i=0;
	while(i<=acertos)
	{
		while(tecla[i]==0)
		{
			tecla[i]=botao();
			if (tecla[i]!=0)
				break;
		}
		PORTD=0b01000000;
		Delay10KTCYx(100);
		if (tecla[i]==apontador[i])
		{}
		else
			return 1;
		
		i++;
	}
	return 0;
}

void limpa_tecla()
{
	int i;
	for(i=0;i<=14;i++)
	{
		tecla[i]=0;
	}
}

//main loop
void main()
{
	int h;
	io();
	inicializacao();
	while((fim==0)||(acertos!=0))
	{	
		limpa_tecla();
		mostra();
		fim=checa();
		if (fim==1)
			break;
		acertos++;
		Delay10KTCYx(100);
	}
	for(h=0;h<5;h++)
	{
		PORTD=0b10000000;
		Delay10KTCYx(10);
		PORTD=0b00000000;
		Delay10KTCYx(10);
	}
}





