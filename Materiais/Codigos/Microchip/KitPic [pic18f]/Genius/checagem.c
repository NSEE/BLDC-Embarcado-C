//checagem

int checa()
{
	int i;
	for (i=0;i<=acertos;i++)
	{
		while(tecla[i]==0)
		{
			tecla[i]=botao();
		}
		Delay10KTCYx(100);
		if (tecla[i]==apontador[i])
		{}
		else
			return 1;	
	}
	return 0;
}

void main()
{
	fim=checa();
	acertos++;
}
