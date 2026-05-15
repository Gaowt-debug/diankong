#include "stm32f10x.h"  

void other_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_Initstructure;
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	
	
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
}


void other_take_control(char in)
{
	if(in==1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
	}
	else
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		
	}
	
}

void other_light_control(char in)
{
	if(in==1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_7);
	}
	else
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
		
	}
	
}

unsigned char other_trace_read(unsigned char choose)
{
	if(choose>=3)
	{
	return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3<<(choose-1));
	}
	else
	{
		return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12<<(choose-1));
	}
}	
