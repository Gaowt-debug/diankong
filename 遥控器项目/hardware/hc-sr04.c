#include "stm32f10x.h"
#include "Delay.h" 


//TIM2 GPIO:A0-echo A1-trig
void HC_SR04_INIT(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//启用时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	

	
	TIM_InternalClockConfig(TIM2);//选择内部时钟
	
	TIM_TimeBaseInitTypeDef TIM2_Initstructure;
	TIM2_Initstructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM2_Initstructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_Initstructure.TIM_Period=65536-1;//pwm计数器最大值（-1）
	TIM2_Initstructure.TIM_Prescaler=432-1;//频率=时钟频率/pd/pr（0—65535）
	TIM2_Initstructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2_Initstructure);//时基单元配置
	
	
	
	TIM_ICInitTypeDef TIM_IC_Initstructure;
	TIM_IC_Initstructure.TIM_Channel=TIM_Channel_1;
	TIM_IC_Initstructure.TIM_ICFilter=0xf;
	TIM_IC_Initstructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_IC_Initstructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_IC_Initstructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2,&TIM_IC_Initstructure);//输入配置
	TIM_PWMIConfig(TIM2,&TIM_IC_Initstructure);
	
	TIM_SelectInputTrigger(TIM2,TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Reset);
	
	TIM_Cmd(TIM2,ENABLE);
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	
}

void HC_SR04_GET(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	Delay_us(13);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}

int HC_SR04_READ(void)
{
	return TIM_GetCapture2(TIM2);
}