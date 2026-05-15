#include "stm32f10x.h"  




//usart1 txd-A9 rxd-A10
void USART_INIT()//tx a9 rx a10
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_Initstructure;
	NVIC_Initstructure.NVIC_IRQChannel=USART1_IRQn;//通道
	NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_Initstructure.NVIC_IRQChannelSubPriority=1;//优先级
	NVIC_Init(&NVIC_Initstructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 启用接收中断
	
	USART_InitTypeDef usart_Initstructure;
	
	usart_Initstructure.USART_BaudRate=9600;
	usart_Initstructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	usart_Initstructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	usart_Initstructure.USART_Parity=USART_Parity_No;
	usart_Initstructure.USART_StopBits=USART_StopBits_1;
	usart_Initstructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&usart_Initstructure);
	
	USART_Cmd(USART1,ENABLE);
}

void USART2_INIT(void)//tx a2 rx a3
{
    // 1. 使能时钟（USART2在APB1总线上）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // USART2同样使用GPIOA
    
    GPIO_InitTypeDef GPIO_Initstructure;
    
    // 2. 配置TX引脚(PA2)为复用推挽输出
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_2; // USART2_TX
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Initstructure);
    
    // 3. 配置RX引脚(PA3)为上拉输入
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_3; // USART2_RX
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Initstructure);
    
    // 4. 配置中断（如果需要）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_Initstructure;
    NVIC_Initstructure.NVIC_IRQChannel = USART2_IRQn; // USART2中断通道
    NVIC_Initstructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Initstructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_Initstructure);
    
    // 5. 启用USART2接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    // 6. 配置USART2参数
    USART_InitTypeDef usart_Initstructure;
    
    usart_Initstructure.USART_BaudRate = 9600;
    usart_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_Initstructure.USART_Parity = USART_Parity_No;
    usart_Initstructure.USART_StopBits = USART_StopBits_1;
    usart_Initstructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &usart_Initstructure);
    
    // 7. 使能USART2
    USART_Cmd(USART2, ENABLE);
}


void USART_sendbit(uint8_t data)
{
	USART_SendData(USART1,data);
	while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	
}

void USART_sendstring(char* data)
{
	uint8_t i;
	for(i=0;data[i] !='\0';i++)
	{
		USART_sendbit(data[i]);
	}
	
}



//void USART1_IRQHandler()
//{
//	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
//	{
//		
//		
//		
//		
//	}
//}


//void USART2_IRQHandler(void)
//{
//    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//    {
//        
//        uint8_t data = USART_ReceiveData(USART2);
//        
//
//    }
//}



	
