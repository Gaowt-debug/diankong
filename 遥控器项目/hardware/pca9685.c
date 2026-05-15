#include "stm32f10x.h"

/* I2C引脚定义（A2=SCL，A3=SDA） */
#define PCA_SCL_PORT    GPIOA
#define PCA_SCL_PIN     GPIO_Pin_2
#define PCA_SDA_PORT    GPIOA
#define PCA_SDA_PIN     GPIO_Pin_3

/* PCA9685地址与寄存器 */
#define PCA9685_ADDR    0x40    // 默认I2C地址（7位）
#define MODE1_REG       0x00    // 模式寄存器1
#define PRE_SCALE_REG   0xFE    // 预分频寄存器
#define LED0_ON_L       0x06    // 通道0输出ON低字节
#define LED0_ON_H       0x07    // 通道0输出ON高字节
#define LED0_OFF_L      0x08    // 通道0输出OFF低字节
#define LED0_OFF_H      0x09    // 通道0输出OFF高字节

/* 位操作宏 */
#define PCA_SCL_HIGH    GPIO_SetBits(PCA_SCL_PORT, PCA_SCL_PIN)
#define PCA_SCL_LOW     GPIO_ResetBits(PCA_SCL_PORT, PCA_SCL_PIN)
#define PCA_SDA_HIGH    GPIO_SetBits(PCA_SDA_PORT, PCA_SDA_PIN)
#define PCA_SDA_LOW     GPIO_ResetBits(PCA_SDA_PORT, PCA_SDA_PIN)
#define PCA_SDA_READ    GPIO_ReadInputDataBit(PCA_SDA_PORT, PCA_SDA_PIN)

/* 延时函数（微秒级，需根据系统时钟校准） */
void PCA_DelayUs(uint32_t us) {
    uint32_t i;
    for (i = 0; i < us * 8; i++); // 假设系统时钟72MHz，需调整系数
}
/* I2C起始信号 */
void PCA_I2C_Start(void) {
    PCA_SDA_HIGH;
    PCA_SCL_HIGH;
    PCA_DelayUs(5);
    PCA_SDA_LOW;  // SDA拉低表示开始
    PCA_DelayUs(5);
    PCA_SCL_LOW;  // SCL拉低，准备发送数据
}

/* I2C停止信号 */
void PCA_I2C_Stop(void) {
    PCA_SDA_LOW;
    PCA_SCL_HIGH;
    PCA_DelayUs(5);
    PCA_SDA_HIGH;  // SDA拉高表示停止
    PCA_DelayUs(5);
}

/* I2C发送应答信号（0=应答，1=非应答） */
void PCA_I2C_Ack(uint8_t ack) {
    if (ack) PCA_SDA_HIGH;
    else PCA_SDA_LOW;
    PCA_SCL_HIGH;
    PCA_DelayUs(5);
    PCA_SCL_LOW;
    PCA_SDA_HIGH; // 释放SDA
}

/* I2C等待应答 */
uint8_t PCA_I2C_WaitAck(void) {
    uint8_t timeout = 0;
    PCA_SDA_HIGH; // 释放SDA，等待从机应答
    PCA_SCL_HIGH;
    PCA_DelayUs(1);
    while (PCA_SDA_READ) { // 若SDA为高，表示未应答
        timeout++;
        if (timeout > 250) {
            PCA_I2C_Stop();
            return 1; // 超时无应答
        }
    }
    PCA_SCL_LOW; // 应答成功，拉低SCL
    return 0;
}

/* I2C发送一个字节 */
void PCA_I2C_SendByte(uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) PCA_SDA_HIGH; // 高位先发送
        else PCA_SDA_LOW;
        data <<= 1;
        PCA_SCL_HIGH;
        PCA_DelayUs(5);
        PCA_SCL_LOW;
    }
    PCA_I2C_WaitAck(); // 等待从机应答
}

/* I2C读取一个字节（ack=0：发送应答，ack=1：发送非应答） */
uint8_t PCA_I2C_ReadByte(uint8_t ack) {
    uint8_t i, data = 0;
    PCA_SDA_HIGH; // 释放SDA
    for (i = 0; i < 8; i++) {
        data <<= 1;
        PCA_SCL_HIGH;
        PCA_DelayUs(5);
        if (PCA_SDA_READ) data |= 0x01; // 读取SDA数据
        PCA_SCL_LOW;
    }
    PCA_I2C_Ack(ack); // 发送应答信号
    return data;
}
/* 向PCA9685写入数据 */
void PCA9685_WriteReg(uint8_t reg, uint8_t data) {
    PCA_I2C_Start();
    PCA_I2C_SendByte(PCA9685_ADDR << 1); // 写地址（7位地址左移1位，最低位0）
    PCA_I2C_SendByte(reg);               // 寄存器地址
    PCA_I2C_SendByte(data);              // 数据
    PCA_I2C_Stop();
}

/* 从PCA9685读取数据 */
uint8_t PCA9685_ReadReg(uint8_t reg) {
    uint8_t data;
    PCA_I2C_Start();
    PCA_I2C_SendByte(PCA9685_ADDR << 1);  // 写地址
    PCA_I2C_SendByte(reg);                // 寄存器地址
    PCA_I2C_Start();
    PCA_I2C_SendByte((PCA9685_ADDR << 1) | 0x01); // 读地址（最低位1）
    data = PCA_I2C_ReadByte(1);           // 读取数据，发送非应答
    PCA_I2C_Stop();
    return data;
}

/* PCA9685初始化（设置PWM频率为50Hz） */
void PCA9685_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* 初始化A2（SCL）和A3（SDA）为开漏输出 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD; // 开漏输出（I2C标准）
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = PCA_SCL_PIN | PCA_SDA_PIN;
    GPIO_Init(PCA_SCL_PORT, &GPIO_InitStruct);
    
    /* 初始状态：SCL和SDA拉高 */
    PCA_SCL_HIGH;
    PCA_SDA_HIGH;
    PCA_DelayUs(100);
    
    /* 进入睡眠模式，设置预分频器（50Hz） */
    PCA9685_WriteReg(MODE1_REG, 0x10); // 睡眠模式（BIT4=1）
    PCA_DelayUs(50);
    // 预分频值计算：PRE_SCALE = (25MHz / (4096 * 50Hz)) - 1 ≈ 121 = 0x79
    PCA9685_WriteReg(PRE_SCALE_REG, 0x79); 
    
    /* 退出睡眠模式，开启自动递增 */
    PCA9685_WriteReg(MODE1_REG, 0xA0); // 0xA0 = 1010 0000（BIT7=1：自动递增，BIT4=0：正常模式）
    PCA_DelayUs(50);
}

/* 设置通道PWM占空比（0~4095对应0%~100%） */
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    PCA_I2C_Start();
    PCA_I2C_SendByte(PCA9685_ADDR << 1);
    PCA_I2C_SendByte(LED0_ON_L + 4 * channel); // 通道起始地址（每个通道占4字节）
    PCA_I2C_SendByte(on & 0xFF);               // ON低字节
    PCA_I2C_SendByte(on >> 8);                 // ON高字节
    PCA_I2C_SendByte(off & 0xFF);              // OFF低字节
    PCA_I2C_SendByte(off >> 8);                // OFF高字节
    PCA_I2C_Stop();
}

/* 简化PWM设置（直接设置占空比，0~4095） */
void PCA9685_SetDuty(uint8_t channel, uint16_t duty) {
    if (duty >= 4095) PCA9685_SetPWM(channel, 0, 4096); // 全亮
    else if (duty == 0) PCA9685_SetPWM(channel, 0, 0);   // 全灭
    else PCA9685_SetPWM(channel, 0, duty);              // 正常PWM
}

