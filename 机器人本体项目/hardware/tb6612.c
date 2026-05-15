#include "stm32f10x.h"  
#include "pca9685.h"

// TB6612引脚定义 - 使用GPIOA和GPIOB的可用引脚（避开B8,B9）
#define TB_AIN_1_PORT    GPIOB
#define TB_AIN_1_PIN     GPIO_Pin_7
#define TB_BIN_1_PORT    GPIOB
#define TB_BIN_1_PIN     GPIO_Pin_6

#define TB_AIN_2_PORT    GPIOB
#define TB_AIN_2_PIN     GPIO_Pin_1
#define TB_BIN_2_PORT    GPIOB
#define TB_BIN_2_PIN     GPIO_Pin_0

#define TB_AIN_3_PORT    GPIOB
#define TB_AIN_3_PIN     GPIO_Pin_12
#define TB_BIN_3_PORT    GPIOB
#define TB_BIN_3_PIN     GPIO_Pin_11

#define TB_AIN_4_PORT    GPIOA
#define TB_AIN_4_PIN     GPIO_Pin_11
#define TB_BIN_4_PORT    GPIOA
#define TB_BIN_4_PIN     GPIO_Pin_8

#define TB_AIN_5_PORT    GPIOB
#define TB_AIN_5_PIN     GPIO_Pin_15
#define TB_BIN_5_PORT    GPIOB
#define TB_BIN_5_PIN     GPIO_Pin_14

// TB6612控制模式定义
#define TB_MODE_BRAKE    1  // 刹车
#define TB_MODE_FORWARD  2  // 正转
#define TB_MODE_BACKWARD 3  // 反转  
#define TB_MODE_STOP     4  // 停止



void tb6612_control(int choose, int mode, int speed)
{
    uint16_t pin_a, pin_b;
    GPIO_TypeDef * port_a;
    GPIO_TypeDef * port_b;
    uint8_t channel;
    
    // 参数验证
    if(choose < 1 || choose > 5 || speed < 0 || speed > 4096) {
        return; // 参数错误，直接返回
    }
    
    // 根据选择的通道配置引脚和PCA9685通道
    switch(choose) {
        case 1:
            pin_a = TB_AIN_1_PIN;
            pin_b = TB_BIN_1_PIN;
            port_a = TB_AIN_1_PORT;
            port_b = TB_BIN_1_PORT;
            channel = 4;  // PCA9685通道4
            break;
            
        case 2:
            pin_a = TB_AIN_2_PIN;
            pin_b = TB_BIN_2_PIN;
            port_a = TB_AIN_2_PORT;
            port_b = TB_BIN_2_PORT;
            channel = 5;  // PCA9685通道5
            break;
            
        case 3:
            pin_a = TB_AIN_3_PIN;
            pin_b = TB_BIN_3_PIN;
            port_a = TB_AIN_3_PORT;
            port_b = TB_BIN_3_PORT;
            channel = 6;  // PCA9685通道6
            break;
            
        case 4:
            pin_a = TB_AIN_4_PIN;
            pin_b = TB_BIN_4_PIN;
            port_a = TB_AIN_4_PORT;
            port_b = TB_BIN_4_PORT;
            channel = 7;  // PCA9685通道7
            break;
            
        case 5:
            pin_a = TB_AIN_5_PIN;
            pin_b = TB_BIN_5_PIN;
            port_a = TB_AIN_5_PORT;
            port_b = TB_BIN_5_PORT;
            channel = 14 ;  // PCA9685通道8
            break;
            
        default:
            return; // 无效通道
    }
    
    // 根据模式设置AIN和BIN引脚状态
    switch(mode) {
        case TB_MODE_BRAKE:    // 刹车：AIN=1, BIN=1
            GPIO_SetBits(port_a, pin_a);
            GPIO_SetBits(port_b, pin_b);
            PCA9685_SetDuty(channel, 0); // PWM输出为0
            break;
            
        case TB_MODE_FORWARD:  // 正转：AIN=1, BIN=0
            GPIO_SetBits(port_a, pin_a);
            GPIO_ResetBits(port_b, pin_b);
            PCA9685_SetDuty(channel, speed);
            break;
            
        case TB_MODE_BACKWARD: // 反转：AIN=0, BIN=1
            GPIO_ResetBits(port_a, pin_a);
            GPIO_SetBits(port_b, pin_b);
            PCA9685_SetDuty(channel, speed);
            break;
            
        case TB_MODE_STOP:     // 停止：AIN=0, BIN=0
            GPIO_ResetBits(port_a, pin_a);
            GPIO_ResetBits(port_b, pin_b);
            PCA9685_SetDuty(channel, 0); // PWM输出为0
            break;
            
        default:
            // 默认设置为停止状态
            GPIO_ResetBits(port_a, pin_a);
            GPIO_ResetBits(port_b, pin_b);
            PCA9685_SetDuty(channel, 0);
            break;
    }
}

void tb6612_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 使能GPIOA和GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置所有控制引脚为推挽输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 初始化第1组引脚
    GPIO_InitStruct.GPIO_Pin = TB_AIN_1_PIN | TB_BIN_1_PIN;
    GPIO_Init(TB_AIN_1_PORT, &GPIO_InitStruct);
    
    // 初始化第2组引脚
    GPIO_InitStruct.GPIO_Pin = TB_AIN_2_PIN | TB_BIN_2_PIN;
    GPIO_Init(TB_AIN_2_PORT, &GPIO_InitStruct);
    
    // 初始化第3组引脚
    GPIO_InitStruct.GPIO_Pin = TB_AIN_3_PIN | TB_BIN_3_PIN;
    GPIO_Init(TB_AIN_3_PORT, &GPIO_InitStruct);
    
    // 初始化第4组引脚
    GPIO_InitStruct.GPIO_Pin = TB_AIN_4_PIN | TB_BIN_4_PIN;
    GPIO_Init(TB_AIN_4_PORT, &GPIO_InitStruct);
    
    // 初始化第5组引脚
    GPIO_InitStruct.GPIO_Pin = TB_AIN_5_PIN | TB_BIN_5_PIN;
    GPIO_Init(TB_AIN_5_PORT, &GPIO_InitStruct);
    
    // 默认设置为停止状态
    tb6612_control(1, TB_MODE_STOP, 0);
    tb6612_control(2, TB_MODE_STOP, 0);
    tb6612_control(3, TB_MODE_STOP, 0);
    tb6612_control(4, TB_MODE_STOP, 0);
    tb6612_control(5, TB_MODE_STOP, 0);
}
