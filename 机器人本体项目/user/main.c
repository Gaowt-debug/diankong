#include "stm32f10x.h"                  // Device header
#include "Delay.h"  
#include "OLED.h"
#include "hc-sr04.h"
#include "usart.h"
#include "pca9685.h"
#include "other.h"
#include "tb6612.h"
#include "math.h"


#define PI_f 3.141592f

int count=0;
int timer_s_count=0;
int timer_tms_count=0;

char take_change_swi=1;

/* ============================================================
   ★ 云台追踪模块 —— 重写版
   策略：卡尔曼滤波 + 圆周运动参数在线拟合 + 超前预测补偿
   ============================================================ */

/* ---------- 数据包 & 基础状态 ---------- */
char  pack_state = 0;
int   pack_sx = 0, pack_sy = 0;   /* 1280*720 压缩到 640*480 后传回 */
int   pack_cx = 0, pack_cy = 0;
int   pack_r  = 0;
char  hit_wait = 0;                /* 2s 看门狗 */

/* ---------- 云台 PWM 位置（1000 倍定点，原接口不变） ---------- */
int   hit_gml_x  = 450000;
int   hit_gml_y  = 180000;
float hit_gmln_x = 450.0f;        /* x: 300~600  y: 120~240 */
float hit_gmln_y = 180.0f;
int   hit_gml_c_x = 0;
int   hit_gml_c_y = 0;

/* ---------- 步骤 & 扫描 ---------- */
char  hit_step         = 1;       /* 1找圆心 2锁定 3标定K 4追踪 */
int   hit_stable_count = 0;
int   hit_find_count   = 1;
char  hit_testlist     = 0;
char  hit_move_wait    = 0;
char  hit_ready        = 0;
int   hit_ept_count    = 1;
int   hit_t     = 0;
int   hit_t_la  = 0;

/* ---------- 像素坐标（以图像中心为原点，右+上+） ---------- */
int   hit_xx = 0, hit_xy = 0;
int   hit_xx_la = 0, hit_xy_la = 0;

/* ---------- 标定参数 ---------- */
int   hit_c_x = 0, hit_c_y = 0;  /* 圆周中心像素坐标 */
float hit_big_r = 0.0f;           /* 圆周轨迹半径（像素） */
float hit_Kx    = 1.0f;           /* 像素→云台PWM比例 x */
float hit_Ky    = 1.0f;           /*                   y */
int   hit_test_x = 35;
int   hit_test_y = 20;

/* ============================================================
   卡尔曼滤波器（2阶：位置+速度，x/y 各一套）
   状态: [位置, 速度]，过程噪声 Q，观测噪声 R
   ============================================================ */
#define KF_Q_POS  5.0f
#define KF_Q_VEL  20.0f
#define KF_R_POS  25.0f

typedef struct {
    float x;                         /* 位置估计（像素） */
    float v;                         /* 速度估计（像素/ms） */
    float p00, p01, p10, p11;        /* 2x2 协方差矩阵 */
} KalmanState;

KalmanState kf_x = {0, 0, 100, 0, 0, 100};
KalmanState kf_y = {0, 0, 100, 0, 0, 100};

/**
 * 卡尔曼预测 + 更新
 * @param kf  滤波器状态
 * @param z   观测值（像素）
 * @param dt  时间步长（ms）
 */
void KF_Update(KalmanState *kf, float z, float dt)
{
    /* --- 预测步 --- */
    float x_p   = kf->x + kf->v * dt;
    float v_p   = kf->v;
    float p00_p = kf->p00 + dt*(kf->p10 + kf->p01) + dt*dt*kf->p11 + KF_Q_POS;
    float p01_p = kf->p01 + dt*kf->p11;
    float p10_p = kf->p10 + dt*kf->p11;
    float p11_p = kf->p11 + KF_Q_VEL;

    /* --- 更新步 --- */
    float S  = p00_p + KF_R_POS;
    float k0 = p00_p / S;
    float k1 = p10_p / S;
    float innov = z - x_p;

    kf->x   = x_p + k0 * innov;
    kf->v   = v_p + k1 * innov;
    kf->p00 = (1.0f - k0) * p00_p;
    kf->p01 = (1.0f - k0) * p01_p;
    kf->p10 = p10_p - k1 * p00_p;
    kf->p11 = p11_p - k1 * p01_p;
}

/* ============================================================
   圆周运动模型
   小机关：ω = π/3 ≈ 1.047 rad/s（匀速）
   大机关：ω(t) = a·sin(w·t + φ) + b，b = 2.090 - a
           a ∈ [0.780, 1.045]，w ∈ [1.884, 2.000]
   ============================================================ */

/* 圆周运动状态 */
float circ_theta    = 0.0f;        /* 当前目标角度（rad） */
float circ_omega_ms = 0.0f;        /* 平滑角速度（rad/ms） */

/* 大机关正弦拟合参数 */
float sinfit_a   = 0.9f;
float sinfit_w   = 1.942f;         /* rad/s */
float sinfit_phi = 0.0f;
float sinfit_t0  = 0.0f;           /* 拟合起始时间（ms） */

/* 能量机关模式：0=未知  1=小机关(匀速)  2=大机关(正弦) */
char  energy_mode = 0;

/* 角速度历史缓冲，用于模式判断（方差检测） */
#define OMEGA_BUF_SIZE 32
float omega_buf[OMEGA_BUF_SIZE];
int   omega_buf_idx = 0;
int   omega_buf_cnt = 0;


/* ======================== 新增：遥控器角度数据包接收 ======================== */
// 解析状态：0=普通遥控模式 1=角度包解析模式
volatile uint8_t agl_pack_state = 0;
// 解析完的4个角度值，你的其他代码直接读这个数组用就行！
volatile int remote_agl[4] = {0};
// 解析临时变量
volatile int agl_tmp_num = 0;
volatile uint8_t agl_tmp_idx = 0;

/* 超前预测时间（ms）= 视觉延迟 + 通信延迟 + 云台响应时间 */
#define PREDICT_LEAD_MS  20.0f

/* ---------- 前馈 + 残差 PID 控制器 ---------- */
/*
 * 架构：云台目标位置 = 前馈（模型预测位置） + PID修正（残差）
 * 前馈占比大 → 跟得快，不依赖PID增益 → 不抖
 * PID只修正模型误差/标定误差 → 增益可以小 → 不抖
 *
 * 关键：前馈值经过低通滤波平滑，避免视觉噪声导致云台阶跃跳变
 *
 * 残差 PID 增益（仅修正前馈残余误差，很小即可）
 */
float gml_Kp = 0.008f;    /* 残差比例 */
float gml_Kd = 0.000f;    /* 残差微分 */
float gml_Ki = 0.001f;    /* 残差积分 */

float gml_err_x_la = 0.0f;
float gml_err_y_la = 0.0f;
float gml_ix       = 0.0f;
float gml_iy       = 0.0f;
#define GML_I_MAX 3.0f

/* 前馈低通滤波状态 */
float ff_gml_smooth_x = 0.0f;   /* 平滑后的前馈云台PWM x */
float ff_gml_smooth_y = 0.0f;   /* 平滑后的前馈云台PWM y */
#define FF_SMOOTH_ALPHA  0.12f   /* 平滑系数：0=不动 1=不平滑，值越小越平滑 */

char  hit_dir = 0;                 /* 保留原变量，供外部使用 */

/* ============================================================
   以下原始变量保留，供 move/eat/take 模块使用（不修改）
   ============================================================ */
char  move_mode    = 0;
char  move_mode_la = 1;
char  eat_mode     = 3;
char  eat_mode_la  = 0;
char  eat_pull_mode    = 0;
char  eat_pull_mode_la = 1;
char  take_switch  = 0;
char  take_mode    = 0;
char  take_revolve    = 0;
char  take_revolve_la = 1;
int take_dir[4]    = {0, 120, 100, 600};
int take_dir_la[4] = {1, 121, 101, 599};

/* ============================================================
   函数声明
   ============================================================ */
void hit_gml(char choose, int dir);
void hit_step_start(char step);
void gml_find(void);
void KF_Update(KalmanState *kf, float z, float dt);

/* ============================================================
   外设初始化（与原代码相同，不修改）
   ============================================================ */
void interrupt_init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Initstructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Initstructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_Pin_0);

    EXTI_InitTypeDef EXTI_Initstructure;
    EXTI_Initstructure.EXTI_Line    = EXTI_Line0;
    EXTI_Initstructure.EXTI_LineCmd = ENABLE;
    EXTI_Initstructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_Initstructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_Initstructure;
    NVIC_Initstructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Initstructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_Initstructure);
}

void timer_init()   /* 1 ms 节拍 */
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitTypeDef TIM_Initstructure;
    TIM_Initstructure.TIM_ClockDivision  = TIM_CKD_DIV1;
    TIM_Initstructure.TIM_CounterMode    = TIM_CounterMode_Up;
    TIM_Initstructure.TIM_Period         = 100 - 1;
    TIM_Initstructure.TIM_Prescaler      = 720 - 1;
    TIM_Initstructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_Initstructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_Initstructure;
    NVIC_Initstructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_Initstructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_Initstructure);

    TIM_Cmd(TIM3, ENABLE);
}

void timer_pwm_init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Initstructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Initstructure);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM2_Initstructure;
    TIM2_Initstructure.TIM_ClockDivision    = TIM_CKD_DIV1;
    TIM2_Initstructure.TIM_CounterMode      = TIM_CounterMode_Up;
    TIM2_Initstructure.TIM_Period           = 100 - 1;
    TIM2_Initstructure.TIM_Prescaler        = 720 - 1;
    TIM2_Initstructure.TIM_RepetitionCounter= 0;
    TIM_TimeBaseInit(TIM2, &TIM2_Initstructure);

    TIM_OCInitTypeDef TIM2_OCInitstructure;
    TIM_OCStructInit(&TIM2_OCInitstructure);
    TIM2_OCInitstructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM2_OCInitstructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM2_OCInitstructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM2_OCInitstructure.TIM_Pulse       = 0;
    TIM_OC1Init(TIM2, &TIM2_OCInitstructure);

    TIM_Cmd(TIM2, ENABLE);
}

void Laser_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);   /* 初始关闭 */
}

void Laser_On(void)  { GPIO_ResetBits(GPIOA, GPIO_Pin_15); }
void Laser_Off(void) { GPIO_SetBits(GPIOA, GPIO_Pin_15);   }

/* ============================================================
   云台舵机控制（原接口不变）
   choose=0: 水平轴(通道0) PWM 300~600
   choose=1: 俯仰轴(通道2) PWM 120~240（dir+120 映射到 223~383）
   ============================================================ */
void hit_gml(char choose, int dir)
{
    if (choose == 1 && dir+120 >= 223 && dir+120 <= 383)
    {
        PCA9685_SetDuty(2, dir);
    }
    if (choose == 0 && dir >= 300 && dir <= 600)
    {
        PCA9685_SetDuty(0, dir);
    }
}

/* ============================================================
   螺旋扫描（原代码完整保留，不修改）
   ============================================================ */
void gml_find()
{
    if(hit_find_count==0)
    {
        hit_gml_x=425000;
        hit_gml_y=180000;
    }
    else if(hit_find_count<2)
    {
        hit_gml_x=425000+2500*(hit_find_count);
        hit_gml_y=180000;
    }
    else if(hit_find_count<3)
    {
        hit_gml_x=427500;
        hit_gml_y=180000+6000*(hit_find_count-1);
    }
    else if(hit_find_count<5)
    {
        hit_gml_x=427500-2500*(hit_find_count-2);
        hit_gml_y=186000;
    }
    else if(hit_find_count<7)
    {
        hit_gml_x=422500;
        hit_gml_y=186000-6000*(hit_find_count-4);
    }
    else if(hit_find_count<10)
    {
        hit_gml_x=422500+2500*(hit_find_count-6);
        hit_gml_y=174000;
    }
    else if(hit_find_count<13)
    {
        hit_gml_x=430000;
        hit_gml_y=174000+6000*(hit_find_count-9);
    }
    else if(hit_find_count<17)
    {
        hit_gml_x=430000-2500*(hit_find_count-12);
        hit_gml_y=192000;
    }
    else if(hit_find_count<21)
    {
        hit_gml_x=420000;
        hit_gml_y=192000-6000*(hit_find_count-16);
    }
    else if(hit_find_count<26)
    {
        hit_gml_x=420000+2500*(hit_find_count-20);
        hit_gml_y=168000;
    }
    else if(hit_find_count<31)
    {
        hit_gml_x=432500;
        hit_gml_y=168000+6000*(hit_find_count-25);
    }
    else if(hit_find_count<37)
    {
        hit_gml_x=432500-2500*(hit_find_count-30);
        hit_gml_y=198000;
    }
    else if(hit_find_count<43)
    {
        hit_gml_x=417500;
        hit_gml_y=198000-6000*(hit_find_count-36);
    }
    else if(hit_find_count<50)
    {
        hit_gml_x=417500+2500*(hit_find_count-42);
        hit_gml_y=162000;
    }
    else if(hit_find_count<57)
    {
        hit_gml_x=435000;
        hit_gml_y=162000+6000*(hit_find_count-49);
    }
    else if(hit_find_count<65)
    {
        hit_gml_x=435000-2500*(hit_find_count-56);
        hit_gml_y=204000;
    }
    else if(hit_find_count<73)
    {
        hit_gml_x=415000;
        hit_gml_y=204000-6000*(hit_find_count-64);
    }
    else if(hit_find_count<82)
    {
        hit_gml_x=415000+2500*(hit_find_count-72);
        hit_gml_y=156000;
    }
    else if(hit_find_count<91)
    {
        hit_gml_x=437500;
        hit_gml_y=156000+6000*(hit_find_count-81);
    }
    else if(hit_find_count<101)
    {
        hit_gml_x=437500-2500*(hit_find_count-90);
        hit_gml_y=210000;
    }
    else if(hit_find_count<111)
    {
        hit_gml_x=412500;
        hit_gml_y=210000-6000*(hit_find_count-100);
    }
    else if(hit_find_count<166)
    {
        hit_gml_x=412500+2500*(hit_find_count-110);
        hit_gml_y=120000;
    }
    else if(hit_find_count<266)
    {
        hit_gml_x=550000-2500*(hit_find_count-165);
        hit_gml_y=120000;
    }
    else
    {
        hit_find_count=0;
    }
    hit_find_count++;

    hit_gml(0, hit_gml_x/1000);
    hit_gml(1, hit_gml_y/1000);
}

/* ============================================================
   ★ hit_step_start —— 重写版
   ============================================================ */
void hit_step_start(char step)
{
    switch (step)
    {
        /* ---- Step1: 重置，螺旋扫描 ---- */
        case 1:
            Laser_Off();
            hit_find_count = 1;
            hit_step = 1;
            /* 重置卡尔曼 */
            kf_x.x=0; kf_x.v=0; kf_x.p00=100; kf_x.p11=100; kf_x.p01=0; kf_x.p10=0;
            kf_y.x=0; kf_y.v=0; kf_y.p00=100; kf_y.p11=100; kf_y.p01=0; kf_y.p10=0;
            /* 重置运动模型 */
            energy_mode   = 0;
            omega_buf_cnt = 0; omega_buf_idx = 0;
            circ_theta    = 0; circ_omega_ms = 0;
            sinfit_a=0.9f; sinfit_w=1.942f; sinfit_phi=0.0f;
            gml_ix=0; gml_iy=0;
            gml_err_x_la=0; gml_err_y_la=0;
            break;

        /* ---- Step2: 锁定圆心 ---- */
        case 2:
            hit_stable_count = 0;
            hit_gmln_x = hit_gml_x / 1000.0f;
            hit_gmln_y = hit_gml_y / 1000.0f;
            hit_step   = 2;
            break;

        /* ---- Step3: 标定 Kx/Ky ---- */
        case 3:
            hit_gml_c_x  = (int)hit_gmln_x;
            hit_gml_c_y  = (int)hit_gmln_y;
            hit_c_x      = hit_xx;
            hit_c_y      = hit_xy;
            hit_testlist = 0;
            hit_big_r    = 0.0f;
            hit_step     = 3;
            break;

        /* ---- Step4: 追踪 ---- */
        case 4:
            Laser_On();
            hit_step   = 4;
            hit_gmln_x = (float)hit_gml_c_x;
            hit_gmln_y = (float)hit_gml_c_y;
            /* 初始化前馈平滑为当前云台位置（避免启动跳变） */
            ff_gml_smooth_x = hit_gmln_x;
            ff_gml_smooth_y = hit_gmln_y;
            hit_t_la   = hit_t;
            /* 重置控制器 */
            gml_ix=0; gml_iy=0;
            gml_err_x_la=0; gml_err_y_la=0;
            /* 重置运动模型 */
            energy_mode   = 0;
            omega_buf_cnt = 0; omega_buf_idx = 0;
            circ_theta    = 0; circ_omega_ms = 0;
            sinfit_a=0.9f; sinfit_w=1.942f; sinfit_phi=0.0f;
            sinfit_t0 = (float)hit_t;
            break;
    }
}

/* ============================================================
   全局初始化（不修改）
   ============================================================ */
void all_init(void)
{
    OLED_Init();
    USART_INIT();
    USART2_INIT();
    timer_init();
    PCA9685_Init();
    other_init();
    tb6612_init();
    Delay_ms(3);
}

/* ============================================================
   主循环（不修改云台无关部分）
   ============================================================ */
int main(void)
{
    all_init();
    hit_gml(0, 450);
    PCA9685_SetDuty(2, 510);
    Laser_Init();

    while(1)
    {
			
			
			if(take_dir[0]!=take_dir_la[0])
			{
				if(take_dir[0]==1)
				{
					PCA9685_SetDuty(13, 400);
				}
				else if(take_dir[0]==2)
				{
					PCA9685_SetDuty(13, 320);
				}
				else if(take_dir[0]==0)
				{
					PCA9685_SetDuty(13, 360);
				}
				
				take_dir_la[0]=take_dir[0];
				
			}
			
			
			
        /* ---- 底盘移动 ---- */
        if(move_mode != move_mode_la)
        {
            if(move_mode==0)
            {
                tb6612_control(1,1,2000);
                tb6612_control(2,1,2000);
                tb6612_control(3,1,2000);
                tb6612_control(4,1,2000);
            }
            if(move_mode==7)
            {
                tb6612_control(1,2,3500);
                tb6612_control(2,3,3600);
                tb6612_control(3,3,3500);
                tb6612_control(4,2,3500);
            }
            if(move_mode==8)
            {
                tb6612_control(1,3,3500);
                tb6612_control(2,2,3500);
                tb6612_control(3,2,3500);
                tb6612_control(4,3,3500);
            }
            if(move_mode==3)
            {
                tb6612_control(1,3,3800);
                tb6612_control(2,3,3900);
                tb6612_control(3,3,3800);
                tb6612_control(4,3,3800);
            }
            if(move_mode==4)
            {
                tb6612_control(1,2,4000);
                tb6612_control(2,2,4000);
                tb6612_control(3,2,4000);
                tb6612_control(4,2,4000);
            }
            if(move_mode==5)
            {
                tb6612_control(1,2,2500);
                tb6612_control(2,3,2500);
                tb6612_control(3,2,2500);
                tb6612_control(4,3,2500);
            }
            if(move_mode==6)
            {
                tb6612_control(1,3,2500);
                tb6612_control(2,2,2500);
                tb6612_control(3,3,2500);
                tb6612_control(4,2,2500);
            }
            if(move_mode==1)
            {
                tb6612_control(1,2,2000);
                tb6612_control(2,3,2000);
                tb6612_control(3,3,2000);
                tb6612_control(4,2,2000);
            }
            if(move_mode==2)
            {
                tb6612_control(1,3,2000);
                tb6612_control(2,2,2000);
                tb6612_control(3,2,2000);
                tb6612_control(4,3,2000);
            }
            move_mode_la = move_mode;
        }

        /* ---- 吃球 ---- */
        if(eat_mode != eat_mode_la)
        {
            if(eat_mode==3) { tb6612_control(5,1,2000); }
            if(eat_mode==2) { tb6612_control(5,3,1000); }
            if(eat_mode==1) { tb6612_control(5,3,2000); }
            if(eat_mode==0) { tb6612_control(5,3,3000); }
            if(eat_mode==4) { tb6612_control(5,2,1000); }
            if(eat_mode==5) { tb6612_control(5,2,2000); }
            if(eat_mode==6) { tb6612_control(5,2,3000); }
            eat_mode_la = eat_mode;
        }

        /* ---- 取球舵机 ---- */
        

        if(take_revolve != take_revolve_la)
        {
            if(take_revolve==1) { PCA9685_SetDuty(8,310); }
            if(take_revolve==2) { PCA9685_SetDuty(8,410); }
            if(take_revolve==0) { PCA9685_SetDuty(8,360); }
						if(take_revolve==3) { PCA9685_SetDuty(8,270); }
            if(take_revolve==4) { PCA9685_SetDuty(8,450); }
            take_revolve_la = take_revolve;
        }

        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)==Bit_SET && eat_pull_mode==1)
        {
            eat_pull_mode = 0;
            PCA9685_SetDuty(3, 360);
        }

        if(eat_pull_mode != eat_pull_mode_la)
        {
            if(eat_pull_mode==1 && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)==0)
            {
                PCA9685_SetDuty(3, 120);
            }
            if(eat_pull_mode==2) { PCA9685_SetDuty(3, 600); }
            if(eat_pull_mode==0) { PCA9685_SetDuty(3, 360); }
            eat_pull_mode_la = eat_pull_mode;
        }
    }
}


/* ============================================================
   EXTI0 中断（保留原代码）
   ============================================================ */
void EXTI0_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}


/* ============================================================
   TIM3 中断 —— 1 ms 节拍
   ============================================================ */
void TIM3_IRQHandler()
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        hit_t++;

        /* ---- 20 ms 节拍：云台输出 ---- */
        if(timer_tms_count < 20)
        {
            timer_tms_count++;
        }
        else
        {
            timer_tms_count = 0;

            if(hit_wait > 0)
            {
                if(hit_step == 4)
                {
                    /*
                     * 追踪状态下，云台命令由 USART1_IRQHandler 实时计算
                     * 此处以 20 ms 节拍输出，保证舵机平滑
                     */
                    hit_gml(0, (int)hit_gmln_x);
                    hit_gml(1, (int)hit_gmln_y);
                }
            }

            /* 取球舵机缓动 */
            if(take_mode == 1)
            {
//                if(take_switch==1)  { take_dir[0]=1; }
                if(take_switch==2 && take_dir[1]<=594)  { take_dir[1]+=2; }
                if(take_switch==3 && take_dir[2]>=106)  { take_dir[2]-=2; }
                if(take_switch==4 && take_dir[3]>=300)  { take_dir[3]-=1; }
            }
            if(take_mode == 2)
            {
//                if(take_switch==1)  { take_dir[0]=2; }
                if(take_switch==2 && take_dir[1]>=106)  { take_dir[1]-=2; }
                if(take_switch==3 && take_dir[2]<=594)  { take_dir[2]+=2; }
                if(take_switch==4 && take_dir[3]<=594)  { take_dir[3]+=1; }
            }
//						+
							 
							 
							 
							 
							if(take_change_swi<3)
								take_change_swi++;
							else
								take_change_swi=0;
 /* 机械臂缓动 */
    if(take_dir_la[take_change_swi] != take_dir[take_change_swi])
    {
        int diff = take_dir[take_change_swi] - take_dir_la[take_change_swi];
        
        
        if(take_change_swi==3)
				{
					if(diff > 7)       diff =  7;
        else if(diff < -7) diff = -7 ;
					
				}
				else
				{
			if(diff > 12)       diff =  12;
        else if(diff < -12) diff = -12 ;
				}
			
        take_dir_la[take_change_swi] += diff;
			
        PCA9685_SetDuty(13-take_change_swi, take_dir_la[take_change_swi]);

        if(take_change_swi == 3) { Delay_us(100);PCA9685_SetDuty(9, take_dir_la[take_change_swi]); }
        
        
    }







        }

        /* ---- 1 s 节拍：hit_wait 看门狗 ---- */
        if(timer_s_count < 1000)
        {
            timer_s_count++;
        }
        else
        {
            timer_s_count = 0;
            if(hit_wait > 0) { hit_wait--; }
        }

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}


/* ============================================================
   USART1 中断 —— 接收视觉数据包
   数据格式（ASCII）：sx,sy,r,cx,cy\n
   ============================================================ */
void USART1_IRQHandler()
{
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
    {
        char data = USART_ReceiveData(USART1);

        /* ---- 看门狗超时 → 重置追踪状态 ---- */
        if(hit_wait == 0)
        {
            pack_sx=0; pack_sy=0; pack_r=0; pack_state=0;
            hit_xx=0; hit_xy=0; hit_xx_la=0; hit_xy_la=0;
            hit_step=1; hit_ept_count=1; hit_t=0;
            hit_find_count=1;
            /* 重置卡尔曼 */
            kf_x.x=0; kf_x.v=0; kf_x.p00=100; kf_x.p11=100; kf_x.p01=0; kf_x.p10=0;
            kf_y.x=0; kf_y.v=0; kf_y.p00=100; kf_y.p11=100; kf_y.p01=0; kf_y.p10=0;
            energy_mode=0;
            omega_buf_cnt=0; omega_buf_idx=0;
            circ_theta=0; circ_omega_ms=0;
            gml_ix=0; gml_iy=0;
            gml_err_x_la=0; gml_err_y_la=0;
        }

        OLED_ShowChar(1, 6, data);
        hit_wait = 2;

        /* ---- 数据包解析 ---- */
        if(data != 'A')
        {
            switch(pack_state)
            {
                case 0:
                    if(data==',')        { pack_state=1; }
                    else                 { pack_sx = pack_sx*10 + data-'0'; }
                    break;
                case 1:
                    if(data==',')        { pack_state=2; }
                    else                 { pack_sy = pack_sy*10 + data-'0'; }
                    break;
                case 2:
                    if(data==',')        { pack_state=3; }
                    else                 { pack_r  = pack_r *10 + data-'0'; }
                    break;
                case 3:
                    if(data==',')        { pack_state=4; }
                    else                 { pack_cx = pack_cx*10 + data-'0'; }
                    break;
                case 4:
                    if(data=='n')        { pack_state=5; }
                    else if(data!='\\')  { pack_cy = pack_cy*10 + data-'0'; }
                    break;
            }

            /* ---- 完整帧处理 ---- */
            if(pack_state == 5)
            {
                /* 目标像素坐标（图像中心为原点） */
                hit_xx = pack_sx - 320;
                hit_xy = 240 - pack_sy;

                switch(hit_step)
                {
                    /* ===================================================
                       Step 1：螺旋扫描，等待目标出现
                       =================================================== */
                    case 1:
                        /* 扫描逻辑在 hit_find_count!=0 分支处理 */
                        break;

                    /* ===================================================
                       Step 2：锁定圆周中心（利用圆心坐标 cx,cy 对准）
                       =================================================== */
                    case 2:
                    {
                        hit_xx = pack_cx - 320;
                        hit_xy = 240 - pack_cy;

                        if(pack_cx != 0)
                        {
                            /* 粗跟踪：将圆心移至图像中心 */
                            hit_gmln_x -= hit_xx / 100.0f;
                            hit_gmln_y += hit_xy / 100.0f;

                            /* 精调：1像素死区消除残差 */
                            if(hit_xx  > 12)  { hit_gmln_x -= 1.0f; }
                            if(hit_xx <= -12) { hit_gmln_x += 1.0f; }
                            if(hit_xy > 10 && hit_xy <=  100) { hit_gmln_y += 1.0f; }
                            if(hit_xy < -10 && hit_xy >= -100){ hit_gmln_y -= 1.0f; }
                        }

                        /* 稳定计数：连续 20 帧在死区内 → 进入标定 */
                        if(hit_xx < 26 && hit_xx > -26 && hit_xy < 24 && hit_xy > -24)
                        {
                            hit_stable_count++;
                            if(hit_stable_count >= 20)
                            {
                                hit_step_start(3);
                            }
                        }
                        else
                        {
                            hit_stable_count = 0;
                        }

                        hit_gml(0, (int)hit_gmln_x);
                        hit_gml(1, (int)hit_gmln_y);
                        break;
                    }

                    /* ===================================================
                       Step 3：标定 Kx/Ky（像素/PWM 比例）
                       ─ 先向 -y 方向移动 hit_test_y PWM 单位，测 Ky
                       ─ 再向 +x 方向移动 hit_test_x PWM 单位，测 Kx
                       =================================================== */
                    case 3:
                    {
                        hit_xx = pack_cx - 320;
                        hit_xy = 240 - pack_cy;

                        switch(hit_testlist)
                        {
                            /* 等待目标出现并测量圆周半径 */
                            case 0:
                                hit_xx = pack_sx - 320;
                                hit_xy = 240 - pack_sy;
                                if(pack_cx != 0)
                                {
                                    if(hit_big_r == 0.0f)
                                    {
                                        if(hit_xy-hit_c_y >= -50 && hit_xy-hit_c_y <= 50)
                                        {
                                            hit_big_r = sqrtf(
                                                (float)((hit_xx-hit_c_x)*(hit_xx-hit_c_x) +
                                                        (hit_xy-hit_c_y)*(hit_xy-hit_c_y)));
                                        }
                                    }
                                    else if(fabsf(hit_big_r - sqrtf(
                                                (float)((hit_xx-hit_c_x)*(hit_xx-hit_c_x) +
                                                        (hit_xy-hit_c_y)*(hit_xy-hit_c_y)))) <= 40.0f)
                                    {
                                        hit_testlist  = 1;
                                        hit_move_wait = 1;
                                        hit_gml(0, hit_gml_c_x);
                                        hit_gml(1, hit_gml_c_y - hit_test_y);
                                    }
                                    else
                                    {
                                        hit_big_r = 0.0f;
                                    }
                                }
                                break;

                            /* 等待 Ky 方向稳定 → 计算 Ky */
                            case 1:
                                if(hit_move_wait >= 6)
                                {
                                    hit_Ky = (float)(hit_c_y - hit_xy) / (float)hit_test_y;
                                    hit_testlist  = 3;
                                    hit_move_wait = 1;
                                    hit_gml(0, hit_gml_c_x + hit_test_x);
                                    hit_gml(1, hit_gml_c_y);
                                }
                                else { hit_move_wait++; }
                                break;

                            case 2:
                                if(hit_move_wait >= 20)
                                {
                                    hit_Ky = (float)(hit_xy - hit_c_y) / (float)hit_test_y;
                                    hit_testlist  = 3;
                                    hit_move_wait = 1;
                                    hit_gml(0, hit_gml_c_x + hit_test_x);
                                    hit_gml(1, hit_gml_c_y);
                                }
                                else { hit_move_wait++; }
                                break;

                            /* 等待 Kx 方向稳定 → 计算 Kx → 进入追踪 */
                            case 3:
                                if(hit_move_wait >= 20)
                                {
                                    hit_Kx = (float)(hit_c_x - hit_xx) / (float)hit_test_x;
                                    hit_step_start(4);
                                }
                                else { hit_move_wait++; }
                                break;

                            case 4:
                                if(hit_move_wait >= 20)
                                {
                                    hit_Kx = (float)(hit_xx - hit_c_x) / (float)hit_test_x;
                                    hit_step_start(4);
                                }
                                else { hit_move_wait++; }
                                break;
                        }
                        break;
                    }

                    /* ===================================================
                       Step 4：★ 追踪核心（重写）
                       算法：卡尔曼滤波 → 圆周角度更新 →
                             能量机关模式判断 → 正弦参数拟合 →
                             超前预测目标位置 → PID 云台控制
                       =================================================== */
                    case 4:
                    {
                        /* ── 1. 时间差（ms） ── */
                        float dt = (float)(hit_t - hit_t_la);
                        if(dt <= 0.0f || dt > 500.0f) dt = 20.0f;
                        hit_t_la = hit_t;

                        /* ── 2. 原始目标像素坐标 ── */
                        hit_xx = pack_sx - 320;
                        hit_xy = 240 - pack_sy;

                        /* ── 3. 卡尔曼滤波 ── */
                        KF_Update(&kf_x, (float)hit_xx, dt);
                        KF_Update(&kf_y, (float)hit_xy, dt);

                        float px = kf_x.x;   /* 平滑位置（像素） */
                        float py = kf_y.x;

                        /* ── 4. 圆周角度与瞬时角速度 ── */
                        float dx = px - (float)hit_c_x;
                        float dy = py - (float)hit_c_y;
                        float new_theta = atan2f(dy, dx);

                        /* 处理角度跳变（-π ~ π） */
                        float d_theta = new_theta - circ_theta;
                        if(d_theta >  PI_f) d_theta -= 2.0f * PI_f;
                        if(d_theta < -PI_f) d_theta += 2.0f * PI_f;
                        circ_theta = new_theta;

                        float inst_omega_ms = d_theta / dt;   /* rad/ms */

                        /* 写入角速度缓冲 */
                        omega_buf[omega_buf_idx] = inst_omega_ms;
                        omega_buf_idx = (omega_buf_idx + 1) % OMEGA_BUF_SIZE;
                        if(omega_buf_cnt < OMEGA_BUF_SIZE) omega_buf_cnt++;

                        /* ── 5. 模式判断：角速度方差 ── */
                        if(omega_buf_cnt >= OMEGA_BUF_SIZE)
                        {
                            int i;
                            float sum=0, sum2=0;
                            for(i=0; i<OMEGA_BUF_SIZE; i++) sum += omega_buf[i];
                            float mean = sum / OMEGA_BUF_SIZE;
                            for(i=0; i<OMEGA_BUF_SIZE; i++)
                                sum2 += (omega_buf[i]-mean)*(omega_buf[i]-mean);
                            float var = sum2 / OMEGA_BUF_SIZE;

                            /*
                             * 阈值参考：
                             *   小机关匀速 ω≈1.047 rad/s = 0.001047 rad/ms，方差极小
                             *   大机关正弦 ω波动大，方差明显
                             * 3e-8 (rad/ms)2 约对应 ±0.003 rad/ms 波动，可根据实测调整
                             */
                            if(var < 3e-8f)
                                energy_mode = 1;  /* 小机关 */
                            else
                                energy_mode = 2;  /* 大机关 */
                        }

                        /* ── 6. 角速度 IIR 低通平滑 ── */
                        circ_omega_ms = circ_omega_ms * 0.85f + inst_omega_ms * 0.15f;

                        /* ── 7. 大机关：在线梯度下降拟合正弦参数 ── */
                        if(energy_mode == 2)
                        {
                            float t_now      = (float)hit_t - sinfit_t0;
                            float sw_ms      = sinfit_w * 0.001f;   /* rad/ms */
                            float phase      = sw_ms * t_now + sinfit_phi;
                            float b          = 2.090f - sinfit_a;
                            float omega_pred = sinfit_a * sinf(phase) + b;  /* rad/ms */
                            float e          = inst_omega_ms - omega_pred;

                            /* 梯度下降（学习率经验值，可按实际调整） */
                            sinfit_a   += 2e-6f * e * sinf(phase);
                            sinfit_w   += 1e-7f * e * sinfit_a * t_now * cosf(phase) * 0.001f;
                            sinfit_phi += 1e-5f * e * sinfit_a * cosf(phase);

                            /* 参数约束 */
                            if(sinfit_a < 0.780f) sinfit_a = 0.780f;
                            if(sinfit_a > 1.045f) sinfit_a = 1.045f;
                            if(sinfit_w < 1.884f) sinfit_w = 1.884f;
                            if(sinfit_w > 2.000f) sinfit_w = 2.000f;
                        }

                        /* ── 8. 超前预测目标角度（补偿系统总延迟） ── */
                        float pred_theta;
                        float lead = PREDICT_LEAD_MS;

                        if(energy_mode == 1)
                        {
                            /* 小机关匀速：线性外推 */
                            pred_theta = circ_theta + circ_omega_ms * lead;
                        }
                        else if(energy_mode == 2)
                        {
                            /*
                             * 大机关正弦：数值积分预测角度
                             * θ(t+lead) = θ(t) + ∫[0,lead] ω(t+τ) dτ
                             * 梯形法，步长 5 ms
                             */
                            float t_now  = (float)hit_t - sinfit_t0;
                            float sw_ms  = sinfit_w * 0.001f;
                            float b      = 2.090f - sinfit_a;
                            float step   = 5.0f;
                            pred_theta   = circ_theta;
                            float tau;
                            for(tau = 0.0f; tau < lead; tau += step)
                            {
                                float om = sinfit_a * sinf(sw_ms*(t_now+tau) + sinfit_phi) + b;
                                pred_theta += om * 0.001f * step;   /* rad */
                            }
                        }
                        else
                        {
                            /* 模式未定：用平滑角速度简单外推 */
                            pred_theta = circ_theta + circ_omega_ms * lead;
                        }

                        /* ── 9. 预测目标像素坐标 ── */
                        float pred_px = (float)hit_c_x + hit_big_r * cosf(pred_theta);
                        float pred_py = (float)hit_c_y + hit_big_r * sinf(pred_theta);

                        /*
                         * ======================================================
                         * ★ 核心：前馈 + 残差 PID（替换原纯增量 PID）
                         * ======================================================
                         *
                         * 前馈：直接把预测像素坐标转换为云台 PWM 位置
                         *   云台PWM_x = hit_gml_c_x + pred_px / hit_Kx
                         *   云台PWM_y = hit_gml_c_y + pred_py / hit_Ky
                         * （pred_px, pred_py 是以图像中心为原点的像素偏移）
                         *
                         * 残差 PID：用当前测量位置（卡尔曼滤波后）与预测位置的
                         *   差值做微调，修正模型/标定误差
                         */

                        /* ── 10a. 前馈：预测位置 → 云台 PWM ── */
                        float ff_gml_x = (float)hit_gml_c_x + pred_px / hit_Kx;
                        float ff_gml_y = (float)hit_gml_c_y - pred_py / hit_Ky;

                        /* ── 10b. 前馈低通平滑 ── */
                        /*
                         * 不直接赋值，而是让云台朝前馈目标平滑收敛
                         * alpha 越小越平滑（越慢），alpha 越大响应越快
                         * 0.12 = 约 8 帧时间常数，30fps 下约 270ms
                         */
                        ff_gml_smooth_x += FF_SMOOTH_ALPHA * (ff_gml_x - ff_gml_smooth_x);
                        ff_gml_smooth_y += FF_SMOOTH_ALPHA * (ff_gml_y - ff_gml_smooth_y);

                        /* ── 10c. 残差：测量位置 - 预测位置 ── */
                        /*
                         * px, py 是卡尔曼滤波后的当前测量位置
                         * pred_px, pred_py 是模型预测的超前位置
                         * 残差 = 测量 - 预测，反映模型/标定误差
                         */
                        float res_x = px - pred_px;
                        float res_y = py - pred_py;

                        /* 积分累计（限幅） */
                        gml_ix += res_x * 0.02f;
                        gml_iy += res_y * 0.02f;
                        if(gml_ix >  GML_I_MAX) gml_ix =  GML_I_MAX;
                        if(gml_ix < -GML_I_MAX) gml_ix = -GML_I_MAX;
                        if(gml_iy >  GML_I_MAX) gml_iy =  GML_I_MAX;
                        if(gml_iy < -GML_I_MAX) gml_iy = -GML_I_MAX;

                        float d_res_x = res_x - gml_err_x_la;
                        float d_res_y = res_y - gml_err_y_la;
                        gml_err_x_la  = res_x;
                        gml_err_y_la  = res_y;

                        /* ── 10d. 云台输出 = 平滑前馈 + PID残差修正 ── */
                        hit_gmln_x = ff_gml_smooth_x + (gml_Kp*res_x + gml_Kd*d_res_x + gml_Ki*gml_ix) / hit_Kx;
                        hit_gmln_y = ff_gml_smooth_y - (gml_Kp*res_y + gml_Kd*d_res_y + gml_Ki*gml_iy) / hit_Ky;

                        /* ── 11. 云台位置限幅 ── */
                        if(hit_gmln_x < 300.0f) hit_gmln_x = 300.0f;
                        if(hit_gmln_x > 600.0f) hit_gmln_x = 600.0f;
                        if(hit_gmln_y < 120.0f) hit_gmln_y = 120.0f;
                        if(hit_gmln_y > 240.0f) hit_gmln_y = 240.0f;

                        /* ── 12. 保存历史 ── */
                        hit_xx_la = hit_xx;
                        hit_xy_la = hit_xy;

                        break;
                    }
                } /* end switch(hit_step) */

                /* ---- 扫描 / 锁定逻辑（保持原逻辑） ---- */
                if(hit_find_count == 0)
                {
                    /* 已定位，不再扫描 */
                }
                else if(pack_cx != 0)
                {
                    gml_find();

                    hit_xx = pack_sx - 320;
                    hit_xy = 240 - pack_sy;

                    if(hit_find_count >= 3 && hit_ept_count == 1
                       && (hit_xx_la-hit_xx) + (hit_xy_la-hit_xy) <=  100
                       && (hit_xx_la-hit_xx) + (hit_xy_la-hit_xy) >= -100
                       && (hit_xx_la-hit_xx) - (hit_xy_la-hit_xy) <=  100
                       && (hit_xx_la-hit_xx) - (hit_xy_la-hit_xy) >= -100)
                    {
                        hit_find_count -= 3;
                        gml_find();
                        hit_step_start(2);
                        hit_find_count = 0;
                    }

                    hit_xx_la = hit_xx;
                    hit_xy_la = hit_xy;
                }
                else
                {
                    gml_find();
                }

                /* ---- 清空数据包 ---- */
                pack_sx=0; pack_sy=0; pack_r=0;
                pack_cx=0; pack_cy=0; pack_state=0;
                hit_ept_count = 1;

            } /* end if(pack_state==5) */
        }
        else
        {
            /* ---- 'A'帧（无目标） ---- */
            hit_ept_count++;

            if(hit_find_count != 0)
            {
                gml_find();
            }

            switch(hit_step)
            {
                case 1:
                    break;

                case 2:
                    if(hit_ept_count >= 15)
                    {
                        hit_step_start(1);
                    }
                    break;

                case 3:
                    hit_move_wait++;
                    if(hit_testlist==1 && hit_ept_count>=80)
                    {
                        hit_testlist  = 2;
                        hit_move_wait = 1;
                        hit_gml(0, hit_gml_c_x);
                        hit_gml(1, hit_gml_c_y + hit_test_y);
                    }
                    if(hit_testlist==3 && hit_ept_count>=80)
                    {
                        hit_testlist  = 4;
                        hit_move_wait = 1;
                        hit_gml(0, hit_gml_c_x - hit_test_x);
                        hit_gml(1, hit_gml_c_y);
                    }
                    break;

                case 4:
                    /*
                     * 目标丢失时：利用已拟合的运动模型继续开环预测，
                     * 维持云台跟随，等待目标重新进入视野
                     * （云台输出已在 TIM3 中断 20ms 节拍执行）
                     */
                    break;
            }
        }

        USART_ClearFlag(USART1, USART_FLAG_RXNE);
    }
}


/* ============================================================
   USART2 中断 —— 遥控指令（不修改）
   ============================================================ */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART2);

        /* ==================================================
           新增：角度数据包解析（遥控器发的 [x,x,x,x] 格式）
           ================================================== */
        if(agl_pack_state == 1)
        {
            // 正在解析角度包
            if(data >= '0' && data <= '9')
            {
                // 数字字符，累计到临时数字
							
                agl_tmp_num = agl_tmp_num * 10 + (data - '0');
            }
            else if(data == ',')
            {
                // 逗号，把当前数字存下来，准备下一个
                if(agl_tmp_idx < 4)
                {
                    remote_agl[agl_tmp_idx] = agl_tmp_num;
                    agl_tmp_idx++;
                }
                agl_tmp_num = 0;
            }
            else if(data == ']')
            {
                // 包结束，存最后一个数字，回到普通模式
                if(agl_tmp_idx == 3) // 正好4个角度
                {
                    remote_agl[3] = agl_tmp_num;
                } 
//								if(remote_agl[0]>700)
//								take_dir[0]=2;
//								else if(remote_agl[0]<50)
//								take_dir[0]=1; 
//								else
//									take_dir[0]=0;
								for(int i=1;i<=3;i++)
								{
									if(remote_agl[i]>=100&&remote_agl[i]<=600)
								take_dir[i]=remote_agl[i] ;
                }
                // 重置状态
                agl_pack_state = 0;
                agl_tmp_num = 0;
                agl_tmp_idx = 0;
            }
            else
            {
                // 异常字符，重置状态
                agl_pack_state = 0;
                agl_tmp_num = 0;
                agl_tmp_idx = 0;
            }
            return; // 解析完就返回，不进遥控指令处理
        }
        else
        {
            // 普通模式，先判断是不是角度包的开头
            if(data == '[')
            {
                // 收到'['，说明接下来是角度包，切换到解析模式
                agl_pack_state = 1;
                agl_tmp_num = 0;
                agl_tmp_idx = 0;
                return;
            }
        }

        /* ==================================================
           原来的遥控指令处理，完全保留，只有普通模式才会进
           ================================================== */
        if(data=='a') { move_mode=0; }
        if(data=='A') { move_mode=1; }
        if(data=='B') { move_mode=2; }
        if(data=='U') { move_mode=7; }
        if(data=='V') { move_mode=8; }
        if(data=='C') { move_mode=3; }
        if(data=='D') { move_mode=4; }
        if(data=='E') { move_mode=5; }
        if(data=='F') { move_mode=6; }

        if(data=='O' && eat_mode>0) { eat_mode--; }
        if(data=='P' && eat_mode<7) { eat_mode++; }
        if(data=='Q') { eat_mode=3; }

        if(data=='G') { take_switch=1; }
        if(data=='H') { take_switch=2; }
        if(data=='I') { take_switch=3; }
        if(data=='J') { take_switch=4; }
        if(data=='K') { take_mode=1; }
        if(data=='L') { take_mode=2; }
        if(data=='b')
        {
            take_mode=0;
        }

        if(data=='M') { take_revolve=1; }
        if(data=='N') { take_revolve=2; }
        if(data=='m') { take_revolve=0; }
				if(data=='i') { take_revolve=4; }
        if(data=='j') { take_revolve=3 ; }
				
				if(data=='o') { take_dir[0]=1; }
        if(data=='p') { take_dir[0]=2; }
        if(data=='q') { take_dir[0]=0; }
				
				
				
				
				
				
        if(data=='R') { eat_pull_mode=1; }
        if(data=='S') { eat_pull_mode=2; }
        if(data=='r') { eat_pull_mode=0; }

				if(data=='d') { take_dir[1]=220;take_dir[2]=100;take_dir[3]=580; }
				if(data=='e') { take_dir[1]=250;take_dir[2]=250;take_dir[3]=500; }
				if(data=='f') { take_dir[1]=100;take_dir[2]=250;take_dir[3]=290; }
				if(data=='g') { take_dir[1]=580;take_dir[2]=410;take_dir[3]=560; }
				if(data=='h') { take_dir[1]=220;take_dir[2]=150;take_dir[3]=600; }

        if(data=='T')
        {
            if(hit_ready == 0)
            {
                hit_gml(0, 450);
                PCA9685_SetDuty(2, 180);
                hit_ready = 1;
            }
            else if(hit_ready==1 && hit_wait==0)
            {
                hit_gml(0, 450);
                PCA9685_SetDuty(2, 510);
                hit_ready = 0;
            }
        }
    }
}