#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Delay.h"
#include "robot.h"

// ================= 1. 初始化方向控制引脚 (2驱极简版) =================
// ================= 1. 初始化方向控制引脚 (2驱极简版) =================
void Dir_Init(void)
{
    // 1. 变量声明必须放在大括号里的最前面！
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 2. 然后再写执行动作
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 现在只用到一块驱动板的方向引脚：PB12, PB13, PB14, PB15
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// ================= 2. 机器人总初始化 =================
void robot_Init(void)
{
    PWM_Init(); // PB6和PB7现在分别控制左前轮和右前轮
    Dir_Init(); 
}

// ================= 3. 核心底层：单轮控制函数 =================
// motor: 1=左前轮, 2=右前轮
void Set_Motor(uint8_t motor, int16_t speed)
{
    uint8_t dir = (speed >= 0) ? 1 : 0; 
    uint16_t abs_speed = (speed >= 0) ? speed : -speed; 
    if(abs_speed > 100) abs_speed = 100; 

    switch(motor)
    {
        case 1: // 左前轮 (对应原来 L298N 的 ENA: PB6, IN1: PB12, IN2: PB13)
            TIM_SetCompare1(TIM4, abs_speed);
            if(abs_speed == 0)      { GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13); } 
            else if(dir)            { GPIO_SetBits(GPIOB, GPIO_Pin_12); GPIO_ResetBits(GPIOB, GPIO_Pin_13); }
            else                    { GPIO_ResetBits(GPIOB, GPIO_Pin_12); GPIO_SetBits(GPIOB, GPIO_Pin_13); }
            break;
            
        case 2: // 右前轮 (对应原来 L298N 的 ENB: PB7, IN3: PB14, IN4: PB15)
            TIM_SetCompare2(TIM4, abs_speed);
            if(abs_speed == 0)      { GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); }
            else if(dir)            { GPIO_SetBits(GPIOB, GPIO_Pin_14); GPIO_ResetBits(GPIOB, GPIO_Pin_15); }
            else                    { GPIO_ResetBits(GPIOB, GPIO_Pin_14); GPIO_SetBits(GPIOB, GPIO_Pin_15); }
            break;
    }
}

// ================= 4. 高层运动控制 =================

void makerobo_run(int8_t speed, uint16_t time)  // 前进
{
    Set_Motor(1, speed); // 左前正转
    Set_Motor(2, speed); // 右前正转
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_back(int8_t speed, uint16_t time) // 后退
{
    Set_Motor(1, -speed); 
    Set_Motor(2, -speed); 
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_brake(uint16_t time) // 刹车
{
    Set_Motor(1, 0); 
    Set_Motor(2, 0); 
    if(time > 0) Delay_ms(time);          
}

void makerobo_Left(int8_t speed, uint16_t time) // 差速左转 (左停，右进)
{
    Set_Motor(1, 0);     
    Set_Motor(2, speed); 
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_Right(int8_t speed, uint16_t time) // 差速右转 (左进，右停)
{
    Set_Motor(1, speed); 
    Set_Motor(2, 0);     
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_Spin_Left(int8_t speed, uint16_t time) // 左原地旋转
{
    Set_Motor(1, -speed); // 左侧退
    Set_Motor(2, speed);  // 右侧进
    Delay_ms(time);                    
    makerobo_brake(0);           			
}

void makerobo_Spin_Right(int8_t speed, uint16_t time) // 右原地旋转
{
    Set_Motor(1, speed);  // 左侧进
    Set_Motor(2, -speed); // 右侧退
    Delay_ms(time);                    
    makerobo_brake(0);           			
}