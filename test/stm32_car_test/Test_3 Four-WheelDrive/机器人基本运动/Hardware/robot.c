#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Delay.h"
#include "robot.h"

// ================= 1. 初始化方向控制引脚 (GPIOB 统一版) =================
void Dir_Init(void)
{
    // 所有的方向引脚都在 GPIOB 上，只需开启 B 端口时钟！
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 一次性初始化 8 个引脚
    // 左侧：PB12, PB13, PB14, PB15
    // 右侧：PB1, PB10, PB11, PB5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | 
                                  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// ================= 2. 机器人总初始化 =================
void robot_Init(void)
{
    PWM_Init(); // 初始化你的 PB6-PB9 (PWM 调速)
    Dir_Init(); // 初始化 GPIOB 方向控制引脚
}

// ================= 3. 核心底层：单轮控制函数 =================
void Set_Motor(uint8_t motor, int16_t speed)
{
    uint8_t dir = (speed >= 0) ? 1 : 0; // 1为正转，0为反转
    uint16_t abs_speed = (speed >= 0) ? speed : -speed; // 绝对值速度
    if(abs_speed > 100) abs_speed = 100; // 限幅

    switch(motor)
    {
        case 1: // 左前轮 (PWM: PB6, DIR: PB12, PB13)
            TIM_SetCompare1(TIM4, abs_speed);
            if(abs_speed == 0)      { GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13); } 
            else if(dir)            { GPIO_SetBits(GPIOB, GPIO_Pin_12); GPIO_ResetBits(GPIOB, GPIO_Pin_13); }
            else                    { GPIO_ResetBits(GPIOB, GPIO_Pin_12); GPIO_SetBits(GPIOB, GPIO_Pin_13); }
            break;
            
        case 2: // 左后轮 (PWM: PB7, DIR: PB14, PB15)
            TIM_SetCompare2(TIM4, abs_speed);
            if(abs_speed == 0)      { GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); }
            else if(dir)            { GPIO_SetBits(GPIOB, GPIO_Pin_14); GPIO_ResetBits(GPIOB, GPIO_Pin_15); }
            else                    { GPIO_ResetBits(GPIOB, GPIO_Pin_14); GPIO_SetBits(GPIOB, GPIO_Pin_15); }
            break;
            
        case 3: // 右前轮 (PWM: PB8, DIR: PB1, PB10)
            TIM_SetCompare3(TIM4, abs_speed);
            if(abs_speed == 0)      { GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_10); }
            else if(dir)            { GPIO_SetBits(GPIOB, GPIO_Pin_1); GPIO_ResetBits(GPIOB, GPIO_Pin_10); }
            else                    { GPIO_ResetBits(GPIOB, GPIO_Pin_1); GPIO_SetBits(GPIOB, GPIO_Pin_10); }
            break;
            
        case 4: // 右后轮 (PWM: PB9, DIR: PB11, PB5)
            TIM_SetCompare4(TIM4, abs_speed);
            if(abs_speed == 0)      { GPIO_ResetBits(GPIOB, GPIO_Pin_11 | GPIO_Pin_5); }
            else if(dir)            { GPIO_SetBits(GPIOB, GPIO_Pin_11); GPIO_ResetBits(GPIOB, GPIO_Pin_5); }
            else                    { GPIO_ResetBits(GPIOB, GPIO_Pin_11); GPIO_SetBits(GPIOB, GPIO_Pin_5); }
            break;
    }
}

// ================= 4. 高层运动控制 (测试版：关闭后两轮动作) =================

void makerobo_run(int8_t speed, uint16_t time)  // 前进
{
    Set_Motor(1, speed); Set_Motor(2, 0); // 仅左前轮动
    Set_Motor(3, speed); Set_Motor(4, 0); // 仅右前轮动
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_back(int8_t speed, uint16_t time) // 后退
{
    Set_Motor(1, -speed); Set_Motor(2, 0); // 仅左前轮反转
    Set_Motor(3, -speed); Set_Motor(4, 0); // 仅右前轮反转
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_brake(uint16_t time) // 刹车
{
    Set_Motor(1, 0); Set_Motor(2, 0); 
    Set_Motor(3, 0); Set_Motor(4, 0);
    if(time > 0) Delay_ms(time);          
}

void makerobo_Left(int8_t speed, uint16_t time) // 差速左转
{	
    Set_Motor(1, 0);     Set_Motor(2, 0);
    Set_Motor(3, speed); Set_Motor(4, 0); // 仅右前轮进
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_Right(int8_t speed, uint16_t time) // 差速右转
{
    Set_Motor(1, speed); Set_Motor(2, 0); // 仅左前轮进
    Set_Motor(3, 0);     Set_Motor(4, 0);
    Delay_ms(time);                 
    makerobo_brake(0);           
}

void makerobo_Spin_Left(int8_t speed, uint16_t time) // 左原地旋转
{
    Set_Motor(1, -speed); Set_Motor(2, 0); // 左前轮退
    Set_Motor(3, speed);  Set_Motor(4, 0); // 右前轮进
    Delay_ms(time);                    
    makerobo_brake(0);           			
}

void makerobo_Spin_Right(int8_t speed, uint16_t time) // 右原地旋转
{
    Set_Motor(1, speed);  Set_Motor(2, 0); // 左前轮进
    Set_Motor(3, -speed); Set_Motor(4, 0); // 右前轮退
    Delay_ms(time);                    
    makerobo_brake(0);           			
}
