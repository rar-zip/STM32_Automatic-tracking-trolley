#include "stm32f10x.h"                  
#include "Delay.h"
#include "robot.h"
#include "Key.h"

// 引入我们在 robot.c 中写的底层单轮控制函数 (1=左前轮, 2=右前轮)
extern void Set_Motor(uint8_t motor, int16_t speed);

// ------------------- 1. 定义 OpenMV 数据结构与全局变量 -------------------
#pragma pack(1) 
typedef struct {
    uint8_t  head1;     // 0x55
    uint8_t  head2;     // 0xAA
    int16_t  x_err;     // X轴偏差
    int16_t  y_err;     // Y轴偏差
    uint8_t  status;    // 状态 (1=看到目标, 0=丢失)
    uint8_t  direction; // 方向 (0=直行, 1=左转, 2=右转)
    uint8_t  checksum;  // 校验和
    uint8_t  tail;      // 0x0D
} OpenMV_Data_t;
#pragma pack()

#define RX_BUF_SIZE 30
uint8_t rx_buffer[RX_BUF_SIZE]; // DMA 接收缓冲区
OpenMV_Data_t target_data;      // 解析后的最终数据
uint8_t new_data_flag = 0;      // 新数据标志位
int16_t last_x_err = 0;         // 上一帧偏差值（用于阻尼控制）

// 控制参数
#define BASE_SPEED     85      // 基础速度
#define MAX_SPEED      95      // 最大速度
#define MIN_SPEED      40      // 最小速度
#define DEAD_ZONE      10      // 死区阈值
#define DAMPING_FACTOR 0.35    // 阻尼系数

// 分级转向参数 - 根据方向自适应调整
#define STRAIGHT_SENSITIVITY 0.04  // 直行灵敏度（低）
#define STRAIGHT_MAX_DIFF    6     // 直行最大速度差
#define LEFT_TURN_SENSITIVITY  0.07  // 左转灵敏度
#define LEFT_TURN_MAX_DIFF     15    // 左转最大速度差
#define RIGHT_TURN_SENSITIVITY 0.07  // 右转灵敏度
#define RIGHT_TURN_MAX_DIFF    15    // 右转最大速度差

// 急弯额外补偿
#define SHARP_BEND_BOOST 0.03   // 急弯时额外增加的灵敏度
#define SHARP_BEND_DIFF_BOOST 8 // 急弯时额外增加的速度差
#define SHARP_BEND_THRESHOLD 80 // 急弯阈值

#define LEFT_COMPENSATE 0      // 左轮速度补偿
#define RIGHT_COMPENSATE 0     // 右轮速度补偿


// ------------------- 2. USART1 与 DMA 初始化函数 -------------------
void OpenMV_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // 1. 打开时钟: GPIOA, USART1, 以及 DMA1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 2. 配置GPIO: PA9 (TX) 复用推挽, PA10 (RX) 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置USART1参数: 115200波特率
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // 4. 配置DMA1通道5 (USART1_RX)
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR; 
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx_buffer;       
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                
    DMA_InitStructure.DMA_BufferSize = RX_BUF_SIZE;                   
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;               
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    // 5. 配置NVIC中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 6. 开启 USART1 的 DMA 接收请求，以及空闲中断
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    // 7. 启动
    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

// ------------------- 3. 主函数 -------------------
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

    robot_Init();        
    Key_Init();          
    OpenMV_UART_Init();  

    uint8_t lose_line_cnt = 0; // 丢线计数器

    // 【安全机制】等待按下 PA15 按键后，才正式启动循迹
    while(Key_GetNum() == 0);

    // 【测试模式】启动后先测试轮子（前进，负值为向前）
    // 阶段1: 两个轮子同时前进1秒
    Set_Motor(1, -70);
    Set_Motor(2, -70);
    for(uint32_t i=0; i<72000000; i++);
    // 阶段2: 停止0.5秒
    Set_Motor(1, 0);
    Set_Motor(2, 0);
    for(uint32_t i=0; i<36000000; i++);

    while (1)
    {
        if (new_data_flag == 1)
        {
            new_data_flag = 0; 
            
            if (target_data.status == 1)  
            {
                lose_line_cnt = 0;
                
                int16_t x_err = target_data.x_err;
                uint8_t direction = target_data.direction;
                
                // 死区处理：偏差小于阈值时不转向，直行
                if (abs(x_err) < DEAD_ZONE) {
                    x_err = 0;
                }
                
                // 阻尼控制
                int16_t delta = x_err - last_x_err;
                float damping = 1.0;
                if (x_err * delta < 0 && abs(delta) > 10) {
                    damping = DAMPING_FACTOR;
                }
                last_x_err = x_err;
                
                // 分级转向：根据摄像头检测的方向自适应调整灵敏度和速度差
                float sensitivity = STRAIGHT_SENSITIVITY;
                int16_t max_diff = STRAIGHT_MAX_DIFF;
                
                // 根据方向选择基础参数
                if (direction == 1) {
                    // 检测到左转：使用左转参数
                    sensitivity = LEFT_TURN_SENSITIVITY;
                    max_diff = LEFT_TURN_MAX_DIFF;
                } else if (direction == 2) {
                    // 检测到右转：使用右转参数
                    sensitivity = RIGHT_TURN_SENSITIVITY;
                    max_diff = RIGHT_TURN_MAX_DIFF;
                }
                
                // 急弯额外补偿：偏差超过阈值时增加转向力度
                if (abs(x_err) >= SHARP_BEND_THRESHOLD) {
                    sensitivity += SHARP_BEND_BOOST;
                    max_diff += SHARP_BEND_DIFF_BOOST;
                }
                
                // 计算转向量
                int16_t turn_amount = (int16_t)(x_err * sensitivity * damping);
                
                // 限制最大速度差
                if (turn_amount > max_diff) turn_amount = max_diff;
                if (turn_amount < -max_diff) turn_amount = -max_diff;
                
                // 计算左右轮速度（前进模式，取负值实现向前）
                int16_t left_speed  = -(BASE_SPEED + turn_amount + LEFT_COMPENSATE);
                int16_t right_speed = -(BASE_SPEED - turn_amount + RIGHT_COMPENSATE);
                
                // 速度限制
                if (left_speed < -MAX_SPEED)  left_speed = -MAX_SPEED;
                if (left_speed > -MIN_SPEED)  left_speed = -MIN_SPEED;
                if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;
                if (right_speed > -MIN_SPEED) right_speed = -MIN_SPEED;
                
                // 确保两个轮子始终同向转动
                if (left_speed > 0) left_speed = -MIN_SPEED;
                if (right_speed > 0) right_speed = -MIN_SPEED;
                
                Set_Motor(1, left_speed);
                Set_Motor(2, right_speed);
            }
            else 
            {
                lose_line_cnt++;
                if(lose_line_cnt > 10) 
                {
                    Set_Motor(1, 0);
                    Set_Motor(2, 0);
                    lose_line_cnt = 10;
                }
            }
        }
    }
}

// ------------------- 4. 中断服务函数 -------------------
void USART1_IRQHandler(void)
{
    // 删掉了 uint8_t clear_temp;
    uint16_t rx_len;    

    // 清除溢出错误 (ORE)
    if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART1->SR; // 用 (void) 强转，完成纯粹的“假读”动作
        (void)USART1->DR; 
    }

    // 处理空闲中断 (IDLE)
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        (void)USART1->SR; // 同样在这里使用 (void) 假读
        (void)USART1->DR; 
        
        DMA_Cmd(DMA1_Channel5, DISABLE); 
        
        rx_len = RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
        
        // 动态查找帧头 (帧长度现在是10字节：2字节帧头 + 4字节数据 + 2字节状态/方向 + 1字节校验 + 1字节尾)
        if (rx_len >= 10) 
        {
            for (int i = 0; i <= rx_len - 10; i++) 
            {
                if (rx_buffer[i] == 0x55 && rx_buffer[i+1] == 0xAA && rx_buffer[i+9] == 0x0D)
                {
                    uint8_t sum = 0;
                    for (int j = i; j <= i + 7; j++) { 
                        sum += rx_buffer[j];
                    }
                    
                    if (sum == rx_buffer[i+8]) // 校验位匹配
                    {
                        target_data = *(OpenMV_Data_t*)(&rx_buffer[i]);
                        new_data_flag = 1; 
                        break; 
                    }
                }
            }
        }
        
        DMA_SetCurrDataCounter(DMA1_Channel5, RX_BUF_SIZE); 
        DMA_Cmd(DMA1_Channel5, ENABLE); 
    }
}