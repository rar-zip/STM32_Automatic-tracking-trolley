#include "stm32f10x.h"                  
#include "Delay.h"
#include "robot.h"
#include "Key.h"


// ------------------- 1. 定义 OpenMV 数据结构与全局变量 -------------------
#pragma pack(1) 
typedef struct {
    uint8_t  head1;     // 0x55
    uint8_t  head2;     // 0xAA
    int16_t  x_err;     // X轴偏差
    int16_t  y_err;     // Y轴偏差
    uint8_t  status;    // 状态
    uint8_t  checksum;  // 校验和
    uint8_t  tail;      // 0x0D
} OpenMV_Data_t;
#pragma pack()

#define RX_BUF_SIZE 30
uint8_t rx_buffer[RX_BUF_SIZE]; // DMA 收货月台 (接收缓冲区)
OpenMV_Data_t target_data;      // 解析后的最终数据
uint8_t new_data_flag = 0;      // 新数据标志位


// ------------------- 2. USART1 与 DMA 初始化函数 -------------------
void OpenMV_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // 1. 打开时钟: GPIOA, USART1, 以及 DMA1 的大门
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 2. 配置GPIO: PA9 (TX) 复用推挽, PA10 (RX) 浮空/上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入更稳定
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置USART1参数: 115200波特率, 8数据位, 1停止位, 无校验
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // 4. 配置DMA1的通道5 (这是USART1_RX的专属通道)
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR; // 外设地址:串口数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx_buffer;       // 内存地址:我们定义的数组
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                // 方向: 从外设到内存
    DMA_InitStructure.DMA_BufferSize = RX_BUF_SIZE;                   // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 每次搬运1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 正常模式(不循环)
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;               // 高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    // 5. 配置NVIC中断优先级 (允许串口触发空闲中断)
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 6. 核心开关：开启 USART1 的 DMA 接收请求，以及 空闲(IDLE) 中断
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    // 7. 最终启动外设
    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

// ------------------- 3. 主函数 -------------------
uint8_t i;

int main(void)
{
    // 配置中断分组 (通常在main最开头调一次)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

    robot_Init();        // 机器人底层PWM初始化
    Key_Init();          // 按键初始化
    
    OpenMV_UART_Init();  // 启动串口和DMA！听到OpenMV的声音

    // 在 while(1) 外面定义一个丢线计数器
    uint8_t lose_line_cnt = 0; 

    while (1)
    {
        if (new_data_flag == 1)
        {
            new_data_flag = 0; 
            
            if (target_data.status == 1) 
            {
                lose_line_cnt = 0; // 只要看到目标，计数器立刻清零（续命）
                
                // 阈值 20 根据画面大小调节
                if (target_data.x_err > 20) 
                {
                    // 目标偏右，右转：左轮转，右轮停 (或设为反转速度)
                    robot_speed(0, 50, 0, 0); 
                }
                else if (target_data.x_err < -20) 
                {
                    // 目标偏左，左转：左轮停，右轮转
                    robot_speed(0, 0, 50, 0); 
                }
                else 
                {
                    // 偏差很小，全速直行
                    robot_speed(0, 70, 70, 0); 
                }
            }
            else 
            {
                // 没识别到目标，千万不要立刻刹车！
                lose_line_cnt++;
                
                // 假设 OpenMV 一秒发 30 帧，连续 10 帧(约0.3秒)没看到，才真停车
                if(lose_line_cnt > 10) 
                {
                    robot_speed(0, 0, 0, 0); // 真正平滑停车
                    lose_line_cnt = 10;      // 防止变量溢出
                }
                // 注：如果 lose_line_cnt <= 10，代码什么都不做。
                // 定时器会保持上一个瞬间的速度，小车会靠着惯性和原速“冲”过那个噪点盲区！
            }
        }
    }
	}
// ------------------- 4. 中断服务函数 (写在 main.c 末尾) -------------------
void USART1_IRQHandler(void)
{
    uint8_t clear_temp; 
    uint16_t rx_len;    

    // 【新增】检查并清除溢出错误 (ORE) - 这是导致“死机重启才行”的常见元凶！
    if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        clear_temp = USART1->SR;
        clear_temp = USART1->DR; // 读SR再读DR即可清除ORE
    }

    // 判断是否是 串口空闲中断 (IDLE) 触发
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        clear_temp = USART1->SR;
        clear_temp = USART1->DR; // 清除 IDLE 标志位
        
        DMA_Cmd(DMA1_Channel5, DISABLE); // 1. 暂停 DMA
        
        rx_len = RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
        
        // 2. 动态查找帧头，而不是死板地只看 rx_buffer[0]
        if (rx_len >= 9) // 你的结构体总长是9字节
        {
            for (int i = 0; i <= rx_len - 9; i++) 
            {
                // 找到包头 0x55 0xAA，并且包尾是 0x0D
                if (rx_buffer[i] == 0x55 && rx_buffer[i+1] == 0xAA && rx_buffer[i+8] == 0x0D)
                {
                    uint8_t sum = 0;
                    // 你的原逻辑：包头到status(前7个字节)累加和等于校验位
                    for (int j = i; j <= i + 6; j++) { 
                        sum += rx_buffer[j];
                    }
                    
                    if (sum == rx_buffer[i+7]) // 校验位匹配
                    {
                        // 内存拷贝赋值
                        target_data = *(OpenMV_Data_t*)(&rx_buffer[i]);
                        new_data_flag = 1; 
                        break; // 找到一帧有效数据就退出循环
                    }
                }
            }
        }
        
        // 3. 重启 DMA
        DMA_SetCurrDataCounter(DMA1_Channel5, RX_BUF_SIZE); 
        DMA_Cmd(DMA1_Channel5, ENABLE); 
    }
}