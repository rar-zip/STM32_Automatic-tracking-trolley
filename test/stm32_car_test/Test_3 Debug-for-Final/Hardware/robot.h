#ifndef __ROBOT_H
#define __ROBOT_H

#include "stm32f10x.h" // 包含标准库，确保识别 int8_t 和 uint16_t

void robot_Init(void);

// 基本运动函数 (注意：这里的 speed 已经全部统一修改为 int8_t)
void makerobo_run(int8_t speed, uint16_t time);        // 机器人前进
void makerobo_back(int8_t speed, uint16_t time);       // 后退函数
void makerobo_brake(uint16_t time);                    // 机器人停止
void makerobo_Left(int8_t speed, uint16_t time);       // 差速左转
void makerobo_Right(int8_t speed, uint16_t time);      // 差速右转
void makerobo_Spin_Left(int8_t speed, uint16_t time);  // 原地左旋转
void makerobo_Spin_Right(int8_t speed, uint16_t time); // 原地右旋转

#endif

