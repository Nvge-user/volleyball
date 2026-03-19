#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

#define E18_PIN   GPIO_PIN_0
#define E18_PORT  GPIOA         //定义E18引脚所在的GPIO端口和引脚号

#define MOVE_TIME_MS  1000     // 定义每个状态持续的时间，单位为毫秒

void MotorControl_Init(void);

void MotorControl_Update(void);

#endif
