#include "MotorControl.h"
#include "motor_control.h"

// 三个电机变量
MOTOR_send cmd_1 = {
    cmd_1.id   = 0,
    cmd_1.mode = 1,
    cmd_1.T    = 0.0f,
    cmd_1.W    = 0.0f,
    cmd_1.Pos  = 4.31f,
    cmd_1.K_P  = 0.6f,
    cmd_1.K_W  = 0.04f
};
MOTOR_send cmd_2 = {
    cmd_2.id   = 3,
    cmd_2.mode = 1,
    cmd_2.T    = 0.0f,
    cmd_2.W    = 0.0f,
    cmd_2.Pos  = 5.56f,
    cmd_2.K_P  = 0.6f,
    cmd_2.K_W  = 0.04f
};
MOTOR_send cmd_3 = {
    cmd_3.id   = 4,
    cmd_3.mode = 1,
    cmd_3.T    = 0.0f,
    cmd_3.W    = 0.0f,
    cmd_3.Pos  = 1.07f,
    cmd_3.K_P  = 0.6f,
    cmd_3.K_W  = 0.04f
};

MOTOR_recv data_1;
MOTOR_recv data_2;
MOTOR_recv data_3;

static uint32_t last_check_time = 0;
static uint8_t  motor_state = 0;
static int32_t state_start_time = 0;
static uint8_t  last_pin_state = 0xFF;

void MotorControl_Init(void)
{
    last_pin_state = HAL_GPIO_ReadPin(E18_PORT, E18_PIN);
    motor_state = 0;
}

void MotorControl_Update(void)
{

    if (last_pin_state == 0xFF)
    {
        last_pin_state = HAL_GPIO_ReadPin(E18_PORT, E18_PIN);
    }

    uint8_t current_pin_state = HAL_GPIO_ReadPin(E18_PORT, E18_PIN);
    if (current_pin_state == GPIO_PIN_SET)
    {
        if(state_start_time == 0)
        {motor_state = 1;}
        state_start_time ++;
    }
    last_pin_state = current_pin_state;

    switch (motor_state)
    {
        case 0:
            cmd_1.Pos = 4.31f;
            cmd_2.Pos = 5.56f;
            cmd_3.Pos = 1.07f;
            cmd_1.K_P = 0.0f;
            cmd_2.K_P = 0.0f;
            cmd_3.K_P = 0.0f;
            break;

        case 1:
            cmd_1.Pos = -1.6f;
            cmd_2.Pos = -1.46f;
            cmd_3.Pos = -5.95f;
            cmd_1.K_P = 0.6f;
            cmd_2.K_P = 0.6f;
            cmd_3.K_P = 0.6f;

            if (state_start_time >= MOVE_TIME_MS)
            {
                motor_state = 2;
                state_start_time = -MOVE_TIME_MS;
            }
            break;

        case 2:
            cmd_1.Pos = 4.31f;
            cmd_2.Pos = 5.56f;
            cmd_3.Pos = 1.07f;
            cmd_1.K_P = 0.3f;
            cmd_2.K_P = 0.3f;
            cmd_3.K_P = 0.3f;
            if (HAL_GetTick() - state_start_time >= MOVE_TIME_MS)
            {
                motor_state = 0;
            }
            break;

        default:
            motor_state = 0;
            break;
    }

    SERVO_Send_recv_Motor2(&cmd_1, &data_1);
	
//    SERVO_Send_recv_Motor1(&cmd_2, &data_2);
//    
//    SERVO_Send_recv_Motor1(&cmd_3, &data_3);
    
}
