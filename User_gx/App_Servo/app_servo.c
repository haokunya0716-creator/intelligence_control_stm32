//
// Created by gaoxu on 2026/9/26.
//

//控制180度舵机
//此文件默认定时器频率为50Hz  --   20ms
//高电平脉冲的时间0.5ms的话，舵机转到0度，1ms -45度 1.5ms--90度 2ms--135度2.5ms-180度

//当前文件arr+1 = 10000

#include "app_servo.h"

#include <math.h>
#include <stddef.h>

#include "tim.h"

Servo_t servo_1;//夹爪舵机
Servo_t servo_2;//摄像头舵机

static uint32_t Servo_angle_Compute(float angle_target);
//
//@brief: 初始化函数
//
void App_Servo_Init(void) {
    HAL_StatusTypeDef status;

    servo_1.htim = &htim5;
    servo_1.Channel = TIM_CHANNEL_2;

    servo_2.htim = &htim5;
    servo_2.Channel = TIM_CHANNEL_3;

    /* 先加载两路初始脉宽，再开启输出，避免先输出零脉宽。 */
    App_Servo_SetAngle(&servo_1, 180.0f);
    App_Servo_SetAngle(&servo_2, 180.0f);

    HAL_TIM_PWM_Start(servo_1.htim, servo_1.Channel);
    HAL_TIM_PWM_Start(servo_2.htim, servo_2.Channel);

}

void App_Servo_SetAngle(Servo_t *servo, float angle) {
    uint32_t ccr;

    if (servo == NULL || servo->htim == NULL ||
        servo->htim->Instance == NULL ||
        !IS_TIM_CCX_INSTANCE(servo->htim->Instance, servo->Channel) ||
        !isfinite(angle)) {
        return;
    }

    ccr = Servo_angle_Compute(angle);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, ccr);
}

static uint32_t Servo_angle_Compute(float angle_target) {
    if (angle_target > 180.0f) {
        angle_target = 180.0f;
    }
    if (angle_target <= 0.0f) {
        angle_target = 0.0f;
    }
    // TIM5计数频率为500kHz，每个计数为2us
    // 500us对应250，2500us对应1250
    return 250U + (uint32_t)(angle_target * 1000.0f / 180.0f);
}
