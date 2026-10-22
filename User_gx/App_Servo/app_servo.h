//
// Created by gaoxu on 2026/9/26.
//

#ifndef INTELLIGENT_RESCUE_APP_SERVO_H
#define INTELLIGENT_RESCUE_APP_SERVO_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
    float Angle; // 最近一次写入 PWM 的设定角度，单位：度，范围 0～180；不是实测角度
} Servo_t;

extern Servo_t servo_1; /* 夹爪：TIM5_CH2 / PA1 */
extern Servo_t servo_2; /* 摄像头：TIM5_CH3 / PA2 */

/* MX_TIM5_Init()后调用一次，前框初始化180°、摄像头75°。 */
void App_Servo_Init(void);

/* 初始化后调用；角度限幅到 0～180°，写入 PWM 后同步更新 servo->Angle。
 * 无效指针、通道或非有限角度不写寄存器，也不改变保存的角度。
 */
void App_Servo_SetAngle(Servo_t *servo, float angle);

#endif //INTELLIGENT_RESCUE_APP_SERVO_H
