//
// Created by gaoxu on 2026/10/23.
//

#ifndef INTELLIGENT_RESCUE_APP_CARGO_TEST_H
#define INTELLIGENT_RESCUE_APP_CARGO_TEST_H

#include "app_task_move.h"
#include "app_servo.h"
#include "app_usart.h"
#include "rescue_usart2.h"

/*
 * 前框区域标定测试，独立头文件，无需加入CMake，不主动修改正式任务。
 * main.c包含本头文件，在while(1)最前面调用：
 *     if(App_Cargo_Test()){ continue; }
 * 不测试时注释掉这一整行；文件保留。测试期间跳过正式按键/任务处理。
 * 需先完成原有舵机、电机及Rescue_Usart2_Start()初始化。
 * 串口1：115200、8N1、文本接收；x/y及四条边均按640×480换算。
 */

// @简介：前框和摄像头保持落下，逐帧打印所有绿色的中心和边界；不筛置信度或前框区域。
// @返回：固定返回1，配合主循环continue独占测试；每次调用都处理串口接收，无阻塞延时。
static inline uint8_t App_Cargo_Test(void)
{
    static RescueScene scene; // 缓存最新场景，避免在主循环栈上反复分配大结构体。
    static uint8_t started, sent, seen; // 初始化、视角已发送及是否已处理过图像。
    static uint16_t view, frame; // 测试视角编号和上一张打印图像的编号。
    static uint32_t begin, send_time, capture; // 舵机等待、重发时间及上一张图的采集时间。
    uint32_t now = HAL_GetTick(); // 无阻塞计时，单位ms。

    App_Task_Move_Stop(); // 测试只观察物块，保持车轮停止。
    Rescue_Usart2_Process(); // 继续处理相机串口数据，使用原有接收中断，不重复启动接收。
    if(started == 0){
        view = Rescue_Usart2_GetScene(&scene) ? (uint16_t)(scene.view_id + 1U) : 65535U; // 避开当前缓存图的视角编号。
        if(view == 0){ view = 1; } // 编号回绕时跳过0。
        App_Servo_SetAngle(&servo_1, SERVO1_DROP); // 使用当前前框落下角度宏。
        App_Servo_SetAngle(&servo_2, SERVO2_DROP); // 使用当前摄像头低头角度宏。
        begin = now; // 从设置舵机时刻开始等待机械和画面稳定。
        started = 1;
    }
    if((uint32_t)(now - begin) < APP_TASK_SERVO_WAIT_MS + APP_TASK_SETTLE_TIME_MS){ return 1; }
    if(sent == 0 || (uint32_t)(now - send_time) >= APP_TASK_VIEW_RETRY_MS){
        sent = Rescue_Usart2_SendViewContext(view, 0); // 相机须给低头后采集的新图回显此编号。
        send_time = now;
    }
    if(sent == 0 || Rescue_Usart2_GetScene(&scene) == 0 || scene.view_id != view){ return 1; }
    if(scene.image_width == 0 || scene.image_height == 0){ return 1; } // 防止换算时除零。
    if(seen && frame == scene.frame_id && capture == scene.capture_ms){ return 1; } // 同一张图只打印一次。
    seen = 1;
    frame = scene.frame_id;
    capture = scene.capture_ms;
    for(uint8_t i = 0; i < scene.object_count; i++){
        const RescueObject *object = &scene.objects[i];
        if(object->type != RESCUE_OBJECT_SUPPLY){ continue; } // 所有绿色都打印，不经过正式单件/边界筛选。
        float x = object->cx * APP_TASK_IMAGE_W / scene.image_width;
        float y = object->cy * APP_TASK_IMAGE_H / scene.image_height;
        float half_w = object->width * (APP_TASK_IMAGE_W * 0.5f) / scene.image_width;
        float half_h = object->height * (APP_TASK_IMAGE_H * 0.5f) / scene.image_height;
        App_USART1_Printf("[GREEN] i=%u x=%.1f y=%.1f L=%.1f R=%.1f T=%.1f B=%.1f\r\n",
                          (unsigned int)i, x, y, x - half_w, x + half_w, y - half_h, y + half_h); // 中心及左/右/上/下边，便于标定整个检测框范围。
    }
    return 1; // 测试持续运行，无绿色时保持静默；注释主循环调用后恢复原程序。
}

#endif
