/*
* app_button.c
 *
 *  Created on: May 6, 2025
 *      Author: gaoxi
 */

#include "../Inc/app_button.h"
#include "../Inc/button.h"

static Button_TypeDef userKey1; // 用户按钮
static Button_TypeDef userKey2;
static Button_TypeDef userKey3;

int key1_flag;//单击调试标志，不启动任务
int key2_flag;//单击选择蓝方并启动
int key3_flag;//单击选择红方并启动

static void OnUserKey_Clicked1(uint8_t clicks);
static void OnUserKey_Clicked2(uint8_t clicks);
static void OnUserKey_Clicked3(uint8_t clicks);

//
// @简介：按钮初始化
//
void App_Button_Init(void)
{
    Button_InitTypeDef Button_InitStruct1 = {0};

    Button_InitStruct1.GPIOx = GPIOC;
    Button_InitStruct1.GPIO_Pin = GPIO_PIN_10;

    My_Button_Init(&userKey1, &Button_InitStruct1);

    My_Button_SetClickCb(&userKey1, OnUserKey_Clicked1);
    /////////////////////////////////////////////////
    Button_InitTypeDef Button_InitStruct2 = {0};

    Button_InitStruct2.GPIOx = GPIOC;
    Button_InitStruct2.GPIO_Pin = GPIO_PIN_11;

    My_Button_Init(&userKey2, &Button_InitStruct2);

    My_Button_SetClickCb(&userKey2, OnUserKey_Clicked2);
    //////////////////////////////////////////////////
    Button_InitTypeDef Button_InitStruct3 = {0};

    Button_InitStruct3.GPIOx = GPIOC;
    Button_InitStruct3.GPIO_Pin = GPIO_PIN_9;

    My_Button_Init(&userKey3, &Button_InitStruct3);

    My_Button_SetClickCb(&userKey3, OnUserKey_Clicked3);
}

void App_Button_Proc(void)
{
    My_Button_Proc(&userKey1);
    My_Button_Proc(&userKey2);
    My_Button_Proc(&userKey3);
}
//
// @简介：按钮点击的回调函数
//
static void OnUserKey_Clicked1(uint8_t clicks)
{
    if(clicks == 1)
    {
        key1_flag = 1;
    }
}
static void OnUserKey_Clicked2(uint8_t clicks)
{
    if(clicks == 1)
    {
        key2_flag = 1;
    }
}
static void OnUserKey_Clicked3(uint8_t clicks)
{
    if(clicks == 1)
    {
        key3_flag = 1;
    }
}
