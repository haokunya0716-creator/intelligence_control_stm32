/*
* app_button.h
 *
 *  Created on: May 6, 2025
 *      Author: gaoxi
 */

#ifndef INC_APP_BUTTON_H_
#define INC_APP_BUTTON_H_


extern int key1_flag;//单击调试标志，不启动任务
extern int key2_flag;//单击选择蓝方并启动
extern int key3_flag;//单击选择红方并启动

#include "main.h"

void App_Button_Init(void);
void App_Button_Proc(void);


#endif /* INC_APP_BUTTON_H_ */
