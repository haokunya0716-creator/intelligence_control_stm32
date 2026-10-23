//
// Created by gaoxu on 2026/10/20.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_VISION_H // 检查头文件保护宏，防止重复包含时反复展开声明。
#define INTELLIGENT_RESCUE_APP_TASK_VISION_H // 定义头文件保护宏，后续重复包含时跳过本文件。

#include "app_task_move.h" // 引入运动动作及统一结果类型。

// @简介：锁存队伍对应物资分区，并复位视觉跟踪；不发送相机命令。
void App_Task_Vision_Init(uint8_t team); // 声明接口：锁存队伍对应物资分区，并复位视觉跟踪；不发送相机命令。

// @简介：停车并开始新视角；angle为摄像头设定俯角，settle_ms为发送新编号前的等待，0表示立即发送。
void App_Task_Vision_Begin(float angle, uint32_t settle_ms); // 声明接口：启动旋转搜索、复检和物资区搜索传0，撞散重选保留画面稳定等待。

// @简介：从完整新视角中选目标；1已锁定，0连续多张新图无候选，-1继续等待。
// @说明：初步调试只选可信绿色，不检查相邻物块，也不按相邻关系自动触发撞散。
int App_Task_Vision_Select(void); // 声明接口：从完整新视角中选目标；1已锁定，0连续多张新图无候选，-1继续等待。

// @简介：当前锁定是否用于撞散；锁定后保持该用途，不因列表顺序改变而跳目标。
uint8_t App_Task_Vision_IsMixed(void); // 声明接口：当前锁定是否用于撞散；锁定后保持该用途，不因列表顺序改变而跳目标。

// @简介：首次原地对准+持续差速接近+到位停车多帧复核；DONE时可执行下一机械动作。
// @说明：物块沿用3帧/600ms丢失处理；安全区失配立即旋转并返回RESEARCH，重新锁定后重新对准。
App_Task_Result App_Task_Vision_Follow(void); // 声明接口：首次原地对准+持续差速接近+到位停车多帧复核；DONE时可执行下一机械动作。

// @简介：摄像头不动，按640×480中心坐标检查三张有效新图，至少一张恰好一个可信绿色才DONE。
App_Task_Result App_Task_Vision_Verify(void); // 三张均未通过或出现多件/非绿色/低置信冲突返回FAILED，重复帧不计数。

// @简介：持续寻找本队物资区（直接标签或整体区+分界线推导），有效新图找到即锁定。
App_Task_Result App_Task_Vision_SearchZone(void); // 声明接口：原地寻找已锁存的本队物资区；DONE时停车并锁定分区供Follow继续接近。

// @简介：只读打印最后一次视觉检查现场，用于失败、复检超时及重选结束诊断。
void App_Task_Vision_LogDetails(const char *stage);

// @简介：撞散期间检查完整新鲜图像、物资数量及危险实体；不再跟踪被撞移动的单个框。
// @返回：1可继续，0立即停止；旋转会扫过周边，因此对整个可见列表进行保守检查。
uint8_t App_Task_Vision_ScatterClear(void); // 声明接口：撞散期间检查完整新鲜图像、物资数量及危险实体；不再跟踪被撞移动的单个框。

#endif // 结束本头文件的重复包含保护。
