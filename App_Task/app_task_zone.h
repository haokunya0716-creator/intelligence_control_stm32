#ifndef INTELLIGENT_RESCUE_APP_TASK_ZONE_H
#define INTELLIGENT_RESCUE_APP_TASK_ZONE_H

#include "rescue_protocol.h"

#define APP_TASK_DERIVED_ZONE_MAX 4U

/* 同帧整体安全区+唯一分界线 -> 左物资/右伤员；输入必须是入口正面视角。
 * 不检查贴边、最小尺寸、形状、居中、线在区内或同时对应其它区。
 * 两侧区间取与安全区内部的交集，零宽区域不输出；保留原围栏内缩。
 * 只生成本地辅助框，不改写/裁剪原始场景，不占串口目标名额，不伪造跟踪ID。
 * 对应颜色已有直接分区标签时沿用原标签，不重复生成。失败返回0或仅其它颜色结果。
 */
uint8_t App_Task_Zone_Derive(const RescueScene *scene, RescueObject *zones,
                            uint8_t capacity);

/* 只读获取最近一帧对应队伍的推导原因，不改变分区或控制状态。 */
const char *App_Task_Zone_LastReason(uint8_t supply_type);

#endif
