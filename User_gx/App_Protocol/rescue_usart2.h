#ifndef RESCUE_USART2_H
#define RESCUE_USART2_H

#include <stdint.h>
#include "rescue_protocol.h"

#define RESCUE_VISION_TIMEOUT_MS 500U

/* MX_USART2_UART_Init() 后调用一次，开始 USART2 单字节中断接收。 */
uint8_t Rescue_Usart2_Start(void);

/* 只在 HAL_UART_RxCpltCallback() 的 USART2 分支中调用。 */
void Rescue_Usart2_RxCallback(void);

/*
 * 主循环每次迭代都调用：处理视觉帧并更新最新场景缓存。
 * 不执行动作、不运行闭环、不发送逐帧 ACK；动作由 STM32 业务层决定。
 */
void Rescue_Usart2_Process(void);

/* 通用帧发送接口，主要用于状态上报或联调；成功返回 1。 */
uint8_t Rescue_Usart2_SendFrame(const RescueFrame *frame);

/* 主循环调用：返回最新且图像有效、接收年龄 < 500 ms 的完整快照。
 * count=0 是有效空场景。返回 0 时不写 out，业务层不得继续使用旧坐标。
 */
uint8_t Rescue_Usart2_GetScene(RescueScene *out);

/* 在线心跳和视觉帧均可证明链路在线；心跳不能让旧图像继续有效。 */
uint8_t Rescue_Usart2_LinkOnline(void);

/* 舵机稳定后发送新视角，直到收到该 view_id 的图像再使用坐标。
 * 相机必须在曝光/采集时锁存 view_id，不得把新编号贴到旧图像上。
 */
uint8_t Rescue_Usart2_SendViewContext(uint16_t view_id, int16_t yaw_deci_deg);

uint32_t Rescue_Usart2_DroppedCount(void);
uint32_t Rescue_Usart2_ParseErrorCount(void);
uint32_t Rescue_Usart2_BadDataCount(void);

#endif
