#include "rescue_usart2.h"

#include "app_motor.h"
#include "app_servo.h"
#include "app_speed.h"
#include "usart.h"

#define RESCUE_RX_QUEUE_SIZE       4U
#define RESCUE_INTERBYTE_TIMEOUT_MS 20U
#define RESCUE_LINK_TIMEOUT_MS     500U
#define RESCUE_UART_TX_TIMEOUT_MS  30U
#define RESCUE_MAX_WHEEL_CM_S      60
#define RESCUE_MAX_ANGLE_DECI_DEG  1800

typedef enum {
    RESCUE_CONTROL_STOPPED = 0,
    RESCUE_CONTROL_WHEEL_SPEED,
    RESCUE_CONTROL_ANGLE
} RescueControlMode;

static RescueParser parser;
static uint8_t rx_byte;
static RescueFrame queue[RESCUE_RX_QUEUE_SIZE];
static volatile uint8_t head;
static volatile uint8_t tail;
static volatile uint8_t count;
static volatile uint32_t dropped_count;
static volatile uint32_t parse_error_count;
static volatile uint32_t ack_error_count;
static uint32_t last_byte_ms;
static uint32_t last_valid_command_ms;
static uint8_t rx_active;
static RescueControlMode control_mode = RESCUE_CONTROL_STOPPED;
static int16_t last_relative_angle_deci_deg;
static uint8_t last_relative_angle_sequence;
static uint8_t last_relative_angle_valid;

uint8_t Rescue_Usart2_Start(void)
{
    if (rx_active != 0U) {
        return 1U;
    }

    Rescue_ParserInit(&parser);
    head = 0U;
    tail = 0U;
    count = 0U;
    dropped_count = 0U;
    parse_error_count = 0U;
    ack_error_count = 0U;
    last_relative_angle_deci_deg = 0;
    last_relative_angle_sequence = 0U;
    last_relative_angle_valid = 0U;
    last_byte_ms = HAL_GetTick();
    last_valid_command_ms = last_byte_ms;
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK) {
        return 0U;
    }
    rx_active = 1U;
    return 1U;
}

/*
 * 发送一条已经填好 command、sequence、length 和 payload 的帧。
 * 发送函数只在主循环调用，避免在接收中断里等待 UART 发完。
 */
uint8_t Rescue_Usart2_SendFrame(const RescueFrame *frame)
{
    uint8_t bytes[RESCUE_MAX_FRAME_SIZE];
    size_t length = Rescue_Encode(frame, bytes, sizeof(bytes));

    if (length == 0U ||
        HAL_UART_Transmit(&huart2, bytes, (uint16_t)length,
                          RESCUE_UART_TX_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    return 1U;
}

/* 接收中断只负责续接收、组帧和将有效帧放入小队列，不控制执行器。 */
void Rescue_Usart2_RxCallback(void)
{
    RescueFrame frame = {0};
    RescueParseResult result;
    uint8_t byte = rx_byte;
    uint32_t now_ms = HAL_GetTick();

    /* 每收到一个字节就立即挂上下一字节接收，降低连续数据漏收概率。 */
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK) {
        rx_active = 0U;
    }

    /* 半帧断流时丢弃旧半帧，防止新帧被拼到旧帧后面。 */
    if ((uint32_t)(now_ms - last_byte_ms) > RESCUE_INTERBYTE_TIMEOUT_MS) {
        Rescue_ParserInit(&parser);
    }
    last_byte_ms = now_ms;

    result = Rescue_ParserFeed(&parser, byte, &frame);
    if (result == RESCUE_PARSE_ERROR) {
        parse_error_count++;
        return;
    }
    if (result != RESCUE_PARSE_FRAME) {
        return;
    }

    /* ACK 是下位机回传帧，收到它时不能再次排队或再次应答。 */
    if (frame.command == RESCUE_CMD_ACK) {
        return;
    }

    if (frame.command == RESCUE_CMD_STOP) {
        /* 停车命令清除排队中的旧目标，优先交给主循环处理。 */
        head = 0U;
        tail = 0U;
        count = 0U;
    } else if (count != 0U && queue[tail].command == RESCUE_CMD_STOP) {
        dropped_count++;
        return;
    }

    if (count >= RESCUE_RX_QUEUE_SIZE) {
        dropped_count++;
        return;
    }

    queue[head] = frame;
    head = (uint8_t)((head + 1U) % RESCUE_RX_QUEUE_SIZE);
    count++;
}

/* 原子取出一帧，避免主循环与接收中断同时修改队列索引。 */
static uint8_t Rescue_Usart2_Pop(RescueFrame *out)
{
    uint32_t irq_state;

    irq_state = __get_PRIMASK();
    __disable_irq();
    if (count == 0U) {
        __set_PRIMASK(irq_state);
        return 0U;
    }

    *out = queue[tail];
    tail = (uint8_t)((tail + 1U) % RESCUE_RX_QUEUE_SIZE);
    count--;
    __set_PRIMASK(irq_state);
    return 1U;
}

/* ACK 的序号回显收到的命令序号，便于上位机对应请求与应答。 */
static void Rescue_Usart2_SendAck(uint8_t command, uint8_t sequence,
                                  RescueAckResult result)
{
    RescueFrame ack;

    ack.command = RESCUE_CMD_ACK;
    ack.sequence = sequence;
    ack.length = 2U;
    ack.payload[0] = command;
    ack.payload[1] = (uint8_t)result;
    if (Rescue_Usart2_SendFrame(&ack) == 0U) {
        ack_error_count++;
    }
}

/*
 * 业务分发：先检查命令载荷，再调用工程中已有的闭环或舵机接口。
 * ACK_ACCEPTED 表示目标已经交给控制模块，不代表车辆已到达目标。
 */
static RescueAckResult Rescue_Usart2_Execute(const RescueFrame *frame)
{
    int16_t left_cm_s;
    int16_t right_cm_s;
    int16_t angle_deci_deg;
    uint8_t servo_category;
    uint16_t servo_deci_deg;

    switch (frame->command) {
    case RESCUE_CMD_WHEEL_SPEED:
        if (frame->length != 4U) {
            return RESCUE_ACK_BAD_DATA;
        }
        left_cm_s = Rescue_ReadI16(&frame->payload[0]);
        right_cm_s = Rescue_ReadI16(&frame->payload[2]);
        if (left_cm_s < -RESCUE_MAX_WHEEL_CM_S ||
            left_cm_s > RESCUE_MAX_WHEEL_CM_S ||
            right_cm_s < -RESCUE_MAX_WHEEL_CM_S ||
            right_cm_s > RESCUE_MAX_WHEEL_CM_S) {
            return RESCUE_ACK_BAD_DATA;
        }

        /* 协议和底层速度环统一使用 cm/s，因此这里无需换算。 */
        App_Speed_Set((float)left_cm_s, (float)right_cm_s);
        control_mode = RESCUE_CONTROL_WHEEL_SPEED;
        last_valid_command_ms = HAL_GetTick();
        return RESCUE_ACK_ACCEPTED;

    case RESCUE_CMD_ANGLE_RELATIVE:
        if (frame->length != 2U) {
            return RESCUE_ACK_BAD_DATA;
        }
        angle_deci_deg = Rescue_ReadI16(frame->payload);
        if (angle_deci_deg < -RESCUE_MAX_ANGLE_DECI_DEG ||
            angle_deci_deg > RESCUE_MAX_ANGLE_DECI_DEG) {
            return RESCUE_ACK_BAD_DATA;
        }

        /*
         * 实车方向确认后，底层相对角度接口的正值对应逆时针，
         * 因此协议角度直接传给已有的相对角度接口。
         * 同一序号和同一增量是重发帧，只刷新链路时间，不重复叠加角度；
         * 需要再次转同样角度时，应使用新的序号。
         */
        if (last_relative_angle_valid == 0U ||
            last_relative_angle_sequence != frame->sequence ||
            last_relative_angle_deci_deg != angle_deci_deg) {
            Set_Angle_SP((float)angle_deci_deg / 10.0f);
            last_relative_angle_deci_deg = angle_deci_deg;
            last_relative_angle_sequence = frame->sequence;
            last_relative_angle_valid = 1U;
        }
        control_mode = RESCUE_CONTROL_ANGLE;
        last_valid_command_ms = HAL_GetTick();
        return RESCUE_ACK_ACCEPTED;

    case RESCUE_CMD_SERVO_ANGLE:
        if (frame->length != 3U) {
            return RESCUE_ACK_BAD_DATA;
        }
        servo_category = frame->payload[0];
        servo_deci_deg = Rescue_ReadU16(&frame->payload[1]);
        if (servo_deci_deg > 1800U) {
            return RESCUE_ACK_BAD_DATA;
        }
        switch (servo_category) {
        case RESCUE_SERVO_CATEGORY_1:
            App_Servo_SetAngle(&servo_1, (float)servo_deci_deg / 10.0f);
            break;
        case RESCUE_SERVO_CATEGORY_2:
            App_Servo_SetAngle(&servo_2, (float)servo_deci_deg / 10.0f);
            break;
        default:
            /* 舵机 3 及其他类别尚未开放。 */
            return RESCUE_ACK_UNSUPPORTED;
        }
        last_valid_command_ms = HAL_GetTick();
        return RESCUE_ACK_ACCEPTED;

    case RESCUE_CMD_STOP:
        if (frame->length != 0U) {
            return RESCUE_ACK_BAD_DATA;
        }
        App_Speed_Set(0.0f, 0.0f);
        control_mode = RESCUE_CONTROL_STOPPED;
        last_relative_angle_valid = 0U;
        last_valid_command_ms = HAL_GetTick();
        return RESCUE_ACK_ACCEPTED;

    case RESCUE_CMD_HEARTBEAT:
        if (frame->length != 0U) {
            return RESCUE_ACK_BAD_DATA;
        }
        last_valid_command_ms = HAL_GetTick();
        return RESCUE_ACK_ACCEPTED;

    default:
        return RESCUE_ACK_UNSUPPORTED;
    }
}

void Rescue_Usart2_Process(void)
{
    RescueFrame frame;
    RescueAckResult result;
    uint32_t now_ms;

    /* 若接收回调中重启失败，在主循环尝试重新挂接单字节接收。 */
    if (rx_active == 0U &&
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) == HAL_OK) {
        rx_active = 1U;
    }

    while (Rescue_Usart2_Pop(&frame) != 0U) {
        /* 所有通过解析校验的控制帧都执行；ACK 只用于确认处理结果。 */
        result = Rescue_Usart2_Execute(&frame);
        Rescue_Usart2_SendAck(frame.command, frame.sequence, result);
    }

    now_ms = HAL_GetTick();
    if (control_mode != RESCUE_CONTROL_STOPPED &&
        (uint32_t)(now_ms - last_valid_command_ms) >=
            RESCUE_LINK_TIMEOUT_MS) {
        /* 香橙派停止发送有效命令/心跳 500 ms 后，撤销运动目标。 */
        App_Speed_Set(0.0f, 0.0f);
        control_mode = RESCUE_CONTROL_STOPPED;
        last_relative_angle_valid = 0U;
    }

    /* 速度和角度命令会选择不同闭环；两者最终都驱动左右速度环。 */
    if (control_mode == RESCUE_CONTROL_WHEEL_SPEED) {
        App_Speed_Pro();
    } else if (control_mode == RESCUE_CONTROL_ANGLE) {
        App_Angle_Pro();
    }
}

uint32_t Rescue_Usart2_DroppedCount(void)
{
    return dropped_count;
}

uint32_t Rescue_Usart2_ParseErrorCount(void)
{
    return parse_error_count;
}

uint32_t Rescue_Usart2_AckErrorCount(void)
{
    return ack_error_count;
}
