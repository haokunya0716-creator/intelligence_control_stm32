#include "rescue_usart2.h"

#include "usart.h"

#define RESCUE_RX_QUEUE_SIZE        4U
#define RESCUE_INTERBYTE_TIMEOUT_MS 20U
#define RESCUE_UART_TX_TIMEOUT_MS   30U

typedef struct {
    RescueFrame frame;
    uint32_t received_ms;
} RescueReceivedFrame;

static RescueParser parser;
static uint8_t rx_byte;
static RescueReceivedFrame queue[RESCUE_RX_QUEUE_SIZE];
static volatile uint8_t head;
static volatile uint8_t tail;
static volatile uint8_t count;
static volatile uint8_t rx_active;
static volatile uint32_t dropped_count;
static volatile uint32_t parse_error_count;
static uint32_t bad_data_count;
static uint32_t last_byte_ms;
static uint32_t last_link_ms;
static uint8_t link_seen;
static RescueScene latest_scene;
static uint8_t scene_seen;

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
    bad_data_count = 0U;
    link_seen = 0U;
    scene_seen = 0U;
    last_byte_ms = HAL_GetTick();
    last_link_ms = last_byte_ms;
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK) {
        return 0U;
    }
    rx_active = 1U;
    return 1U;
}

/* 只在主循环发送；数据流无需逐帧 ACK 或重发。 */
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

uint8_t Rescue_Usart2_SendViewContext(uint16_t view_id, int16_t yaw_deci_deg)
{
    static RescueFrame frame;

    /* 0 留给尚未设置视角；角度相对车头，正值向左，不是 PWM 绝对角。 */
    if (view_id == 0U || yaw_deci_deg < -1800 || yaw_deci_deg > 1800) {
        return 0U;
    }
    frame.command = RESCUE_CMD_VIEW_CONTEXT;
    frame.sequence = (uint8_t)view_id;
    frame.length = 4U;
    Rescue_WriteU16(frame.payload, view_id);
    Rescue_WriteI16(frame.payload + 2, yaw_deci_deg);
    return Rescue_Usart2_SendFrame(&frame);
}

/* ISR 仅组帧和入队，不选择目标、不控制电机或舵机。 */
void Rescue_Usart2_RxCallback(void)
{
    static RescueFrame frame;
    RescueParseResult result;
    uint8_t byte = rx_byte;
    uint32_t now_ms = HAL_GetTick();

    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK) {
        rx_active = 0U;
    }
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
    if (count >= RESCUE_RX_QUEUE_SIZE) {
        dropped_count++;
        return;
    }
    queue[head].frame = frame;
    queue[head].received_ms = now_ms;
    head = (uint8_t)((head + 1U) % RESCUE_RX_QUEUE_SIZE);
    count++;
}

static uint8_t Rescue_Usart2_Pop(RescueReceivedFrame *out)
{
    uint32_t irq_state = __get_PRIMASK();

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

void Rescue_Usart2_Process(void)
{
    /* 主循环单调用者，静态工作区避免大场景结构体占用嵌套中断栈。 */
    static RescueReceivedFrame received;
    static RescueScene scene;

    if (rx_active == 0U &&
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) == HAL_OK) {
        rx_active = 1U;
    }
    while (Rescue_Usart2_Pop(&received) != 0U) {
        /* 用 ISR 的收帧时间判断年龄，主循环积压不能使旧图像变新。 */
        if ((uint32_t)(HAL_GetTick() - received.received_ms) >=
            RESCUE_VISION_TIMEOUT_MS) {
            continue;
        }
        switch (received.frame.command) {
        case RESCUE_CMD_OBSERVATIONS:
            if (Rescue_DecodeScene(&received.frame, &scene) == 0U) {
                bad_data_count++;
                break;
            }
            last_link_ms = received.received_ms;
            link_seen = 1U;
            /* 重发同一幅图像只证明链路在线，不能延长图像有效期。 */
            if (scene_seen != 0U &&
                (scene.flags & RESCUE_SCENE_IMAGE_VALID) != 0U &&
                (latest_scene.flags & RESCUE_SCENE_IMAGE_VALID) != 0U &&
                scene.frame_id == latest_scene.frame_id &&
                scene.capture_ms == latest_scene.capture_ms &&
                scene.view_id == latest_scene.view_id) {
                break;
            }
            scene.received_ms = received.received_ms;
            latest_scene = scene;
            scene_seen = 1U;
            break;
        case RESCUE_CMD_HEARTBEAT:
            if (received.frame.length != 0U) {
                bad_data_count++;
                break;
            }
            last_link_ms = received.received_ms;
            link_seen = 1U;
            break;
        default:
            /* v1 动作命令及方向不符的帧都不能触发执行器。 */
            bad_data_count++;
            break;
        }
    }
}

uint8_t Rescue_Usart2_GetScene(RescueScene *out)
{
    /* 本函数与 Process 一样只在主循环调用，避免复制过程被业务并发访问。 */
    if (out == NULL || scene_seen == 0U ||
        (latest_scene.flags & RESCUE_SCENE_IMAGE_VALID) == 0U ||
        (uint32_t)(HAL_GetTick() - latest_scene.received_ms) >=
            RESCUE_VISION_TIMEOUT_MS) {
        return 0U;
    }
    *out = latest_scene;
    return 1U;
}

uint8_t Rescue_Usart2_LinkOnline(void)
{
    return link_seen != 0U &&
           (uint32_t)(HAL_GetTick() - last_link_ms) < RESCUE_VISION_TIMEOUT_MS;
}

uint32_t Rescue_Usart2_DroppedCount(void)
{
    return dropped_count;
}

uint32_t Rescue_Usart2_ParseErrorCount(void)
{
    return parse_error_count;
}

uint32_t Rescue_Usart2_BadDataCount(void)
{
    return bad_data_count;
}
