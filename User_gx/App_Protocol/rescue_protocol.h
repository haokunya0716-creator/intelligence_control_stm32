#ifndef RESCUE_PROTOCOL_H
#define RESCUE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/*
 * STM32 与香橙派之间使用原始二进制字节，不发送 ASCII 文本。
 * 例如帧头 AA 55 是两个字节 0xAA、0x55。
 */
#define RESCUE_FRAME_HEAD_0       0xAAU
#define RESCUE_FRAME_HEAD_1       0x55U
#define RESCUE_FRAME_TAIL_0       0x0DU
#define RESCUE_FRAME_TAIL_1       0x0AU
#define RESCUE_PROTOCOL_VERSION   0x01U

/* 数据长度由帧内 length 字段指定；该常量限制缓冲区大小。 */
#define RESCUE_MAX_PAYLOAD        32U
#define RESCUE_FRAME_OVERHEAD     10U
#define RESCUE_MAX_FRAME_SIZE     (RESCUE_FRAME_OVERHEAD + RESCUE_MAX_PAYLOAD)

/*
 * command 字段是命令号。新功能请增加唯一命令号，并在业务分发处增加
 * 对应的长度检查和处理，不必改变通用帧结构。
 */
typedef enum {
    RESCUE_CMD_WHEEL_SPEED = 0x01, /* 左右轮速度闭环目标 */
    RESCUE_CMD_ANGLE_RELATIVE = 0x02,/* 相对偏航角闭环目标 */
    RESCUE_CMD_SERVO_ANGLE = 0x03, /* 指定舵机类别及目标角度 */
    RESCUE_CMD_STOP = 0x04,        /* 立即停止左右轮 */
    RESCUE_CMD_HEARTBEAT = 0x05,   /* 上位机在线心跳 */
    RESCUE_CMD_ACK = 0x80          /* 下位机命令确认 */
} RescueCommand;

typedef enum {
    RESCUE_ACK_ACCEPTED = 0x00,    /* 已校验并接受，闭环仍在运行 */
    RESCUE_ACK_UNSUPPORTED = 0x01, /* 命令或硬件类别当前不支持 */
    RESCUE_ACK_BAD_DATA = 0x02,    /* 命令数据长度或取值错误 */
    RESCUE_ACK_BUSY = 0x03,        /* 暂时无法执行 */
    RESCUE_ACK_TX_ERROR = 0x04     /* 下位机 ACK 发送失败，仅本地统计可见 */
} RescueAckResult;

typedef enum {
    RESCUE_SERVO_CATEGORY_1 = 0x01, /* 夹爪：servo_1，TIM5_CH2 / PA1 */
    RESCUE_SERVO_CATEGORY_2 = 0x02, /* 摄像头：servo_2，TIM5_CH3 / PA2 */
    RESCUE_SERVO_CATEGORY_3 = 0x03  /* 保留，当前返回 UNSUPPORTED */
} RescueServoCategory;

/* CRC 校验和帧尾均通过后交给业务层的完整帧。 */
typedef struct {
    uint8_t command;
    uint8_t sequence;
    uint8_t length;
    uint8_t payload[RESCUE_MAX_PAYLOAD];
} RescueFrame;

/* 解析器按字节工作，可放在 USART 接收中断调用的路径中。 */
typedef struct {
    uint8_t bytes[RESCUE_MAX_FRAME_SIZE];
    uint8_t count;
    uint8_t expected;
} RescueParser;

typedef enum {
    RESCUE_PARSE_WAIT = 0, /* 当前帧还未收完整 */
    RESCUE_PARSE_FRAME,    /* out 中有完整且 CRC 正确的帧 */
    RESCUE_PARSE_ERROR     /* 帧格式、长度、版本、帧尾或 CRC 错误 */
} RescueParseResult;

void Rescue_ParserInit(RescueParser *parser);
RescueParseResult Rescue_ParserFeed(RescueParser *parser, uint8_t byte,
                                    RescueFrame *out);

/* 编码一帧，返回写入字节数；参数错误或输出空间不足返回 0。 */
size_t Rescue_Encode(const RescueFrame *frame, uint8_t *output,
                     size_t capacity);

/* 多字节整数使用小端序；int16 使用二进制补码。 */
int16_t Rescue_ReadI16(const uint8_t *data);
uint16_t Rescue_ReadU16(const uint8_t *data);
void Rescue_WriteI16(uint8_t *data, int16_t value);
void Rescue_WriteU16(uint8_t *data, uint16_t value);

#endif
