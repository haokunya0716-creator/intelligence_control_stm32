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
#define RESCUE_PROTOCOL_VERSION   0x02U

/* 数据长度由帧内 length 字段指定；该常量限制缓冲区大小。 */
#define RESCUE_MAX_OBJECTS        20U
#define RESCUE_SCENE_HEADER_SIZE  14U
#define RESCUE_OBJECT_WIRE_SIZE   10U
#define RESCUE_EXT_HEADER_SIZE    16U /* 原14字节图像头后追加source_age_ms:uint16 */
#define RESCUE_EXT_OBJECT_SIZE    12U /* 原10字节条目后追加track_id:uint16，0表示无ID */
#define RESCUE_EXT_MAX_OBJECTS    19U /* 16+19*12=244；加10字节外壳后254，适配uint8解析器 */
#define RESCUE_MAX_PAYLOAD        (RESCUE_EXT_HEADER_SIZE + RESCUE_EXT_MAX_OBJECTS * RESCUE_EXT_OBJECT_SIZE)
#define RESCUE_FRAME_OVERHEAD     10U
#define RESCUE_MAX_FRAME_SIZE     (RESCUE_FRAME_OVERHEAD + RESCUE_MAX_PAYLOAD)

/*
 * command 字段是命令号。新功能请增加唯一命令号，并在业务分发处增加
 * 对应的长度检查和处理，不必改变通用帧结构。
 */
typedef enum {
    RESCUE_CMD_HEARTBEAT = 0x05,    /* 相机 -> STM32：在线，不代表图像有效 */
    RESCUE_CMD_OBSERVATIONS = 0x10, /* 相机 -> STM32：一幅图像的完整检测列表 */
    RESCUE_CMD_OBSERVATIONS_EXT = 0x11, /* 可选扩展：采集至发送耗时及相机跟踪ID，兼容原0x10 */
    RESCUE_CMD_VIEW_CONTEXT = 0x20 /* STM32 -> 相机：视角编号及舵机目标角 */
} RescueCommand;

typedef enum {
    RESCUE_OBJECT_UNKNOWN = 0,
    RESCUE_OBJECT_SUPPLY = 1,       /* 绿色普通物资 */
    RESCUE_OBJECT_CORE_SUPPLY = 2,  /* 黑色核心物资 */
    RESCUE_OBJECT_CASUALTY = 3,     /* 橘色伤员 */
    RESCUE_OBJECT_DANGER = 4,       /* 浅蓝危险物块 */
    RESCUE_OBJECT_RED_SAFE_ZONE = 5,  /* 红方安全区整体，分区未知 */
    RESCUE_OBJECT_BLUE_SAFE_ZONE = 6, /* 蓝方安全区整体，分区未知 */
    RESCUE_OBJECT_START_ZONE = 7,
    RESCUE_OBJECT_OBSTACLE = 8,
    RESCUE_OBJECT_ROBOT = 9,
    /* 分区单独上报；坐标和检测框对应分区本身，不是整个安全区。 */
    RESCUE_OBJECT_RED_SUPPLY_ZONE = 10,    /* 红方物资放置区 */
    RESCUE_OBJECT_RED_CASUALTY_ZONE = 11,  /* 红方伤员放置区 */
    RESCUE_OBJECT_BLUE_SUPPLY_ZONE = 12,   /* 蓝方物资放置区 */
    RESCUE_OBJECT_BLUE_CASUALTY_ZONE = 13  /* 蓝方伤员放置区 */
} RescueObjectType;

#define RESCUE_SCENE_IMAGE_VALID  0x01U
#define RESCUE_SCENE_TRUNCATED    0x02U

/* 列表下标仅在本帧有效；相机ID可选，STM32仍检查类型、位置和尺寸。 */
typedef struct {
    uint8_t type;
    uint8_t confidence; /* 0～100 */
    uint16_t cx;        /* 像素坐标：左上角原点，x 向右，y 向下 */
    uint16_t cy;
    uint16_t width;
    uint16_t height;
    uint16_t track_id;  /* 扩展帧非零ID；同一视角内不可复用，普通帧为0 */
} RescueObject;

typedef struct {
    uint16_t frame_id;
    uint32_t capture_ms; /* 相机单调时钟，不可与 HAL_GetTick() 直接相减 */
    uint16_t view_id;    /* 拍摄时生效的视角编号；0 为未设置 */
    uint16_t image_width;
    uint16_t image_height;
    uint8_t flags;
    uint8_t object_count;
    uint8_t has_source_age; /* 1=0x11有真实耗时；0=0x10缺失，不可当成零延迟 */
    uint16_t source_age_ms; /* 相机本地：采集到准备发送的耗时，不与STM32时钟相减 */
    RescueObject objects[RESCUE_MAX_OBJECTS];
    uint32_t received_ms; /* STM32 收完串口帧的本地时间，不在线上传输 */
} RescueScene;

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
uint32_t Rescue_ReadU32(const uint8_t *data);
void Rescue_WriteU32(uint8_t *data, uint32_t value);

/* 逐字段解码，禁止直接把 payload 强转成结构体；失败不改变 out。 */
uint8_t Rescue_DecodeScene(const RescueFrame *frame, RescueScene *out);

#endif
