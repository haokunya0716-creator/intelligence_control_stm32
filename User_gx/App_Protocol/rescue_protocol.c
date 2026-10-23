#include "rescue_protocol.h"

/*
 * CRC-16/CCITT-FALSE 参数：初值 0xFFFF、多项式 0x1021、无反射、异或值 0。
 * CRC 覆盖 version 到 payload，不包括帧头、CRC 自身和固定帧尾。
 */
static uint16_t Rescue_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                    ? (uint16_t)((crc << 1) ^ 0x1021U)
                    : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

int16_t Rescue_ReadI16(const uint8_t *data)
{
    uint16_t raw = Rescue_ReadU16(data);
    return raw < 0x8000U ? (int16_t)raw
                         : (int16_t)((int32_t)raw - 65536);
}

uint16_t Rescue_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void Rescue_WriteI16(uint8_t *data, int16_t value)
{
    Rescue_WriteU16(data, (uint16_t)value);
}

void Rescue_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

uint32_t Rescue_ReadU32(const uint8_t *data)
{
    return (uint32_t)Rescue_ReadU16(data) |
           ((uint32_t)Rescue_ReadU16(data + 2) << 16);
}

void Rescue_WriteU32(uint8_t *data, uint32_t value)
{
    Rescue_WriteU16(data, (uint16_t)value);
    Rescue_WriteU16(data + 2, (uint16_t)(value >> 16));
}

uint8_t Rescue_DecodeScene(const RescueFrame *frame, RescueScene *out)
{
    RescueScene scene = {0};
    uint8_t i;
    uint8_t header_size;
    uint8_t object_size;
    uint8_t max_objects;

    if (frame == NULL || out == NULL ||
        (frame->command != RESCUE_CMD_OBSERVATIONS &&
         frame->command != RESCUE_CMD_OBSERVATIONS_EXT)) {
        return 0U;
    }
    scene.has_source_age = frame->command == RESCUE_CMD_OBSERVATIONS_EXT;
    header_size = scene.has_source_age ? RESCUE_EXT_HEADER_SIZE : RESCUE_SCENE_HEADER_SIZE;
    object_size = scene.has_source_age ? RESCUE_EXT_OBJECT_SIZE : RESCUE_OBJECT_WIRE_SIZE;
    max_objects = scene.has_source_age ? RESCUE_EXT_MAX_OBJECTS : RESCUE_MAX_OBJECTS;
    if (frame->length < header_size) { return 0U; }
    if (scene.has_source_age) { scene.source_age_ms = Rescue_ReadU16(frame->payload + 14); }
    scene.frame_id = Rescue_ReadU16(frame->payload);
    scene.capture_ms = Rescue_ReadU32(frame->payload + 2);
    scene.view_id = Rescue_ReadU16(frame->payload + 6);
    scene.image_width = Rescue_ReadU16(frame->payload + 8);
    scene.image_height = Rescue_ReadU16(frame->payload + 10);
    scene.flags = frame->payload[12];
    scene.object_count = frame->payload[13];
    if (scene.object_count > max_objects ||
        frame->length != header_size + scene.object_count * object_size ||
        scene.image_width == 0U || scene.image_height == 0U ||
        (scene.flags & ~(RESCUE_SCENE_IMAGE_VALID | RESCUE_SCENE_TRUNCATED)) != 0U ||
        ((scene.flags & RESCUE_SCENE_IMAGE_VALID) == 0U &&
         (scene.object_count != 0U || scene.flags != 0U))) {
        return 0U;
    }
    for (i = 0U; i < scene.object_count; ++i) {
        const uint8_t *data = frame->payload + header_size + i * object_size;
        RescueObject *object = &scene.objects[i];
        object->type = data[0];
        object->confidence = data[1];
        object->cx = Rescue_ReadU16(data + 2);
        object->cy = Rescue_ReadU16(data + 4);
        object->width = Rescue_ReadU16(data + 6);
        object->height = Rescue_ReadU16(data + 8);
        if (scene.has_source_age) { object->track_id = Rescue_ReadU16(data + 10); }
        /* 非零ID在本帧必须唯一；重复ID不能作为重识别原目标的依据。 */
        if (object->track_id != 0U) {
            uint8_t j;
            for (j = 0U; j < i; ++j) {
                if (scene.objects[j].track_id == object->track_id) { return 0U; }
            }
        }
        /* 中心定义为 xmin + width/2（整数向下取整），框须在图像内。 */
        if (object->type > RESCUE_OBJECT_SAFE_ZONE_DIVIDER || object->confidence > 100U ||
            object->width == 0U || object->height == 0U ||
            object->width > scene.image_width || object->height > scene.image_height ||
            object->cx < object->width / 2U || object->cy < object->height / 2U ||
            (uint32_t)object->cx - object->width / 2U + object->width > scene.image_width ||
            (uint32_t)object->cy - object->height / 2U + object->height > scene.image_height) {
            return 0U;
        }
    }
    *out = scene;
    return 1U;
}

size_t Rescue_Encode(const RescueFrame *frame, uint8_t *output,
                     size_t capacity)
{
    size_t total;
    size_t i;
    uint16_t crc;
    uint8_t payload_length;

    if (frame == NULL || output == NULL ||
        frame->length > RESCUE_MAX_PAYLOAD) {
        return 0U;
    }

    payload_length = frame->length;
    total = RESCUE_FRAME_OVERHEAD + payload_length;
    if (capacity < total) {
        return 0U;
    }

    output[0] = RESCUE_FRAME_HEAD_0;
    output[1] = RESCUE_FRAME_HEAD_1;
    output[2] = RESCUE_PROTOCOL_VERSION;
    output[3] = frame->command;
    output[4] = frame->sequence;
    output[5] = payload_length;
    for (i = 0U; i < payload_length; ++i) {
        output[6U + i] = frame->payload[i];
    }

    crc = Rescue_Crc16(&output[2], 4U + payload_length);
    output[6U + payload_length] = (uint8_t)crc;
    output[7U + payload_length] = (uint8_t)(crc >> 8);
    output[8U + payload_length] = RESCUE_FRAME_TAIL_0;
    output[9U + payload_length] = RESCUE_FRAME_TAIL_1;
    return total;
}

void Rescue_ParserInit(RescueParser *parser)
{
    if (parser != NULL) {
        parser->count = 0U;
        parser->expected = 0U;
    }
}

/*
 * 错误后重新找帧头。若当前字节是 AA，就把它保留为新帧的第一个字节，
 * 这样相邻帧或损坏帧后的帧可以尽快重新同步。
 */
static void Rescue_ParserRestart(RescueParser *parser, uint8_t byte)
{
    parser->count = 0U;
    parser->expected = 0U;
    if (byte == RESCUE_FRAME_HEAD_0) {
        parser->bytes[0] = byte;
        parser->count = 1U;
    }
}

RescueParseResult Rescue_ParserFeed(RescueParser *parser, uint8_t byte,
                                    RescueFrame *out)
{
    uint8_t length;
    uint8_t i;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (parser == NULL || out == NULL) {
        return RESCUE_PARSE_ERROR;
    }

    if (parser->count == 0U) {
        if (byte == RESCUE_FRAME_HEAD_0) {
            parser->bytes[parser->count++] = byte;
        }
        return RESCUE_PARSE_WAIT;
    }
    if (parser->count == 1U) {
        if (byte == RESCUE_FRAME_HEAD_1) {
            parser->bytes[parser->count++] = byte;
        } else {
            Rescue_ParserRestart(parser, byte);
        }
        return RESCUE_PARSE_WAIT;
    }

    /* 最大帧长度由 length 检查保证，写入前不会超过固定缓冲区。 */
    parser->bytes[parser->count++] = byte;

    /* 收到 length 字段后，已经可以算出 CRC 与帧尾所在的精确位置。 */
    if (parser->count == 6U) {
        length = parser->bytes[5];
        if (parser->bytes[2] != RESCUE_PROTOCOL_VERSION ||
            length > RESCUE_MAX_PAYLOAD) {
            Rescue_ParserRestart(parser, byte);
            return RESCUE_PARSE_ERROR;
        }
        parser->expected = (uint8_t)(RESCUE_FRAME_OVERHEAD + length);
    }

    if (parser->expected == 0U || parser->count < parser->expected) {
        return RESCUE_PARSE_WAIT;
    }

    length = parser->bytes[5];
    received_crc = (uint16_t)parser->bytes[6U + length] |
                   ((uint16_t)parser->bytes[7U + length] << 8);
    calculated_crc = Rescue_Crc16(&parser->bytes[2], 4U + length);
    if (received_crc != calculated_crc ||
        parser->bytes[8U + length] != RESCUE_FRAME_TAIL_0 ||
        parser->bytes[9U + length] != RESCUE_FRAME_TAIL_1) {
        Rescue_ParserRestart(parser, byte);
        return RESCUE_PARSE_ERROR;
    }

    out->command = parser->bytes[3];
    out->sequence = parser->bytes[4];
    out->length = length;
    for (i = 0U; i < length; ++i) {
        out->payload[i] = parser->bytes[6U + i];
    }

    /* 完整帧已经交给调用者，最后一个帧尾字节不参与下一帧同步。 */
    Rescue_ParserRestart(parser, 0U);
    return RESCUE_PARSE_FRAME;
}
