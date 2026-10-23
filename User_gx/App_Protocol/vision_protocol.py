"""Vision v2 wire codec; detection and STM32 mission logic are separate."""

from dataclasses import dataclass
import binascii
import struct
import time

VERSION = 2
HEARTBEAT = 0x05
OBSERVATIONS = 0x10
OBSERVATIONS_EXT = 0x11
VIEW_CONTEXT = 0x20
IMAGE_VALID = 0x01
TRUNCATED = 0x02
MAX_OBJECTS = 20
MAX_EXT_OBJECTS = 19
MAX_PAYLOAD = 244

# 与 rescue_protocol.h 一致；YOLO 类别编号须显式映射到这些协议类型。
OBJECT_UNKNOWN = 0
OBJECT_SUPPLY = 1
OBJECT_CORE_SUPPLY = 2
OBJECT_CASUALTY = 3
OBJECT_DANGER = 4
OBJECT_RED_SAFE_ZONE = 5   # 安全区整体，尚未区分物资区和伤员区
OBJECT_BLUE_SAFE_ZONE = 6
OBJECT_START_ZONE = 7
OBJECT_OBSTACLE = 8
OBJECT_ROBOT = 9
OBJECT_RED_SUPPLY_ZONE = 10
OBJECT_RED_CASUALTY_ZONE = 11
OBJECT_BLUE_SUPPLY_ZONE = 12
OBJECT_BLUE_CASUALTY_ZONE = 13
OBJECT_SAFE_ZONE_DIVIDER = 14  # 安全区中间黑色分割线


@dataclass(frozen=True)
class Detection:
    type: int
    confidence: int
    cx: int
    cy: int
    width: int
    height: int
    track_id: int = 0  # 真实跟踪器ID；0表示没有，禁止用本帧列表下标代替。


def encode_frame(command, sequence, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = struct.pack("<BBBB", VERSION, command, sequence, len(payload)) + payload
    return b"\xaa\x55" + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF)) + b"\r\n"


def encode_scene(*, sequence, frame_id, capture_ms, view_id, image_width,
                 image_height, objects, image_valid=True, truncated=False, source_age_ms=None):
    """编码完整场景；传source_age_ms启用0x11扩展，否则保留原0x10格式。"""
    objects = tuple(objects)
    extended = source_age_ms is not None
    if extended and not 0 <= source_age_ms <= 65535:
        raise ValueError("invalid source age")
    if not 0 < image_width <= 65535 or not 0 < image_height <= 65535:
        raise ValueError("invalid image dimensions")
    if len(objects) > (MAX_EXT_OBJECTS if extended else MAX_OBJECTS):
        raise ValueError("too many objects; explicitly select entries and set truncated")
    if not image_valid and (objects or truncated):
        raise ValueError("invalid image must have an empty, non-truncated list")
    flags = (IMAGE_VALID if image_valid else 0) | (TRUNCATED if truncated else 0)
    payload = struct.pack("<HIHHHBB", frame_id, capture_ms, view_id,
                          image_width, image_height, flags, len(objects))
    if extended:
        payload += struct.pack("<H", source_age_ms)
    ids = set()
    for obj in objects:
        if not 0 <= obj.track_id <= 65535 or (obj.track_id and not extended):
            raise ValueError("track ID requires extended scene with source age")
        if obj.track_id:
            if obj.track_id in ids:
                raise ValueError("duplicate track ID")
            ids.add(obj.track_id)
        if not 0 <= obj.type <= OBJECT_SAFE_ZONE_DIVIDER or not 0 <= obj.confidence <= 100:
            raise ValueError("invalid type/confidence")
        if not 0 < obj.width <= image_width or not 0 < obj.height <= image_height:
            raise ValueError("invalid box size")
        xmin, ymin = obj.cx - obj.width // 2, obj.cy - obj.height // 2
        if xmin < 0 or ymin < 0 or xmin + obj.width > image_width or ymin + obj.height > image_height:
            raise ValueError("box outside image")
        payload += struct.pack("<BBHHHH", obj.type, obj.confidence, obj.cx,
                               obj.cy, obj.width, obj.height)
        if extended:
            payload += struct.pack("<H", obj.track_id)
    return encode_frame(OBSERVATIONS_EXT if extended else OBSERVATIONS, sequence, payload)


def encode_latest_scene(*, capture_ms, max_age_ms, now_ms=None, **scene):
    """在串口发送前编码，并用相机自己的单调时钟计算耗时；过期结果直接拒绝。

    capture_ms必须在采集时记录，同一时钟取monotonic_ns()//1_000_000，线上保留低32位。
    调用方采集/推理队列只留最新帧，编码后立即发送，不能把结果再次放进积压队列。
    本函数不包含YOLO或摄像头驱动，也不能测出驱动内部缓存的曝光延迟。
    """
    if now_ms is None:
        now_ms = time.monotonic_ns() // 1_000_000
    age = (now_ms - capture_ms) & 0xFFFFFFFF
    if not 0 < max_age_ms <= 65535 or age >= max_age_ms:
        raise ValueError("expired image; discard queued inference result")
    return encode_scene(capture_ms=capture_ms & 0xFFFFFFFF, source_age_ms=age, **scene)


def decode_frame(packet):
    if len(packet) < 10 or packet[:2] != b"\xaa\x55" or packet[-2:] != b"\r\n":
        raise ValueError("invalid frame boundaries")
    version, command, sequence, length = struct.unpack_from("<BBBB", packet, 2)
    if version != VERSION or length > MAX_PAYLOAD or len(packet) != 10 + length:
        raise ValueError("invalid version/length")
    expected = binascii.crc_hqx(packet[2:6 + length], 0xFFFF)
    if struct.unpack_from("<H", packet, 6 + length)[0] != expected:
        raise ValueError("CRC mismatch")
    return command, sequence, packet[6:6 + length]


def decode_view_context(payload):
    if len(payload) != 4:
        raise ValueError("invalid view context length")
    view_id, yaw_deci_deg = struct.unpack("<Hh", payload)
    if view_id == 0 or not -1800 <= yaw_deci_deg <= 1800:
        raise ValueError("invalid view context")
    return view_id, yaw_deci_deg


class FrameParser:
    """Feed serial read chunks; returns (command, sequence, payload) tuples.

    On a >20 ms gap inside a partial frame, the caller should invoke reset(),
    matching the STM32 inter-byte timeout. This codec does not own a clock.
    """

    def __init__(self):
        self.buffer = bytearray()

    def reset(self):
        self.buffer.clear()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while self.buffer:
            start = self.buffer.find(b"\xaa\x55")
            if start < 0:
                self.buffer[:] = b"\xaa" if self.buffer[-1] == 0xAA else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            if self.buffer[2] != VERSION or self.buffer[5] > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            size = 10 + self.buffer[5]
            if len(self.buffer) < size:
                break
            try:
                frame = decode_frame(bytes(self.buffer[:size]))
            except ValueError:
                del self.buffer[0]
                continue
            frames.append(frame)
            del self.buffer[:size]
        return frames
