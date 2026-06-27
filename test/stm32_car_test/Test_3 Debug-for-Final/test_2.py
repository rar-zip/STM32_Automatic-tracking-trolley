import sensor
import time
import struct
from pyb import UART, LED
import gc
import os

# ============================================================
# 1. 初始化
# ============================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
for _ in range(30):
    sensor.snapshot()
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(3, 115200)  # UART(3) 对应 P4(TX)/P5(RX)
clock = time.clock()

led_red = LED(1)
led_blue = LED(2)
last_led_time = time.ticks_ms()
led_state = False

# ============================================================
# 2. 核心参数
# ============================================================
SCREEN_CENTER_X = sensor.width() // 2
SCREEN_CENTER_Y = sensor.height() // 2

width = sensor.width()
height = sensor.height()
margin = int(width * 0.05)
ROI_RECT = (margin, 0, width - 2 * margin, height)

BLACK_THRESHOLDS = [
    (0, 30, -20, 20, -20, 20),
    (0, 15, -128, 127, -128, 127),
    (0, 20, -33, 50, -11, 27)
]

# ============================================================
# 3. 防错参数
# ============================================================
LOST_THRESHOLD_MS = 500
last_detect_time = time.ticks_ms()
last_offset_x = 0
last_offset_y = 0
last_blob_cx = SCREEN_CENTER_X  # 记录上一帧blob的中心X坐标
last_blob_cy = SCREEN_CENTER_Y  # 记录上一帧blob的中心Y坐标
FILTER_STRENGTH = 0.4           # 滤波强度（增大使输出更平滑）
TRACKING_DISTANCE_THRESHOLD = 100  # 帧间追踪最大距离阈值

# 状态常量
STATUS_TRACKING = 0x01
STATUS_LOST = 0x00

# ============================================================
# 视频录制参数
# ============================================================
RECORD_INTERVAL_MS = 500  # 每500ms保存一帧
MAX_RECORD_FRAMES = 500   # 最多保存500帧
last_record_time = time.ticks_ms()
frame_count = 0
video_dir = "/sd/video"


def init_video_dir():
    """初始化视频保存目录"""
    try:
        if "video" not in os.listdir("/sd"):
            os.mkdir("/sd/video")
        print("Video directory ready")
    except Exception as e:
        print(f"SD card error: {e}")


def save_frame(img):
    """保存帧到SD卡"""
    global frame_count, last_record_time
    if time.ticks_diff(time.ticks_ms(), last_record_time) >= RECORD_INTERVAL_MS:
        if frame_count < MAX_RECORD_FRAMES:
            filename = f"{video_dir}/frame_{frame_count:04d}.jpg"
            img.save(filename, quality=70)
            frame_count += 1
            last_record_time = time.ticks_ms()
            print(f"Saved frame {frame_count}")


def build_frame(x_err, y_err, status):
    """打包为STM32端 OpenMV_Data_t 二进制帧"""
    # <: little-endian, B: uint8, h: int16
    payload = struct.pack('<BBhhB', 0x55, 0xAA, x_err, y_err, status)
    # 模拟STM32端的uint8_t累加，确保溢出行为一致
    checksum = 0
    for byte in payload:
        checksum = (checksum + byte) & 0xFF
    return payload + struct.pack('<BB', checksum, 0x0D)


# ============================================================
# 4. 主循环
# ============================================================
while True:
    gc.collect()
    clock.tick()
    img = sensor.snapshot()

    img.median(1, percentile=0.5)
    img.erode(1)

    blobs = img.find_blobs(BLACK_THRESHOLDS, roi=ROI_RECT,
                           pixels_threshold=600, merge=True, margin=5)

    target_blob = None
    valid_blobs = []

    # 第一步：筛选出所有合格的blob
    if blobs:
        for b in blobs:
            w, h = b.w(), b.h()
            if h == 0:
                continue
            ratio = w / h
            # 放宽宽高比限制，适应不同形状的黑线
            if 0.15 < ratio < 8.0 and b.density() > 0.25 and b.pixels() > 400:
                valid_blobs.append(b)

    # 第二步：从合格blob中选择目标（帧间追踪优先）
    if valid_blobs:
        # 策略：优先选择与上一帧位置接近的blob（避免跳变）
        min_track_dist = 10000
        best_track_blob = None

        for b in valid_blobs:
            # 计算与上一帧blob的距离
            dist_to_last = abs(b.cx() - last_blob_cx) + abs(b.cy() - last_blob_cy)
            if dist_to_last < min_track_dist:
                min_track_dist = dist_to_last
                best_track_blob = b

        # 如果找到与上一帧位置接近的blob，使用它（帧间追踪成功）
        if min_track_dist < TRACKING_DISTANCE_THRESHOLD:
            target_blob = best_track_blob
        else:
            # 帧间追踪失败，选择距离屏幕中心最近的blob（作为备选）
            min_center_dist = 10000
            for b in valid_blobs:
                dist_to_center = abs(b.cx() - SCREEN_CENTER_X)
                if dist_to_center < min_center_dist:
                    min_center_dist = dist_to_center
                    target_blob = b

    if target_blob:
        current_x = target_blob.cx() - SCREEN_CENTER_X
        current_y = target_blob.cy() - SCREEN_CENTER_Y

        # 更新历史位置（用于下一帧的帧间追踪）
        last_blob_cx = target_blob.cx()
        last_blob_cy = target_blob.cy()

        filtered_x = int(last_offset_x * (1 - FILTER_STRENGTH) + current_x * FILTER_STRENGTH)
        filtered_y = int(last_offset_y * (1 - FILTER_STRENGTH) + current_y * FILTER_STRENGTH)
        last_offset_x = filtered_x
        last_offset_y = filtered_y

        frame = build_frame(filtered_x, filtered_y, STATUS_TRACKING)
        uart.write(frame)
        print(f"X_err: {filtered_x}, Y_err: {filtered_y}, blobs: {len(valid_blobs)}")
        last_detect_time = time.ticks_ms()

        # 蓝灯闪烁
        led_red.off()
        if time.ticks_diff(time.ticks_ms(), last_led_time) > 200:
            led_state = not led_state
            led_blue.on() if led_state else led_blue.off()
            last_led_time = time.ticks_ms()

        # 调试绘制
        img.draw_rectangle(target_blob.rect(), color=(0, 255, 0), thickness=3)
        img.draw_cross(target_blob.cx(), target_blob.cy(), color=(0, 255, 0))
        img.draw_line(target_blob.cx(), target_blob.cy(),
                      SCREEN_CENTER_X, SCREEN_CENTER_Y, color=(255, 0, 0))
    else:
        if time.ticks_diff(time.ticks_ms(), last_detect_time) > LOST_THRESHOLD_MS:
            # 丢失时发送最后一帧已知位置，状态标记为 LOST
            frame = build_frame(last_offset_x, last_offset_y, STATUS_LOST)
            uart.write(frame)
            print("LOST")
        else:
            print("Searching...")

        # 红灯闪烁
        led_blue.off()
        if time.ticks_diff(time.ticks_ms(), last_led_time) > 200:
            led_state = not led_state
            led_red.on() if led_state else led_red.off()
            last_led_time = time.ticks_ms()

    img.draw_rectangle(ROI_RECT, color=(0, 0, 255))
    img.draw_line(SCREEN_CENTER_X, 0, SCREEN_CENTER_X, sensor.height(), color=(128, 128, 128))
    img.draw_line(0, SCREEN_CENTER_Y, sensor.width(), SCREEN_CENTER_Y, color=(128, 128, 128))

    save_frame(img)
