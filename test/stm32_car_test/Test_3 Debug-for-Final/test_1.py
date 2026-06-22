import sensor
import time
import struct
from pyb import UART, LED
import gc

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
FILTER_STRENGTH = 0.3

# 状态常量
STATUS_TRACKING = 0x01
STATUS_LOST = 0x00


def build_frame(x_err, y_err, status):
    """打包为STM32端 OpenMV_Data_t 二进制帧"""
    # <: little-endian, B: uint8, h: int16
    payload = struct.pack('<BBhhB', 0x55, 0xAA, x_err, y_err, status)
    checksum = sum(payload) & 0xFF
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

    if blobs:
        blob_list = list(blobs)
        min_dist = 10000
        for b in blob_list:
            w, h = b.w(), b.h()
            if h == 0:
                continue
            ratio = w / h
            if 0.2 < ratio < 5.0 and b.density() > 0.2:
                dist = abs(b.cx() - SCREEN_CENTER_X)
                if dist < min_dist:
                    min_dist = dist
                    target_blob = b

    if target_blob:
        current_x = target_blob.cx() - SCREEN_CENTER_X
        current_y = target_blob.cy() - SCREEN_CENTER_Y

        filtered_x = int(last_offset_x * (1 - FILTER_STRENGTH) + current_x * FILTER_STRENGTH)
        filtered_y = int(last_offset_y * (1 - FILTER_STRENGTH) + current_y * FILTER_STRENGTH)
        last_offset_x = filtered_x
        last_offset_y = filtered_y

        frame = build_frame(filtered_x, filtered_y, STATUS_TRACKING)
        uart.write(frame)
        print(f"X_err: {filtered_x}, Y_err: {filtered_y}")
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
