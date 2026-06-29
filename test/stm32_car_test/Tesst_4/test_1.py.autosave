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

uart = UART(3, 115200)
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
ROI_RECT = (margin, 0, width - 2 * margin, int(height * 0.70))

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
FILTER_STRENGTH = 0.35
DEAD_ZONE = 10
TRACKING_DIST_THRESHOLD = 200
last_blob_cx = SCREEN_CENTER_X
last_blob_cy = SCREEN_CENTER_Y

STATUS_TRACKING = 0x01
STATUS_LOST = 0x00

# ============================================================
# 4. 方向检测参数
# ============================================================
DIR_STRAIGHT = 0
DIR_LEFT = 1
DIR_RIGHT = 2
last_direction = DIR_STRAIGHT


def build_frame(x_err, y_err, status, direction=0):
    payload = struct.pack('<BBhhBB', 0x55, 0xAA, x_err, y_err, status, direction)
    checksum = sum(payload) & 0xFF
    return payload + struct.pack('<BB', checksum, 0x0D)


# ============================================================
# 5. 主循环
# ============================================================
while True:
    gc.collect()
    clock.tick()
    img = sensor.snapshot()

    img.median(1, percentile=0.5)
    img.erode(1)

    blobs = img.find_blobs(BLACK_THRESHOLDS, roi=ROI_RECT,
                           pixels_threshold=400, merge=True, margin=5)

    target_blob = None

    if blobs:
        blob_list = list(blobs)
        valid_blobs = []
        for b in blob_list:
            w, h = b.w(), b.h()
            if h == 0:
                continue
            ratio = w / h
            if (0.1 < ratio < 8.0) and (b.density() > 0.15):
                valid_blobs.append(b)

        if valid_blobs:
            min_track_dist = 10000
            best_track_blob = None
            for b in valid_blobs:
                dist_to_last = abs(b.cx() - last_blob_cx) + abs(b.cy() - last_blob_cy)
                if dist_to_last < min_track_dist:
                    min_track_dist = dist_to_last
                    best_track_blob = b

            if best_track_blob and min_track_dist < TRACKING_DIST_THRESHOLD:
                target_blob = best_track_blob
            else:
                best_score = 10000
                for b in valid_blobs:
                    score = abs(b.cx() - SCREEN_CENTER_X) + b.cy() * 0.5
                    if score < best_score:
                        best_score = score
                        target_blob = b

    if target_blob:
        last_blob_cx = target_blob.cx()
        last_blob_cy = target_blob.cy()

        current_x = target_blob.cx() - SCREEN_CENTER_X
        current_y = target_blob.cy() - SCREEN_CENTER_Y

        # 方向检测：根据blob的形状和位置判断黑线走向
        blob_width = target_blob.w()
        blob_height = target_blob.h()
        blob_cx = target_blob.cx()
        blob_left = target_blob.x()
        blob_right = target_blob.x() + blob_width

        # 判断方向：检测黑线是向左还是向右转弯
        # 只有当blob明显偏向一侧时才判定为转弯，否则保持直行
        direction = DIR_STRAIGHT
        if blob_width > blob_height * 2.0:
            # 非常宽的blob，可能是转弯处
            if blob_cx < SCREEN_CENTER_X - 40:
                direction = DIR_LEFT
            elif blob_cx > SCREEN_CENTER_X + 40:
                direction = DIR_RIGHT
        elif blob_left < SCREEN_CENTER_X - 60 and blob_right < SCREEN_CENTER_X + 30:
            # blob完全在左侧，判定为左转
            direction = DIR_LEFT
        elif blob_right > SCREEN_CENTER_X + 60 and blob_left > SCREEN_CENTER_X - 30:
            # blob完全在右侧，判定为右转
            direction = DIR_RIGHT

        # 平滑处理：如果当前检测到转弯，但之前是直行，且偏差不大，则保持直行
        if last_direction == DIR_STRAIGHT and direction != DIR_STRAIGHT:
            if abs(current_x) < 50:
                direction = DIR_STRAIGHT

        last_direction = direction

        filtered_x = int(last_offset_x * (1 - FILTER_STRENGTH) + current_x * FILTER_STRENGTH)
        filtered_y = int(last_offset_y * (1 - FILTER_STRENGTH) + current_y * FILTER_STRENGTH)
        last_offset_x = filtered_x
        last_offset_y = filtered_y

        output_x = filtered_x if abs(filtered_x) >= DEAD_ZONE else 0

        frame = build_frame(output_x, filtered_y, STATUS_TRACKING, direction)
        uart.write(frame)
        print(f"X_err: {output_x}, Dir: {direction}")
        last_detect_time = time.ticks_ms()

        led_red.off()
        if time.ticks_diff(time.ticks_ms(), last_led_time) > 200:
            led_state = not led_state
            led_blue.on() if led_state else led_blue.off()
            last_led_time = time.ticks_ms()

        # 绘制所有有效blob的真实轮廓（手动连接角点）
        for b in valid_blobs:
            corners = b.corners()
            for i in range(len(corners)):
                x1, y1 = corners[i]
                x2, y2 = corners[(i + 1) % len(corners)]
                img.draw_line(x1, y1, x2, y2, color=(0, 128, 0), thickness=2)

        # 绘制目标blob的真实轮廓（手动连接角点，绿色加粗）
        corners = target_blob.corners()
        for i in range(len(corners)):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % len(corners)]
            img.draw_line(x1, y1, x2, y2, color=(0, 255, 0), thickness=3)
        img.draw_cross(target_blob.cx(), target_blob.cy(), color=(0, 255, 0))

        # 根据方向绘制箭头指示
        if direction == DIR_LEFT:
            img.draw_line(SCREEN_CENTER_X, SCREEN_CENTER_Y,
                         SCREEN_CENTER_X - 30, SCREEN_CENTER_Y, color=(0, 0, 255))
            img.draw_line(SCREEN_CENTER_X - 30, SCREEN_CENTER_Y,
                         SCREEN_CENTER_X - 20, SCREEN_CENTER_Y - 5, color=(0, 0, 255))
            img.draw_line(SCREEN_CENTER_X - 30, SCREEN_CENTER_Y,
                         SCREEN_CENTER_X - 20, SCREEN_CENTER_Y + 5, color=(0, 0, 255))
        elif direction == DIR_RIGHT:
            img.draw_line(SCREEN_CENTER_X, SCREEN_CENTER_Y,
                         SCREEN_CENTER_X + 30, SCREEN_CENTER_Y, color=(0, 0, 255))
            img.draw_line(SCREEN_CENTER_X + 30, SCREEN_CENTER_Y,
                         SCREEN_CENTER_X + 20, SCREEN_CENTER_Y - 5, color=(0, 0, 255))
            img.draw_line(SCREEN_CENTER_X + 30, SCREEN_CENTER_Y,
                         SCREEN_CENTER_X + 20, SCREEN_CENTER_Y + 5, color=(0, 0, 255))
    else:
        if time.ticks_diff(time.ticks_ms(), last_detect_time) > LOST_THRESHOLD_MS:
            frame = build_frame(last_offset_x, last_offset_y, STATUS_LOST, last_direction)
            uart.write(frame)
            print("LOST")
        else:
            print("Searching...")

        led_blue.off()
        if time.ticks_diff(time.ticks_ms(), last_led_time) > 200:
            led_state = not led_state
            led_red.on() if led_state else led_red.off()
            last_led_time = time.ticks_ms()

    img.draw_rectangle(ROI_RECT, color=(0, 0, 255))
    img.draw_line(SCREEN_CENTER_X, 0, SCREEN_CENTER_X, sensor.height(), color=(128, 128, 128))
    img.draw_line(0, SCREEN_CENTER_Y, sensor.width(), SCREEN_CENTER_Y, color=(128, 128, 128))
