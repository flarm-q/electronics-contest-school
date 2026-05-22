import time
import sensor
import lcd
from machine import UART
from fpioa_manager import fm


# UART1 wiring for the K210 board shown in the pin diagram:
#   IO7 -> U1_TX, connect to STM32 USART RX
#   IO9 -> U1_RX, connect to STM32 USART TX if bidirectional debug is needed
#   GND must be common with STM32 GND
UART_BAUDRATE = 115200
UART1_TX_PIN = 7
UART1_RX_PIN = 9

# K210 runs this script at QVGA for frame rate. The STM32 side already uses
# the K230/VGA error scale, so line error and angle are scaled by 2 before send.
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
ERROR_SCALE = 2

# 摄像头方向校正。
#
# 如果摄像头实际安装方向和小车前进方向相反，LCD/IDE 里看到的画面会“反着”。
# 这会直接影响巡线 error 的正负方向和色块 L/C/R 位置判断，所以必须在图像进入
# 识别算法之前完成方向校正，而不是只在显示时旋转。
#
# 当前按“摄像头倒装 180 度”处理:
#   CAMERA_HMIRROR = True   水平镜像，修正左右方向
#   CAMERA_VFLIP   = True   垂直翻转，修正上下方向
#
# 如果你的现象只是左右反了，把 CAMERA_VFLIP 改成 False。
# 如果只是上下反了，把 CAMERA_HMIRROR 改成 False。
CAMERA_HMIRROR = True
CAMERA_VFLIP = True

SEND_LINE_INTERVAL_MS = 50
SEND_COLOR_INTERVAL_MS = 120
DEBUG_PRINT_INTERVAL_MS = 500

# 色块避障开关。
#
# True:
#   K210 会周期性查找红/绿/蓝色块，并通过 $C,color_id,pos,size# 帧上报给 STM32。
#   STM32 收到后会根据 color_id/pos/size 进入视觉避障逻辑。
#
# 注意这里没有每帧都做色块识别。K210 算力比 K230 弱，如果巡线和三色识别都每帧
# 全量执行，实际运行时很容易表现为帧率很低甚至像“卡住”。因此色块识别按
# SEND_COLOR_INTERVAL_MS 限频执行，巡线仍然保持更高频率输出。
ENABLE_COLOR_AVOIDANCE = True

# LAB 色块阈值，顺序依次为红、绿、蓝。
#
# 每个元组格式为:
#   (L_min, L_max, A_min, A_max, B_min, B_max)
#
# 现场调试建议:
#   1. 如果误识别阴影或黑线，优先收窄 A/B 范围。
#   2. 如果目标颜色识别断断续续，适当放宽对应颜色的 A/B 范围。
#   3. 如果整体太暗或曝光变化大，先调整摄像头安装和补光，再改 L 范围。
COLOR_THRESHOLDS = [
    (0, 80, 20, 127, -10, 30),      # 红色
    (0, 80, -120, -10, 0, 30),      # 绿色
    (0, 80, 0, 90, -128, -20),      # 蓝色
]

# LCD 调试画框颜色，与 COLOR_THRESHOLDS 顺序一致。
DRAW_COLORS = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
]

COLOR_NAMES = ("NONE", "RED", "GREEN", "BLUE")

# 色块过滤下限。
#
# K210 当前使用 QVGA(320x240)，像素面积比 K230/VGA 小 4 倍，所以这里的阈值
# 也比 K230 脚本低。向 STM32 上报 size 时会再乘 4，尽量保持下位机原来的
# K230_AVOID_MIN_SIZE 判断尺度不用马上重调。
COLOR_MIN_PIXELS = 50
COLOR_MIN_AREA = 50

# 位置分类边界。
#
# STM32 侧避障只关心 L/C/R 三个区域，不需要接收精确坐标。
# color_id 非 0 且 size 足够大时:
#   L: 障碍在画面左侧
#   C: 障碍在画面中间
#   R: 障碍在画面右侧
LEFT_BOUND = FRAME_WIDTH // 3
RIGHT_BOUND = FRAME_WIDTH * 2 // 3

# White background / black line threshold. Tune L_MAX first for your venue:
# lower value resists shadows, higher value reconnects broken black tracks.
LINE_THRESHOLD = (0, 45, -128, 127, -128, 127)

# ROI format: (x, y, w, h, weight). Same strategy as the K230 script, scaled
# to QVGA: near ROI has the largest weight, middle/far ROIs judge curve trend.
LINE_ROIS = [
    (0, 165, FRAME_WIDTH, 35, 50),
    (0, 125, FRAME_WIDTH, 33, 30),
    (0, 85, FRAME_WIDTH, 35, 20),
]

LINE_MIN_PIXELS = 20
LINE_MIN_AREA = 20
LINE_MAX_JUMP = 80
LINE_CURVE_THRESHOLD = 35
LINE_FLAG_VALID = 0x01
LINE_FLAG_LOST = 0x02
LINE_FLAG_CURVE = 0x04

last_line_error = 0


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, old):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, old)
    return now - old


def init_uart1():
    fm.register(UART1_TX_PIN, fm.fpioa.UART1_TX, force=True)
    fm.register(UART1_RX_PIN, fm.fpioa.UART1_RX, force=True)

    # MaixPy firmware versions differ slightly in UART constructor support.
    # Try the full form first, then fall back to the common minimal form.
    try:
        return UART(UART.UART1, UART_BAUDRATE, 8, None, 1, timeout=1000, read_buf_len=256)
    except TypeError:
        return UART(UART.UART1, UART_BAUDRATE, 8, 0, 1)


def write_uart(uart, frame):
    if uart is None:
        return

    try:
        uart.write(frame)
    except TypeError:
        uart.write(frame.encode())


def send_line(uart, line):
    # 巡线帧协议:
    #   $L,error,angle,confidence,flags#
    #
    # error:
    #   黑线中心相对画面中心的横向偏差，左负右正。这里已经乘 ERROR_SCALE，
    #   对齐 STM32 当前按 K230/VGA 调出来的控制尺度。
    #
    # angle:
    #   远处 ROI 和近处 ROI 的偏差差值，用来表达前方路线趋势。
    #
    # confidence:
    #   0~100，越高表示命中的 ROI 越多、黑线像素越可靠。
    #
    # flags:
    #   bit0 valid, bit1 lost, bit2 curve。
    frame = "$L,{},{},{},{}#\r\n".format(
        line["error"],
        line["angle"],
        line["confidence"],
        line["flags"],
    )
    write_uart(uart, frame)


def send_color(uart, color_id, pos, size):
    # 色块避障帧协议:
    #   $C,color_id,pos,size#
    #
    # color_id:
    #   0 表示当前没有检测到目标，1/2/3 分别表示红/绿/蓝。
    #
    # pos:
    #   L/C/R/N，N 只和 color_id=0 一起使用，表示无目标。
    #
    # size:
    #   目标像素面积。K210 端 QVGA 面积会乘 4 后上报，使 STM32 侧的
    #   K230_AVOID_MIN_SIZE 不需要因为分辨率变化立即调整。
    write_uart(uart, "$C,{},{},{}#\r\n".format(color_id, pos, size))


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def blob_pixels(blob):
    if hasattr(blob, "pixels"):
        return blob.pixels()
    return blob[4]


def blob_rect(blob):
    if hasattr(blob, "rect"):
        return blob.rect()
    return blob[0], blob[1], blob[2], blob[3]


def blob_center(blob):
    if hasattr(blob, "cx"):
        return blob.cx(), blob.cy()
    x, y, w, h = blob_rect(blob)
    return x + w // 2, y + h // 2


def draw_rect(img, rect, color):
    try:
        img.draw_rectangle(rect, color=color, thickness=2)
    except TypeError:
        img.draw_rectangle(rect, color=color)


def draw_cross(img, x, y, color):
    try:
        img.draw_cross(x, y, color=color, size=8, thickness=2)
    except TypeError:
        img.draw_cross(x, y, color=color)


def draw_text(img, x, y, text, color=(255, 255, 255), scale=1):
    try:
        img.draw_string(x, y, text, color=color, scale=scale)
    except TypeError:
        img.draw_string(x, y, text, color=color)


def classify_pos(cx):
    # 把目标中心点映射成 STM32 避障逻辑需要的三段式位置。
    # 不直接上报坐标，是为了让下位机保持简单的高层策略：
    # 左侧障碍右绕，右侧障碍左绕，中间障碍按固定方向绕开。
    if cx < LEFT_BOUND:
        return "L"
    if cx > RIGHT_BOUND:
        return "R"
    return "C"


def find_line_candidate(img, roi, expected_error):
    x, y, w, h, weight = roi
    blobs = img.find_blobs(
        [LINE_THRESHOLD],
        roi=(x, y, w, h),
        pixels_threshold=LINE_MIN_PIXELS,
        area_threshold=LINE_MIN_AREA,
        merge=True,
    )
    if not blobs:
        return None

    best = None
    best_score = -1000000
    for blob in blobs:
        pixels = blob_pixels(blob)
        bx, by, bw, bh = blob_rect(blob)
        cx, cy = blob_center(blob)
        error = cx - (FRAME_WIDTH // 2)

        continuity_penalty = abs(error - expected_error) * 4
        shape_penalty = 0
        if bw < 3 or bh < 3:
            shape_penalty += 300
        if abs(error - expected_error) > LINE_MAX_JUMP:
            shape_penalty += 500

        score = pixels - continuity_penalty - shape_penalty
        if score > best_score:
            best_score = score
            best = {
                "cx": cx,
                "cy": cy,
                "error": error,
                "pixels": pixels,
                "rect": (bx, by, bw, bh),
                "weight": weight,
            }

    if best is not None:
        draw_rect(img, best["rect"], (255, 255, 0))
        draw_cross(img, best["cx"], best["cy"], (255, 255, 0))

    return best


def find_line(img):
    global last_line_error

    weighted_error = 0
    total_weight = 0
    centers = []
    expected_error = last_line_error

    for roi in LINE_ROIS:
        x, y, w, h, weight = roi
        draw_rect(img, (x, y, w, h), (80, 80, 80))
        candidate = find_line_candidate(img, roi, expected_error)
        if candidate is None:
            continue

        weighted_error += candidate["error"] * candidate["weight"]
        total_weight += candidate["weight"]
        centers.append(candidate)
        expected_error = candidate["error"]

    if total_weight == 0:
        return {
            "error": last_line_error * ERROR_SCALE,
            "angle": 0,
            "confidence": 0,
            "flags": LINE_FLAG_LOST,
        }

    error = int(weighted_error / total_weight)
    angle = 0
    if len(centers) >= 2:
        near = centers[0]
        far = centers[-1]
        angle = int(far["error"] - near["error"])

    confidence = 35 + len(centers) * 18
    pixel_sum = sum([item["pixels"] for item in centers])
    confidence += min(20, pixel_sum // 125)
    confidence = clamp(confidence, 0, 100)

    flags = LINE_FLAG_VALID
    if abs(angle * ERROR_SCALE) > LINE_CURVE_THRESHOLD:
        flags |= LINE_FLAG_CURVE

    last_line_error = error
    return {
        "error": error * ERROR_SCALE,
        "angle": angle * ERROR_SCALE,
        "confidence": confidence,
        "flags": flags,
    }


def find_best_target(img):
    # 在红/绿/蓝三种颜色中，只选择本帧面积最大的目标上报。
    #
    # 这样做的原因:
    #   1. STM32 侧当前避障策略一次只处理一个障碍。
    #   2. 面积最大的色块通常距离车辆最近或最需要优先避让。
    #   3. 串口协议保持短帧，避免中断解析负担过重。
    best = None

    for index, threshold in enumerate(COLOR_THRESHOLDS):
        blobs = img.find_blobs(
            [threshold],
            pixels_threshold=COLOR_MIN_PIXELS,
            area_threshold=COLOR_MIN_AREA,
            merge=True,
        )
        if not blobs:
            continue

        blob = max(blobs, key=blob_pixels)
        pixels = blob_pixels(blob)
        x, y, w, h = blob_rect(blob)
        cx, cy = blob_center(blob)

        draw_rect(img, (x, y, w, h), DRAW_COLORS[index])
        draw_cross(img, cx, cy, DRAW_COLORS[index])

        if best is None or pixels > best["pixels"]:
            best = {
                "color_id": index + 1,
                "pixels": pixels,
                "cx": cx,
                "cy": cy,
            }

    if best is None:
        return None

    return {
        "color_id": best["color_id"],
        "pos": classify_pos(best["cx"]),
        "size": best["pixels"] * ERROR_SCALE * ERROR_SCALE,
    }


def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(CAMERA_HMIRROR)
    sensor.set_vflip(CAMERA_VFLIP)
    sensor.skip_frames(time=2000)
    try:
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
    except Exception:
        pass


def main():
    uart = init_uart1()
    lcd.init()
    init_camera()
    clock = time.clock()

    last_line_send_ms = 0
    last_color_send_ms = 0
    last_color_frame = None
    last_line_frame = None
    last_debug_print_ms = 0

    while True:
        clock.tick()
        img = sensor.snapshot()
        now = ticks_ms()

        line = find_line(img)
        line_frame = (line["error"], line["angle"], line["confidence"], line["flags"])
        if (line_frame != last_line_frame) or (ticks_diff(now, last_line_send_ms) >= SEND_LINE_INTERVAL_MS):
            send_line(uart, line)
            last_line_send_ms = now
            last_line_frame = line_frame

        draw_text(
            img,
            5,
            5,
            "L e:{} a:{} c:{} f:{}".format(line["error"], line["angle"], line["confidence"], line["flags"]),
            scale=1,
        )

        # 色块避障低频执行。
        #
        # 巡线是基础控制输入，需要较高刷新率；色块避障只需要让 STM32 周期性知道
        # 前方是否有明显障碍，因此按 SEND_COLOR_INTERVAL_MS 执行即可。
        # 每次扫描都会发送一帧:
        #   有目标: $C,1,L,356#
        #   无目标: $C,0,N,0#
        # 无目标帧很重要，它能让 STM32 及时退出避障，回到巡线控制。
        if ENABLE_COLOR_AVOIDANCE and ticks_diff(now, last_color_send_ms) >= SEND_COLOR_INTERVAL_MS:
            target = find_best_target(img)
            if target is None:
                color_frame = (0, "N", 0)
            else:
                color_frame = (target["color_id"], target["pos"], target["size"])
                draw_text(
                    img,
                    5,
                    20,
                    "{} {} {}".format(COLOR_NAMES[target["color_id"]], target["pos"], target["size"]),
                    scale=1,
                )

            if color_frame != last_color_frame or ticks_diff(now, last_color_send_ms) >= SEND_COLOR_INTERVAL_MS:
                send_color(uart, color_frame[0], color_frame[1], color_frame[2])
                last_color_frame = color_frame
            last_color_send_ms = now

        lcd.display(img)
        if ticks_diff(now, last_debug_print_ms) >= DEBUG_PRINT_INTERVAL_MS:
            print("fps={}, line={}".format(clock.fps(), last_line_frame))
            last_debug_print_ms = now


try:
    main()
except Exception as e:
    print("Exception:", e)
