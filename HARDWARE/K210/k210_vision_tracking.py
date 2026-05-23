import os
import time

import image
import lcd
import sensor

try:
    from machine import UART
except ImportError:
    UART = None

try:
    from fpioa_manager import fm
except ImportError:
    fm = None


# 串口配置
UART_PORT = 1
UART_BAUDRATE = 115200

# 按硬件图接线：K210 UART1
# U1 TX -> IO7
# U1 RX -> IO9
UART1_TX_PIN = 7
UART1_RX_PIN = 9

# 图像与 ROI
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

ROI_LIST = [
    {"name": "far", "roi": (0, 110, 640, 60), "weight": 0.2},
    {"name": "mid", "roi": (0, 210, 640, 70), "weight": 0.3},
    {"name": "near", "roi": (0, 310, 640, 100), "weight": 0.5},
]

# 黑线提取阈值，按 LAB 空间做黑色区域筛选
BLACK_THRESHOLD = (0, 45, -20, 20, -20, 20)

# 颜色块识别阈值，顺序为红、绿、蓝
COLOR_THRESHOLDS = [
    (0, 80, 20, 127, -10, 30),
    (0, 80, -120, -10, 0, 30),
    (0, 80, 0, 90, -128, -20),
]

COLOR_DRAW_COLORS = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
]

COLOR_NAMES = ("NONE", "RED", "GREEN", "BLUE")

# blob 过滤阈值
MIN_PIXELS = 120
MIN_AREA = 120
MIN_COLOR_PIXELS = 200
MIN_COLOR_AREA = 200

# 调试与输出
ENABLE_DISPLAY = True
ENABLE_DRAW = True
PRINT_INTERVAL_MS = 500

# 画面方向修正
# 1 = 打开，0 = 关闭
IMAGE_HMIRROR = 1
IMAGE_VFLIP = 1

# 运行状态
# 这份字典保存上一帧的巡线结果，用于：
# 1. 估算下一帧的候选中心位置，减少跳变
# 2. 做轻微滤波，抑制单帧噪声
# 3. 在丢线时保留最近一次可信状态
g_line_state = {
    "error": 0,
    "angle": 0,
    "confidence": 0,
    "flags": 0,
    "valid": 0,
    "lost": 1,
    "curve": 0,
    "marker": 0,
    "center_x": FRAME_WIDTH // 2,
    "last_update_ms": 0,
}


def clamp_int(value, low, high):
    """把数值限制在指定区间内，避免后面组帧时出现越界值。"""
    if value < low:
        return low
    if value > high:
        return high
    return value


def resolve_attr(obj, names):
    """按候选名称顺序查找属性，兼容不同固件命名差异。"""
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def blob_pixels(blob):
    """兼容不同 blob 对象写法，统一读取面积/像素数。"""
    if hasattr(blob, "pixels"):
        return blob.pixels()
    return blob[4]


def blob_rect(blob):
    """统一读取 blob 的矩形框 (x, y, w, h)。"""
    if hasattr(blob, "rect"):
        return blob.rect()
    return blob[0], blob[1], blob[2], blob[3]


def blob_center(blob):
    """统一读取 blob 中心点坐标。"""
    if hasattr(blob, "cx") and hasattr(blob, "cy"):
        return blob.cx(), blob.cy()
    x, y, w, h = blob_rect(blob)
    return x + w // 2, y + h // 2


def blob_score(blob, expected_x, roi_h):
    """给黑线候选区域打分。

    分数越高，越像当前应该跟踪的主线。
    评分考虑三个因素：
    1. 像素数足不足
    2. 形状是不是细长线条
    3. 中心位置是否接近上一帧的预测位置
    """
    x, y, w, h = blob_rect(blob)
    cx, _ = blob_center(blob)
    pixels = blob_pixels(blob)

    if w <= 0 or h <= 0:
        return -1

    long_side = float(max(w, h))
    short_side = float(max(1, min(w, h)))
    aspect = long_side / short_side

    # 过于接近正方形的区域更像圆块、噪声或内圆边缘，不适合当主线。
    if aspect < 1.3:
        return -1

    # 黑线通常细长，圆块或噪声一般更接近正方形。
    shape_bonus = int(max(0.0, aspect - 1.0) * 120)
    distance_penalty = int(abs(cx - expected_x) * 6)
    height_penalty = int(max(0.0, h - roi_h * 0.85) * 6)

    return pixels + shape_bonus - distance_penalty - height_penalty


def classify_pos(cx):
    """把颜色块的中心点换成左/中/右位置标记。"""
    if cx < FRAME_WIDTH // 3:
        return "L"
    if cx > FRAME_WIDTH * 2 // 3:
        return "R"
    return "C"


def configure_uart1_pins():
    """把 UART1 映射到硬件图上的 IO7/IO9。

    这一步只负责引脚复用，不负责串口对象创建。
    先复用，再初始化 UART，能兼容更多固件版本。
    """
    if fm is not None:
        try:
            tx_func = resolve_attr(fm.fpioa, ("UART1_TX", "UART1_TXD"))
            rx_func = resolve_attr(fm.fpioa, ("UART1_RX", "UART1_RXD"))
            if tx_func is not None and rx_func is not None:
                fm.register(UART1_TX_PIN, tx_func, force=True)
                fm.register(UART1_RX_PIN, rx_func, force=True)
                print("UART1 pinmux configured by fm: TX=IO{}, RX=IO{}".format(UART1_TX_PIN, UART1_RX_PIN))
                return True
        except Exception as e:
            print("fm pinmux failed: {}".format(e))

    print("UART1 pinmux not configured by script, check board default mapping")
    return False


def init_uart():
    """初始化 UART1。

    这里尝试几种常见构造方式，尽量兼容不同 CanMV / MaixPy 固件版本。
    如果串口模块不存在，就只做视觉，不发送下位机数据。
    """
    if UART is None:
        print("UART module not found, serial output disabled")
        return None

    configure_uart1_pins()

    uart_id = getattr(UART, "UART1", UART_PORT)

    for args, kwargs in (
        ((uart_id, UART_BAUDRATE), {}),
        ((uart_id,), {"baudrate": UART_BAUDRATE, "bits": 8, "parity": None, "stop": 1}),
        ((uart_id, UART_BAUDRATE, 8, None, 1), {}),
    ):
        try:
            return UART(*args, **kwargs)
        except TypeError:
            continue
        except Exception as e:
            print("UART1 init attempt failed: {}".format(e))

    print("UART1 init failed, serial output disabled")
    return None


def init_sensor():
    """初始化摄像头传感器。

    这里统一设置：
    1. RGB565 作为图像格式，便于黑线和颜色块处理
    2. VGA 分辨率，兼顾视野和计算量
    3. 画面翻转参数，适配当前摄像头安装方向
    """
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_hmirror(IMAGE_HMIRROR)
    sensor.set_vflip(IMAGE_VFLIP)
    sensor.skip_frames(time=2000)

    return sensor


def select_best_blob(blobs, expected_x, roi_h):
    """在同一个 ROI 的多个候选 blob 中，选出最像主线的那个。"""
    best = None
    best_score = -1

    for blob in blobs:
        score = blob_score(blob, expected_x, roi_h)
        if score < 0:
            continue

        if score > best_score:
            x, y, w, h = blob_rect(blob)
            cx, cy = blob_center(blob)
            best = {
                "rect": (x, y, w, h),
                "cx": cx,
                "cy": cy,
                "pixels": blob_pixels(blob),
                "score": score,
            }
            best_score = score

    return best


def find_best_color(img):
    """在整幅图里寻找红、绿、蓝三类色块，并返回最大的那个。

    这个函数只做颜色识别本身，不参与巡线误差计算。
    """
    best = None

    for index, threshold in enumerate(COLOR_THRESHOLDS):
        blobs = img.find_blobs(
            [threshold],
            pixels_threshold=MIN_COLOR_PIXELS,
            area_threshold=MIN_COLOR_AREA,
            merge=True,
        )
        if not blobs:
            continue

        blob = max(blobs, key=blob_pixels)
        pixels = blob_pixels(blob)
        x, y, w, h = blob_rect(blob)
        cx, cy = blob_center(blob)

        if ENABLE_DRAW:
            img.draw_rectangle((x, y, w, h), color=COLOR_DRAW_COLORS[index], thickness=4)
            img.draw_cross(cx, cy, color=COLOR_DRAW_COLORS[index], size=12, thickness=2)

        if best is None or pixels > best["pixels"]:
            best = {
                "color_id": index + 1,
                "pixels": pixels,
                "cx": cx,
                "cy": cy,
                "pos": classify_pos(cx),
                "rect": (x, y, w, h),
            }

    if best is None:
        return {
            "color_id": 0,
            "pixels": 0,
            "cx": 0,
            "cy": 0,
            "pos": "N",
            "rect": None,
        }

    return best


def analyze_line(img):
    """分析当前帧中的黑线位置，输出巡线控制所需的数据。

    处理流程：
    1. 逐个 ROI 找黑线候选
    2. 选出每个 ROI 的最优 blob
    3. 按近中远权重融合中心位置
    4. 计算 error / angle / confidence / flags
    """
    image_center_x = FRAME_WIDTH // 2
    expected_x = g_line_state["center_x"] if g_line_state["valid"] else image_center_x
    samples = []

    for spec in ROI_LIST:
        blobs = img.find_blobs(
            [BLACK_THRESHOLD],
            roi=spec["roi"],
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_AREA,
            merge=True,
        )

        best = select_best_blob(blobs, expected_x, spec["roi"][3])
        if best is None:
            continue

        best["name"] = spec["name"]
        best["weight"] = spec["weight"]
        samples.append(best)
        expected_x = best["cx"]

        if ENABLE_DRAW:
            x, y, w, h = best["rect"]
            img.draw_rectangle((x, y, w, h), color=(0, 255, 0), thickness=2)
            img.draw_cross(best["cx"], best["cy"], color=(255, 0, 0), size=10, thickness=2)

    if not samples:
        # 没找到任何候选线段，直接视为丢线。
        return {
            "error": 0,
            "angle": 0,
            "confidence": 0,
            "flags": 0x02,
            "valid": 0,
            "lost": 1,
            "curve": 0,
            "marker": 0,
            "center_x": image_center_x,
            "sample_count": 0,
        }

    weight_sum = 0.0
    weighted_center = 0.0
    total_pixels = 0
    total_aspect = 0.0
    far_x = None
    near_x = None

    for sample in samples:
        x, y, w, h = sample["rect"]
        long_side = float(max(w, h))
        short_side = float(max(1, min(w, h)))
        aspect = long_side / short_side

        weighted_center += sample["cx"] * sample["weight"]
        weight_sum += sample["weight"]
        total_pixels += sample["pixels"]
        total_aspect += aspect

        if sample["name"] == "far":
            far_x = sample["cx"]
        elif sample["name"] == "near":
            near_x = sample["cx"]

    if weight_sum <= 0:
        center_x = image_center_x
    else:
        center_x = int(weighted_center / weight_sum)

    error = center_x - image_center_x

    if far_x is not None and near_x is not None:
        # far 和 near 都存在时，直接用前后两层 ROI 的偏移差估计弯道趋势。
        angle = far_x - near_x
    elif len(samples) >= 2:
        angle = samples[0]["cx"] - samples[-1]["cx"]
    else:
        angle = 0

    # 轻微滤波，减少单帧抖动
    if g_line_state["valid"]:
        # 上一帧可信时才做历史融合；丢线恢复阶段不引入旧值，避免拖偏。
        error = int(g_line_state["error"] * 0.35 + error * 0.65)
        angle = int(g_line_state["angle"] * 0.35 + angle * 0.65)

    avg_pixels = float(total_pixels) / float(len(samples))
    avg_aspect = float(total_aspect) / float(len(samples))

    confidence = 10 + len(samples) * 18
    confidence += min(20, int(avg_pixels / 45))
    confidence += min(20, int(max(0.0, avg_aspect - 1.0) * 8))
    confidence -= min(15, abs(error) // 16)
    confidence = clamp_int(confidence, 0, 100)

    valid = 1
    lost = 0
    curve = 1 if (abs(angle) >= 18 or abs(error) >= 60) else 0
    marker = 0
    flags = (valid << 0) | (lost << 1) | (curve << 2) | (marker << 3)

    return {
        "error": int(error),
        "angle": int(angle),
        "confidence": int(confidence),
        "flags": int(flags),
        "valid": valid,
        "lost": lost,
        "curve": curve,
        "marker": marker,
        "center_x": int(center_x),
        "sample_count": len(samples),
    }


def update_line_history(line):
    """把当前帧巡线结果写回历史状态，用于下一帧预测和滤波。"""
    g_line_state["error"] = line["error"]
    g_line_state["angle"] = line["angle"]
    g_line_state["confidence"] = line["confidence"]
    g_line_state["flags"] = line["flags"]
    g_line_state["valid"] = line["valid"]
    g_line_state["lost"] = line["lost"]
    g_line_state["curve"] = line["curve"]
    g_line_state["marker"] = line["marker"]
    g_line_state["center_x"] = line["center_x"]
    g_line_state["last_update_ms"] = time.ticks_ms()


def send_line_frame(uart, line):
    """按照 `$L,error,angle,confidence,flags#` 发送巡线结果。"""
    if uart is None:
        return

    frame = "$L,{},{},{},{}#\r\n".format(
        line["error"],
        line["angle"],
        line["confidence"],
        line["flags"],
    )

    try:
        uart.write(frame)
    except TypeError:
        uart.write(frame.encode())


def send_color_frame(uart, color):
    """按照 `$C,color_id,pos,size#` 发送颜色块识别结果。"""
    if uart is None:
        return

    frame = "$C,{},{},{}#\r\n".format(
        color["color_id"],
        color["pos"],
        color["pixels"],
    )

    try:
        uart.write(frame)
    except TypeError:
        uart.write(frame.encode())


def draw_status(img, line, color):
    """把调试信息叠加到画面上，方便现场看识别状态。"""
    if not ENABLE_DRAW:
        return

    for spec in ROI_LIST:
        x, y, w, h = spec["roi"]
        img.draw_rectangle((x, y, w, h), color=(0, 0, 255), thickness=1)

    status = "E:{} A:{} C:{} F:{:02X}".format(
        line["error"],
        line["angle"],
        line["confidence"],
        line["flags"],
    )
    img.draw_string(10, 10, status, color=(255, 255, 255), scale=2)

    if line["lost"]:
        img.draw_string(10, 40, "LOST", color=(255, 0, 0), scale=2)
    elif line["curve"]:
        img.draw_string(10, 40, "CURVE", color=(255, 255, 0), scale=2)
    else:
        img.draw_string(10, 40, "TRACK", color=(0, 255, 0), scale=2)

    color_text = "C:{} {} {}".format(
        COLOR_NAMES[color["color_id"]],
        color["pos"],
        color["pixels"],
    )
    img.draw_string(10, 70, color_text, color=(255, 255, 255), scale=2)


def main():
    """程序主入口。

    主循环做四件事：
    1. 采图
    2. 跑巡线和色块识别
    3. 发串口结果给 STM32
    4. 把调试信息显示到屏幕和 IDE
    """
    uart = None
    last_print_ms = 0

    try:
        uart = init_uart()

        init_sensor()
        lcd.init()

        clock = time.clock()
        time.sleep_ms(300)

        while True:
            clock.tick()

            img = sensor.snapshot()
            line = analyze_line(img)
            color = find_best_color(img)
            update_line_history(line)
            send_line_frame(uart, line)
            send_color_frame(uart, color)
            draw_status(img, line, color)

            if ENABLE_DISPLAY:
                lcd.display(img)

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print_ms) >= PRINT_INTERVAL_MS:
                print("fps={}, line={}, color={}".format(clock.fps(), line, color))
                last_print_ms = now

    except KeyboardInterrupt as e:
        print("user stop:", e)
    except BaseException as e:
        print("Exception {}".format(e))
    finally:
        time.sleep_ms(100)


main()
