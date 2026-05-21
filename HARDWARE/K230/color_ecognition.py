import time
import os
from media.sensor import *
from media.display import *
from media.media import *

# 串口类。用于把识别结果通过 UART1 发给下位机 STM32。
try:
    from machine import UART
except ImportError:
    UART = None

# FPIOA 用于给 K230 的物理 IO 口分配片上外设功能。
# 这里需要把 GPIO40/41 复用成 UART1 的 TX/RX。
try:
    from machine import FPIOA
except ImportError:
    FPIOA = None

# 某些固件环境没有 machine.FPIOA，而是提供 fpioa_manager。
# 这里保留兼容处理，避免脚本因为固件版本差异直接无法运行。
try:
    from fpioa_manager import fm
except ImportError:
    fm = None


# LAB 颜色阈值，顺序依次为红、绿、蓝。
# 每个元组格式通常为:
# (L_min, L_max, A_min, A_max, B_min, B_max)
# 后续 find_blobs() 会按这些阈值去找色块。
thresholds = [
    (0, 80, 20, 127, -10, 30),
    (0, 80, -120, -10, 0, 30),
    (0, 80, 0, 90, -128, -20),
]

# 在 LCD/IDE 画面上给三种颜色框分别使用的显示色。
draw_colors = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
]

# UART1 基本配置。
# 这里的 UART_PORT=1 对应 UART1。
UART_PORT = 1
UART_BAUDRATE = 115200

# K230 引脚分配:
# GPIO40 -> UART1_TXD
# GPIO41 -> UART1_RXD
# 这与你给出的参考初始化代码保持一致。
UART1_TX_PIN = 40
UART1_RX_PIN = 41

# 图像宽度按 VGA 显示宽度 640 计算。
# 为了输出“左/中/右”位置，把整幅图横向划分成 3 个区域。
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
LEFT_BOUND = FRAME_WIDTH // 3
RIGHT_BOUND = FRAME_WIDTH * 2 // 3

# 色块筛选下限。
# 太小的噪点不参与识别，减少误检。
MIN_PIXELS = 200
MIN_AREA = 200

# 串口最大发送间隔。
# 即便结果没有变化，也会每 100ms 重发一次，方便下位机持续接收最新状态。
SEND_INTERVAL_MS = 100
SEND_LINE_INTERVAL_MS = 50
ENABLE_COLOR_REPORT = False

# 白底黑线巡线阈值。
#
# find_blobs() 在 RGB565 图像上使用 LAB 阈值查找目标。这里把 A/B 范围放宽，
# 主要用 L 亮度筛出黑线。地图是白底黑线，因此黑线通常表现为低亮度区域。
# 现场光照变化时优先调整 L_MAX：
#   L_MAX 太小：黑线断裂，confidence 降低，容易 lost。
#   L_MAX 太大：阴影、深色色块也可能被当成黑线。
LINE_THRESHOLD = (0, 45, -128, 127, -128, 127)

# 多个横向 ROI 分别观察近处、中处、远处黑线。
#
# 元组格式: (x, y, w, h, weight)
#   x/y/w/h : ROI 在 VGA 画面中的位置和尺寸
#   weight  : 该 ROI 对最终 error 的权重
#
# 近处 ROI 权重大，用来保证车当前不偏离主线；中远处 ROI 权重较小，
# 但能提前看到圆角趋势。这样比单独看画面底部更适合当前圆角矩形地图。
LINE_ROIS = [
    (0, 330, FRAME_WIDTH, 50, 50),
    (0, 250, FRAME_WIDTH, 45, 30),
    (0, 170, FRAME_WIDTH, 45, 20),
]

# 黑线候选区域过滤参数。
#
# LINE_MIN_PIXELS / LINE_MIN_AREA:
#   过滤小噪点，避免色块边缘、阴影或图像噪声被误选为线。
#
# LINE_MAX_JUMP:
#   限制本 ROI 候选线相对上一 ROI/上一帧的横向跳变。底部内圆和左侧短横线
#   进入画面时，往往会和主线位置不连续，用这个约束优先保留外侧闭环主线。
#
# LINE_CURVE_THRESHOLD:
#   angle 超过该阈值时置 curve 标志，STM32 当前只用于调试保留，后续可以
#   用来在弯道自动降低速度或提高转向权重。
LINE_MIN_PIXELS = 80
LINE_MIN_AREA = 80
LINE_MAX_JUMP = 160
LINE_CURVE_THRESHOLD = 35
LINE_FLAG_VALID = 0x01
LINE_FLAG_LOST = 0x02
LINE_FLAG_CURVE = 0x04

# 颜色 ID 到文本名称的映射。
# 0 表示未检测到目标。
COLOR_NAMES = ("NONE", "RED", "GREEN", "BLUE")


def configure_uart1_pins():
    # 优先使用 machine.FPIOA 进行引脚复用。
    # 这是比较直接的 K230 配置方式，等价于：
    # fpioa.set_function(40, FPIOA.UART1_TXD)
    # fpioa.set_function(41, FPIOA.UART1_RXD)
    if FPIOA is not None:
        try:
            fpioa = FPIOA()
            tx_func = getattr(FPIOA, "UART1_TXD", None)
            rx_func = getattr(FPIOA, "UART1_RXD", None)
            if tx_func is not None and rx_func is not None:
                fpioa.set_function(UART1_TX_PIN, tx_func)
                fpioa.set_function(UART1_RX_PIN, rx_func)
                print("UART1 pinmux configured by FPIOA: TX=GPIO{}, RX=GPIO{}".format(UART1_TX_PIN, UART1_RX_PIN))
                return True
        except Exception as e:
            print("FPIOA pinmux failed: {}".format(e))

    # 如果当前固件没有提供 machine.FPIOA，
    # 则回退到 fpioa_manager 的注册方式。
    if fm is not None:
        try:
            tx_func = getattr(fm.fpioa, "UART1_TXD", None)
            rx_func = getattr(fm.fpioa, "UART1_RXD", None)
            if tx_func is not None and rx_func is not None:
                fm.register(UART1_TX_PIN, tx_func, force=True)
                fm.register(UART1_RX_PIN, rx_func, force=True)
                print("UART1 pinmux configured by fm: TX=GPIO{}, RX=GPIO{}".format(UART1_TX_PIN, UART1_RX_PIN))
                return True
        except Exception as e:
            print("fm pinmux failed: {}".format(e))

    # 两种方式都不可用时，不中断脚本运行。
    # 这通常意味着：
    # 1. 板级固件已经提前默认完成了 UART1 引脚映射
    # 2. 或者当前固件环境不支持脚本侧动态复用，需要你手动确认
    print("UART1 pinmux not configured by script, check board default mapping")
    return False


def init_uart():
    # 没有 UART 模块时直接关闭串口发送功能，视觉识别仍可继续跑。
    if UART is None:
        print("UART module not found, serial output disabled")
        return None

    # 先做引脚复用，再创建 UART1 对象。
    configure_uart1_pins()

    uart_id = getattr(UART, "UART1", UART_PORT)

    # 不同版本的 MicroPython / K230 SDK 对 UART() 构造参数格式可能不完全一样。
    # 这里按多种常见方式依次尝试，提升脚本兼容性。
    for args, kwargs in (
        ((uart_id, UART_BAUDRATE), {}),
        ((uart_id,), {"baudrate": UART_BAUDRATE, "bits": 8, "parity": None, "stop": 1}),
        ((uart_id, UART_BAUDRATE, 8, None, 1), {}),
    ):
        try:
            return UART(*args, **kwargs)
        except TypeError:
            continue

    print("UART1 init failed, serial output disabled")
    return None


def blob_pixels(blob):
    # 兼容不同 blob 对象实现：
    # 有的固件版本提供 blob.pixels() 方法，
    # 有的直接使用元组下标访问像素数。
    if hasattr(blob, "pixels"):
        return blob.pixels()
    return blob[4]


def blob_rect(blob):
    # 返回色块外接矩形 (x, y, w, h)。
    if hasattr(blob, "rect"):
        return blob.rect()
    return blob[0], blob[1], blob[2], blob[3]


def blob_center(blob):
    # 返回色块中心点坐标。
    # 优先调用固件内置方法，没有的话就自己根据矩形计算中心点。
    if hasattr(blob, "cx"):
        return blob.cx(), blob.cy()

    x, y, w, h = blob_rect(blob)
    return x + w // 2, y + h // 2


def classify_pos(cx):
    # 根据中心点横坐标，把目标分类成左/中/右三个区域。
    if cx < LEFT_BOUND:
        return "L"
    if cx > RIGHT_BOUND:
        return "R"
    return "C"


def send_result(uart, color_id, pos, size):
    # 串口未初始化成功时，直接跳过发送。
    if uart is None:
        return

    # 发给 STM32 的协议帧格式：
    # $C,颜色ID,位置,面积#
    # 示例：
    # $C,1,L,356#
    # 含义：检测到红色目标，位于左侧，像素面积 356。
    frame = "$C,{},{},{}#\r\n".format(color_id, pos, size)
    try:
        # 有些环境 uart.write() 支持直接写字符串。
        uart.write(frame)
    except TypeError:
        # 有些环境只接受 bytes，因此做一次编码回退。
        uart.write(frame.encode())


def send_line(uart, line):
    # 巡线帧使用 $L 前缀，与色块 $C 帧区分。
    # STM32 侧只需要 error/angle/confidence/flags，不需要接收图像坐标数组，
    # 这样可以降低串口带宽和下位机解析复杂度。
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


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


last_line_error = 0


def find_line_candidate(img, roi, expected_error):
    # 在单个 ROI 中选择最可信的黑线候选区域。
    #
    # 当前地图存在底部内圆和左侧短横线，简单选择“最大黑色 blob”不可靠。
    # 因此这里把候选区域面积、与上一位置的连续性、形状合理性一起评分。
    # expected_error 来自上一帧或上一个 ROI，用于表达“主线应该大致连续”。
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

        # Prefer a continuous main line. This suppresses the inner circle and
        # short side marker when they enter the ROI together with the track.
        continuity_penalty = abs(error - expected_error) * 4
        shape_penalty = 0
        if bw < 4 or bh < 4:
            shape_penalty += 500
        if abs(error - expected_error) > LINE_MAX_JUMP:
            shape_penalty += 800

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
        img.draw_rectangle(best["rect"], color=(255, 255, 0), thickness=2)
        img.draw_cross(best["cx"], best["cy"], color=(255, 255, 0), size=8, thickness=2)

    return best


def find_line(img):
    # 输出一帧视觉巡线摘要。
    #
    # 返回字典字段：
    #   error      : 加权后的黑线中心偏差，左负右正
    #   angle      : 远处和近处偏差差值，用于表示路线趋势
    #   confidence : 识别可信度，STM32 低于阈值时不会接管巡线
    #   flags      : valid/lost/curve 等状态位
    #
    # 这里故意不把旧八路传感器的“状态组合表”搬过来，因为视觉识别能提供
    # 连续偏差，直接做比例/趋势控制更自然，也更适合圆角路线。
    global last_line_error

    weighted_error = 0
    total_weight = 0
    centers = []
    expected_error = last_line_error

    for roi in LINE_ROIS:
        x, y, w, h, weight = roi
        img.draw_rectangle((x, y, w, h), color=(80, 80, 80), thickness=1)
        candidate = find_line_candidate(img, roi, expected_error)
        if candidate is None:
            continue

        weighted_error += candidate["error"] * candidate["weight"]
        total_weight += candidate["weight"]
        centers.append(candidate)
        expected_error = candidate["error"]

    if total_weight == 0:
        # 没有任何 ROI 找到黑线时，保留上一帧 error 只用于显示和短时调试。
        # flags 置 lost，STM32 会据此拒绝接管控制，避免长期沿用旧偏差。
        return {
            "error": last_line_error,
            "angle": 0,
            "confidence": 0,
            "flags": LINE_FLAG_LOST,
        }

    error = int(weighted_error / total_weight)
    angle = 0
    if len(centers) >= 2:
        # 近处点代表当前车头附近的偏差，远处点代表前方路线发展方向。
        # 二者差值可近似表示入弯趋势，比只用 error 更早发现圆角。
        near = centers[0]
        far = centers[-1]
        angle = int(far["error"] - near["error"])

    # confidence 同时考虑“命中的 ROI 数量”和“黑线像素规模”。
    # 多个 ROI 都连续看到黑线时，说明当前主线选择更可信。
    confidence = 35 + len(centers) * 18
    pixel_sum = sum([item["pixels"] for item in centers])
    confidence += min(20, pixel_sum // 500)
    confidence = clamp(confidence, 0, 100)

    flags = LINE_FLAG_VALID
    if abs(angle) > LINE_CURVE_THRESHOLD:
        flags |= LINE_FLAG_CURVE

    last_line_error = error
    return {
        "error": error,
        "angle": angle,
        "confidence": confidence,
        "flags": flags,
    }


def find_best_target(img):
    # 在红/绿/蓝三类目标中，最终只保留“最大色块”作为本帧结果。
    # 这样下位机每次只处理一个目标，协议更简单。
    best = None

    for index, threshold in enumerate(thresholds):
        # 在当前阈值下查找色块。
        # merge=True 用于合并邻近小块，降低目标被分裂成多个 blob 的概率。
        blobs = img.find_blobs(
            [threshold],
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_AREA,
            merge=True,
        )
        if not blobs:
            continue

        # 当前颜色只取最大色块。
        blob = max(blobs, key=blob_pixels)
        pixels = blob_pixels(blob)
        x, y, w, h = blob_rect(blob)
        cx, cy = blob_center(blob)

        # 在图像上画框和十字，便于调试观察识别是否稳定。
        img.draw_rectangle((x, y, w, h), color=draw_colors[index], thickness=4)
        img.draw_cross(cx, cy, color=draw_colors[index], size=12, thickness=2)

        # 如果当前色块比之前记录的目标更大，则更新最终目标。
        if best is None or pixels > best["pixels"]:
            best = {
                "color_id": index + 1,
                "pixels": pixels,
                "cx": cx,
                "cy": cy,
                "rect": (x, y, w, h),
            }

    return best


sensor = None
uart = None

try:
    # 先把 UART1 准备好，后面主循环里每帧都可能发识别结果。
    uart = init_uart()

    # 初始化摄像头。
    # 这里底层传感器分辨率先开到 1280x960，
    # 实际处理帧尺寸设置为 VGA，兼顾识别效果和运行速度。
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(Sensor.VGA)
    sensor.set_pixformat(Sensor.RGB565)

    # 初始化显示与媒体系统。
    # to_ide=True 方便在 IDE 中同步查看图像输出。
    Display.init(Display.ST7701, width=640, height=480, fps=90, to_ide=True)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # 记录上次发送时间和上一帧发送内容。
    # 用于实现“内容变化立即发送，内容不变定时重发”的机制。
    last_send_ms = 0
    last_line_send_ms = 0
    last_frame = None
    last_line_frame = None

    while True:
        # 系统退出检查点，便于脚本被外部正常终止。
        os.exitpoint()
        clock.tick()
        img = sensor.snapshot()

        now = time.ticks_ms()

        # 基础阶段以视觉巡线为主，输出 $L,error,angle,confidence,flags#。
        line = find_line(img)
        line_frame = (line["error"], line["angle"], line["confidence"], line["flags"])
        if (line_frame != last_line_frame) or (time.ticks_diff(now, last_line_send_ms) >= SEND_LINE_INTERVAL_MS):
            send_line(uart, line)
            last_line_send_ms = now
            last_line_frame = line_frame

        img.draw_string(
            10,
            10,
            "L e:{} a:{} c:{} f:{}".format(line["error"], line["angle"], line["confidence"], line["flags"]),
            color=(255, 255, 255),
            scale=2,
        )

        # 色块识别先保留为调试显示。等基础巡线稳定后，再打开串口上报并接入避障。
        target = find_best_target(img)
        if target is None:
            frame = (0, "N", 0)
        else:
            pos = classify_pos(target["cx"])
            frame = (target["color_id"], pos, target["pixels"])
            label = "{} {} {}".format(COLOR_NAMES[target["color_id"]], pos, target["pixels"])
            img.draw_string(10, 35, label, color=(255, 255, 255), scale=2)

        if ENABLE_COLOR_REPORT:
            if (frame != last_frame) or (time.ticks_diff(now, last_send_ms) >= SEND_INTERVAL_MS):
                send_result(uart, frame[0], frame[1], frame[2])
                last_send_ms = now
                last_frame = frame

        # 把调试图像输出到屏幕。
        Display.show_image(img)
        print("fps={}, line={}".format(clock.fps(), last_line_frame))

except KeyboardInterrupt as e:
    print("user stop:", e)
except BaseException as e:
    print("Exception {}".format(e))
finally:
    # 做资源释放，避免摄像头、显示和媒体资源没有正确关闭。
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
