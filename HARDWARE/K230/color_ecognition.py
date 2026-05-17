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
LEFT_BOUND = FRAME_WIDTH // 3
RIGHT_BOUND = FRAME_WIDTH * 2 // 3

# 色块筛选下限。
# 太小的噪点不参与识别，减少误检。
MIN_PIXELS = 200
MIN_AREA = 200

# 串口最大发送间隔。
# 即便结果没有变化，也会每 100ms 重发一次，方便下位机持续接收最新状态。
SEND_INTERVAL_MS = 100

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
    last_frame = None

    while True:
        # 系统退出检查点，便于脚本被外部正常终止。
        os.exitpoint()
        clock.tick()
        img = sensor.snapshot()

        # 在当前图像里寻找最佳颜色目标。
        target = find_best_target(img)
        now = time.ticks_ms()

        if target is None:
            # 未识别到颜色块时，发送空状态。
            # 这里位置字段固定为 N，面积为 0。
            frame = (0, "N", 0)
            img.draw_string(10, 10, "NONE", color=(255, 255, 255), scale=2)
        else:
            # 识别到目标后，计算它位于左/中/右哪个区域，
            # 并组织成准备发送给下位机的数据。
            pos = classify_pos(target["cx"])
            frame = (target["color_id"], pos, target["pixels"])
            label = "{} {} {}".format(COLOR_NAMES[target["color_id"]], pos, target["pixels"])
            img.draw_string(10, 10, label, color=(255, 255, 255), scale=2)

        # 两种情况会发送串口：
        # 1. 当前结果和上一次不同
        # 2. 虽然结果相同，但已经超过最大发送间隔
        # 这样既保证状态切换时响应快，也保证下位机不会长时间收不到数据。
        if (frame != last_frame) or (time.ticks_diff(now, last_send_ms) >= SEND_INTERVAL_MS):
            send_result(uart, frame[0], frame[1], frame[2])
            last_send_ms = now
            last_frame = frame

        # 把调试图像输出到屏幕。
        Display.show_image(img)
        print("fps={}, tx={}".format(clock.fps(), last_frame))

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
