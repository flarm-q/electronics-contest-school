import time
import sensor
import lcd
from machine import UART
from fpioa_manager import fm


# K210 与 STM32 的 UART1 接线说明。
#
# 你给的引脚图中，K210 板子的 UART1 引脚为:
#   IO7 -> U1_TX
#   IO9 -> U1_RX
#
# 实际接线时要交叉连接:
#   K210 IO7 / U1_TX  ->  STM32 视觉串口 RX，当前工程里是 USART3_RX(PB11)
#   K210 IO9 / U1_RX  ->  STM32 视觉串口 TX，当前只接收图像结果时可以不接
#   K210 GND          ->  STM32 GND，必须共地，否则串口电平没有共同参考点
#
# 波特率必须和 STM32 侧 main.c 里的 uart3_init(115200) 保持一致。
UART_BAUDRATE = 115200
UART1_TX_PIN = 7
UART1_RX_PIN = 9

# 图像尺寸设置。
#
# K210 的算力明显弱于 K230，因此这里使用 QVGA(320x240) 处理图像，优先保证帧率。
# 原 K230 脚本使用 VGA(640x480)，STM32 侧控制参数也是按 VGA 的 error 尺度调的。
# 为了尽量不重新大幅调整 STM32 控制参数，K210 端计算出的 error/angle 会乘以 2，
# 让输出数值接近原来的 VGA 坐标尺度。
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

# 白底黑线巡线阈值。
#
# find_blobs() 使用 LAB 阈值找目标。这里把 A/B 色彩范围放到最大，
# 主要依靠 L 亮度筛选黑线，也就是“亮度低的区域认为是黑色赛道线”。
#
# 调参建议:
#   1. 黑线断裂、识别不到: 适当调大 L_max，例如从 45 调到 55。
#   2. 阴影也被当成黑线: 适当调小 L_max，例如从 45 调到 35。
#   3. 现场光照变化大: 优先改善补光和摄像头角度，再细调阈值。
LINE_THRESHOLD = (0, 45, -128, 127, -128, 127)

# 巡线 ROI 设置。
#
# ROI 格式:
#   (x, y, w, h, weight)
#
# 各字段含义:
#   x/y/w/h : 在图像中的矩形区域
#   weight  : 该区域对最终 error 的权重
#
# 这里使用三条横向 ROI:
#   1. 底部 ROI 接近车头，权重最大，主要决定当前是否压线。
#   2. 中部 ROI 用来观察前方路线走势。
#   3. 上部 ROI 更远，用来提前感知弯道趋势。
#
# 如果车反应太慢，可以适当提高中上部 ROI 权重。
# 如果车在弯道里左右摆动明显，可以降低中上部 ROI 权重或降低 STM32 侧转向增益。
LINE_ROIS = [
    (0, 165, FRAME_WIDTH, 35, 50),
    (0, 125, FRAME_WIDTH, 33, 30),
    (0, 85, FRAME_WIDTH, 35, 20),
]

# 巡线 blob 过滤参数。
#
# LINE_MIN_PIXELS / LINE_MIN_AREA:
#   过滤面积很小的噪声点，避免黑色螺丝、阴影边缘、图像噪点被误当成赛道线。
#
# LINE_MAX_JUMP:
#   限制相邻 ROI 或相邻帧之间的横向跳变。圆环、短横线、阴影进入画面时，
#   可能出现多个黑色候选区域，这个参数会优先选择位置连续的主线。
#
# LINE_CURVE_THRESHOLD:
#   angle 超过该值时置 curve 标志。STM32 当前主要用 error/angle 控制，
#   curve 标志后续可以用于弯道降速或调试显示。
LINE_MIN_PIXELS = 20
LINE_MIN_AREA = 20
LINE_MAX_JUMP = 80
LINE_CURVE_THRESHOLD = 35

# 巡线状态标志位。
#
# LINE_FLAG_VALID:
#   当前帧找到了有效黑线，可以给 STM32 用于巡线控制。
#
# LINE_FLAG_LOST:
#   当前帧没有找到黑线，STM32 侧会拒绝接管，避免沿用错误方向。
#
# LINE_FLAG_CURVE:
#   当前角度趋势较大，说明可能进入弯道。
LINE_FLAG_VALID = 0x01
LINE_FLAG_LOST = 0x02
LINE_FLAG_CURVE = 0x04

last_line_error = 0


def ticks_ms():
    # 获取毫秒时间戳。
    #
    # 大多数 MaixPy 固件都有 time.ticks_ms()，但不同版本兼容性不完全一致。
    # 这里做一层封装，如果没有 ticks_ms()，就用 time.time() 退化计算。
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, old):
    # 计算两个时间戳之间的差值。
    #
    # 使用这个函数而不是直接 now - old，是为了兼容 ticks_ms() 计数回绕的情况。
    # 如果固件没有 time.ticks_diff()，才退化为普通减法。
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, old)
    return now - old


def init_uart1():
    # 配置 K210 FPIOA 引脚复用。
    #
    # K210 的物理 IO 并不是固定绑定某个外设功能，需要先通过 fm.register()
    # 把 IO7/IO9 映射到 UART1_TX/UART1_RX，UART1 才能从对应针脚输出数据。
    fm.register(UART1_TX_PIN, fm.fpioa.UART1_TX, force=True)
    fm.register(UART1_RX_PIN, fm.fpioa.UART1_RX, force=True)

    # 不同 MaixPy 固件版本对 UART() 构造参数支持略有差异。
    # 先尝试带 timeout/read_buf_len 的完整写法，失败后回退到更通用的简化写法。
    # 这里没有做接收处理，read_buf_len 主要是为了后续扩展调试命令时预留。
    try:
        return UART(UART.UART1, UART_BAUDRATE, 8, None, 1, timeout=1000, read_buf_len=256)
    except TypeError:
        return UART(UART.UART1, UART_BAUDRATE, 8, 0, 1)


def write_uart(uart, frame):
    # 串口发送统一封装。
    #
    # 有些固件版本 uart.write() 可以直接写字符串，有些只接受 bytes。
    # 这里先按字符串发送，遇到 TypeError 再编码成 bytes，避免脚本因为固件差异退出。
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
    # 把数值限制在指定范围内。
    # 当前主要用于把 confidence 限制在 0~100，避免异常像素面积导致置信度越界。
    if value < low:
        return low
    if value > high:
        return high
    return value


def blob_pixels(blob):
    # 读取 blob 的像素面积。
    #
    # MaixPy/OpenMV 风格的 blob 有时是对象，有时表现得像元组。
    # 为了兼容不同固件，这里优先调用 pixels()，没有该方法就按元组下标读取。
    if hasattr(blob, "pixels"):
        return blob.pixels()
    return blob[4]


def blob_rect(blob):
    # 读取 blob 外接矩形，返回 (x, y, w, h)。
    # 这个封装和 blob_pixels() 一样，是为了兼容对象式和元组式两种 blob 表示。
    if hasattr(blob, "rect"):
        return blob.rect()
    return blob[0], blob[1], blob[2], blob[3]


def blob_center(blob):
    # 读取 blob 中心点坐标。
    #
    # 如果固件提供 cx()/cy() 就直接使用；否则根据外接矩形手动计算中心点。
    # 巡线 error 和色块 L/C/R 判断都依赖这个中心点。
    if hasattr(blob, "cx"):
        return blob.cx(), blob.cy()
    x, y, w, h = blob_rect(blob)
    return x + w // 2, y + h // 2


def draw_rect(img, rect, color):
    # 在 LCD/IDE 调试画面上画矩形框。
    #
    # K210 不同固件对 draw_rectangle() 的 thickness 参数支持不完全一致，
    # 所以这里做 TypeError 兼容，避免仅仅因为调试绘图失败导致主程序退出。
    try:
        img.draw_rectangle(rect, color=color, thickness=2)
    except TypeError:
        img.draw_rectangle(rect, color=color)


def draw_cross(img, x, y, color):
    # 在目标中心画十字，方便肉眼判断当前选中的候选区域是否正确。
    # 和 draw_rect() 一样，这里兼容不支持 size/thickness 参数的固件。
    try:
        img.draw_cross(x, y, color=color, size=8, thickness=2)
    except TypeError:
        img.draw_cross(x, y, color=color)


def draw_text(img, x, y, text, color=(255, 255, 255), scale=1):
    # 在图像上写调试文字。
    #
    # LCD 左上角会显示巡线 error、angle、confidence、flags。
    # 如果色块避障开启并识别到目标，下一行会显示颜色、位置和面积。
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
    # 在单个 ROI 内寻找最可信的黑线候选区域。
    #
    # 不能简单取“面积最大的黑色区域”，原因是赛道中可能同时出现主线、圆环内边、
    # 短横线、阴影等多个黑色 blob。这里把面积、位置连续性、形状合理性一起评分。
    #
    # expected_error 表示“预计主线大概在哪里”:
    #   1. 第一个 ROI 使用上一帧的 error，保持帧间连续。
    #   2. 后续 ROI 使用前一个 ROI 找到的位置，保持上下 ROI 之间连续。
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

        # 连续性惩罚:
        # 候选区域离 expected_error 越远，越可能不是主线，因此扣分越多。
        continuity_penalty = abs(error - expected_error) * 4
        shape_penalty = 0
        if bw < 3 or bh < 3:
            shape_penalty += 300
        if abs(error - expected_error) > LINE_MAX_JUMP:
            shape_penalty += 500

        # 最终评分:
        # 面积越大越可信，但如果位置跳变太大或形状太小，就降低优先级。
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
    # 计算整帧图像的巡线结果。
    #
    # 输出内容:
    #   error      : 加权后的黑线横向偏差，左负右正
    #   angle      : 远处偏差 - 近处偏差，用来估计前方路线趋势
    #   confidence : 置信度，STM32 低于阈值时不会使用该帧巡线结果
    #   flags      : valid/lost/curve 状态位
    #
    # 算法流程:
    #   1. 依次扫描底部、中部、上部三个 ROI。
    #   2. 每个 ROI 选择一个最可信黑线候选。
    #   3. 按 ROI 权重加权求出最终 error。
    #   4. 用远近 ROI 的 error 差计算 angle。
    #   5. 根据命中 ROI 数量和黑线像素数量估算 confidence。
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
        # 完全找不到黑线时，不更新 last_line_error。
        # 返回上一帧 error 只是为了 LCD 显示和短时间观察，flags 会置 lost，
        # STM32 收到 lost 后不会把这帧作为有效巡线控制输入。
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

    # 置信度计算:
    # 命中的 ROI 越多，说明主线越连续；像素越多，说明黑线越明显。
    # 这里不是严格概率，只是给 STM32 一个“当前视觉结果是否可靠”的量化参考。
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
    # 初始化摄像头。
    #
    # set_pixformat(sensor.RGB565):
    #   使用彩色图像，既能做黑线巡线，也能识别红/绿/蓝色块。
    #
    # set_framesize(sensor.QVGA):
    #   使用 320x240，降低 K210 处理压力。
    #
    # set_hmirror/set_vflip:
    #   在图像进入识别算法之前完成方向校正，保证 error 正负和 L/C/R 判断正确。
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(CAMERA_HMIRROR)
    sensor.set_vflip(CAMERA_VFLIP)
    sensor.skip_frames(time=2000)
    try:
        # 关闭自动增益和自动白平衡后，颜色阈值会更稳定。
        # 如果现场亮度变化特别大，可以临时打开自动功能观察效果，
        # 但色块阈值可能会随画面变化而漂移。
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
    except Exception:
        pass


def main():
    # 主流程:
    #   1. 初始化 UART1，用于向 STM32 上报视觉结果。
    #   2. 初始化 LCD，方便现场观察识别框和调试信息。
    #   3. 初始化摄像头。
    #   4. 循环读取图像，先做高频巡线，再按较低频率做色块避障。
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

        # 高频巡线处理。
        #
        # 巡线是平衡车沿赛道走的基础输入，因此刷新频率要高。
        # 当前策略是:
        #   1. 如果巡线结果变化，立即发送。
        #   2. 即使结果不变，也至少每 SEND_LINE_INTERVAL_MS 重发一次。
        # 这样 STM32 侧如果长时间收不到新帧，就能通过超时机制退出视觉控制。
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
                # 没有找到色块时主动发送空目标帧。
                # 这比“什么都不发”更可靠，因为 STM32 能立刻知道障碍物已经消失。
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
            # 串口/IDE 打印也会占用时间，尤其在 K210 上影响比较明显。
            # 因此调试信息按 500ms 左右低频打印，避免影响图像处理帧率。
            print("fps={}, line={}".format(clock.fps(), last_line_frame))
            last_debug_print_ms = now


try:
    main()
except Exception as e:
    print("Exception:", e)
