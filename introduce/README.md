# STM32 平衡小车（520 电机）

基于 `STM32F103C8` 的两轮自平衡小车工程，使用 STM32 标准外设库开发，Keil 工程文件位于 `USER/UpStanding_Car.uvprojx`。

当前工程已集成：

- MPU6050 姿态采集与 DMP 解算
- 双编码器测速
- 双电机 PWM 驱动与方向控制
- 三环控制：直立环 + 速度环 + 转向环
- 蓝牙串口遥控
- 八路巡线模块串口接入
- OLED 状态显示

## 目录结构

```text
CORE/                 Cortex-M3 内核与启动文件
HARDWARE/             外设与功能模块
  CONTROL/            平衡控制算法
  ENCODER/            编码器测速
  EXTI/               MPU6050 外部中断
  MOTOR/              电机方向控制
  MPU6050/            MPU6050 驱动与 DMP
  OLED/               OLED 显示
  PWM/                PWM 输出
  SENSOR/             超声波源码目录
  TRACKING/           八路巡线处理
  USART3/             蓝牙串口
SYSTEM/               系统基础模块
STM32F10x_FWLib/      STM32 标准外设库
USER/                 应用层与 Keil 工程文件
OBJ/                  编译输出目录
```

## 当前功能说明

- 主循环主要负责 OLED 显示。
- 实时闭环控制运行在 `EXTI9_5_IRQHandler()` 中断中。
- `PB5` 接 MPU6050 中断输出，用于触发姿态采样和控制更新。
- 巡线模块当前通过 `USART2` 接入：`PA2` 为 TX，`PA3` 为 RX。

说明：

- 目录中保留了超声波代码，但当前主流程已关闭超声波功能。
- 由于 `PA2/PA3` 已分配给 `USART2` 巡线模块，超声波与巡线功能不能按当前接线同时使用。

## 主要 IO

| 模块 | 引脚 | 作用 |
|---|---|---|
| 左编码器 TIM2 | `PA0`, `PA1` | 编码器输入 |
| 右编码器 TIM4 | `PB6`, `PB7` | 编码器输入 |
| MPU6050 软件 IIC | `PB4` SCL, `PB3` SDA | 姿态传感器通信 |
| MPU6050 中断 | `PB5` | 数据就绪中断 |
| OLED 软件 IIC | `PB8` SCL, `PB9` SDA | OLED 显示 |
| 调试串口 USART1 | `PA9` TX, `PA10` RX | 调试输出 |
| 蓝牙串口 USART3 | `PB10` TX, `PB11` RX | 蓝牙遥控 |
| 巡线串口 USART2 | `PA2` TX, `PA3` RX | 八路巡线模块 |
| 左电机 PWM | `PA8` | `TIM1_CH1` |
| 右电机 PWM | `PA11` | `TIM1_CH4` |
| 电机方向控制 | `PB12` ~ `PB15` | H 桥方向控制 |

## 关键参数

| 参数 | 数值 |
|---|---:|
| 主控芯片 | `STM32F103C8` |
| PWM 周期 | `ARR = 7199` |
| PWM 频率 | 约 `10kHz` |
| 直立环 Kp | `-400` |
| 直立环 Kd | `-1.92` |
| 速度环 Kp | `-0.44` |
| 速度环 Ki | `-0.0022` |
| 转向环 Kp | `20` |
| 转向环 Kd | `0.6` |
| 蓝牙波特率 | `9600` |
| 调试串口波特率 | `115200` |

## 编译与下载

1. 用 Keil 打开 `USER/UpStanding_Car.uvprojx`
2. 确认目标芯片与下载器配置正确
3. 编译工程
4. 下载到开发板

## 备注

- `README.md` 以当前代码实际行为为准。
- `HARDWARE/SENSOR/` 目录存在，不代表当前主流程正在使用超声波。
- `OBJ/` 目录为编译产物，不属于源码核心部分。
