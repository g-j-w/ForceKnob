# Force Knob 力控旋钮

![Platform](https://img.shields.io/badge/Platform-STM32H743-blue)
![Motor](https://img.shields.io/badge/Motor-2804%20BLDC-green)
![Encoder](https://img.shields.io/badge/Encoder-AS5600-orange)
![Driver](https://img.shields.io/badge/Driver-SimpleFOC%20Mini%20(MS8313)-red)
![Mode](https://img.shields.io/badge/Mode-FOC%20Voltage-purple)

一个基于 **STM32H743IIT6** 的力反馈旋钮（Force Feedback Knob）项目。采用电压模式 FOC（无电流采样），支持三种手感：**段落感 / 阻尼 / 弹簧**，通过按键循环切换。

A force feedback knob based on **STM32H743IIT6**. It uses voltage-mode FOC (no current sensing) with three haptic modes: **Detent / Damping / Spring**, switchable by a button.

---

## ✨ 功能 Features

- 🔁 电压模式 FOC（反 Park → 反 Clarke → 中心对齐 → PWM）
- 🎛️ 三种手感模式：
  - **DETENT 段落感**：一圈 12 段咔哒（正弦势阱）
  - **DAMPING 阻尼**：像在油里转（速度比例阻尼）
  - **SPRING 弹簧**：转到哪松手弹回零点
- 📡 AS5600 磁编码器（I2C1），多圈角度 + 速度 EMA 滤波
- 🖥️ SSD1306 OLED 实时显示（I2C2，独立总线不干扰编码器）
- 📤 串口调试输出（USART1，115200）
- 🔔 上电自动对齐（编码器零点校准）
- ⌨️ 板上 K1 按键切换模式

---

## 🔧 硬件清单 Hardware

| 元件 | 型号 |
|------|------|
| 主控 | 极客/WeAct STM32H743IIT6 核心板 V3（无 HSE，用 HSI） |
| 驱动板 | SimpleFOC Mini（主芯片 MS8313，兼容 DRV8313） |
| 电机 | 2804 直流无刷电机（7 对极，内置 AS5600，DFRobot FIT1034） |
| 显示屏 | 0.96" SSD1306 OLED（I2C，地址 0x3C） |
| 调试 | CH340 USB-TTL（串口）+ ST-Link V2（烧录） |
| 电源 | 12V/2A+ 电源（驱动板 VMOT） |

---

## 🔌 引脚分配 Pin Mapping

| 功能 | 引脚 | 说明 |
|------|------|------|
| PWM 三相 | PC6 / PC7 / PC8 | TIM8_CH1/CH2/CH3 → Mini IN1/IN2/IN3 |
| 使能 EN | PB0 | 拉高才使能驱动 |
| 编码器 I2C1 | PB8(SCL) / PB9(SDA) | → AS5600（各加 4.7kΩ 上拉） |
| OLED I2C2 | PB10(SCL) / PB11(SDA) | → SSD1306（各加 4.7kΩ 上拉） |
| 串口 USART1 | PB14(TX) / PB15(RX) | → CH340 |
| 按键 | PC13 | 板上 User Key K1 |

> 注：V3 核心板排针未引出 PB6 / PF0 / PF1，故编码器用 PB8/PB9、OLED 用 PB10/PB11。

### 接线 Wiring

```
12V(+) → Mini VMOT        12V(-) → Mini GND
Mini GND ↔ 核心板 GND（共地）

PC6 → IN1    PC7 → IN2    PC8 → IN3    PB0 → EN
Mini M1/M2/M3 → 电机 U/V/W（方向反就交换任意两根）

PB8 → AS5600 SCL    PB9 → AS5600 SDA   （4.7kΩ 上拉到 3.3V）
PB10 → OLED SCL     PB11 → OLED SDA    （4.7kΩ 上拉到 3.3V）
PB14 → CH340 RXD    PB15 → CH340 TXD   （115200-8-N-1）

ST-Link: PA13(TMS)→SWDIO  PA14(TCK)→SWCLK  GND
```

---

## 🛠️ 编译与烧录 Build & Flash

- 工程由 STM32CubeMX 生成，CMake 构建（VS Code / STM32CubeCLT）
- 编译：`cmake --preset Debug && cmake --build build/Debug`
- 烧录：工程根目录 `flash.bat`（调用 STM32CubeProgrammer CLI，**SWD 频率 800kHz**）
  - ⚠️ 杜邦线接 ST-Link 时 4MHz 会导致 `failed to erase memory`，必须用 800kHz

---

## 🎛️ 手感调参 Tuning

参数集中在 `Core/Src/force_feedback.c` 顶部：

```c
#define NUM_DETENTS  12     /* 段落数 */
#define K_DETENT     0.8f   /* 段落刚度 */
#define K_SPRING     0.5f   /* 弹簧刚度 */
#define B_DAMPING    0.08f  /* 阻尼系数 */
#define TORQUE_GAIN  6.0f   /* 整体力度放大 */
#define VQ_LIMIT     3.5f   /* 限幅（安全上限，别超） */
```

> ⚠️ `VQ_LIMIT` 决定实际出力上限（2804 相阻 2.3Ω，3.5V ≈ 1.5A 峰值），改大 K 之前先看它有没有卡住。

---

## 📂 工程结构 Structure

```
Core/
  Inc/  foc_math.h  encoder.h  force_feedback.h  button.h  ssd1306.h ...
  Src/  main.c  encoder.c  force_feedback.c  button.c  ssd1306.c ...
likong.ioc          # CubeMX 工程配置
CMakeLists.txt      # CMake 构建
flash.bat           # 一键烧录脚本
```

---

## ⚠️ 已知注意点 Notes

- 本板 **HSE 晶振用不了**，时钟只用内部 HSI 64MHz → PLL → 240MHz
- 电机上电"咔"一声对齐是正常现象
- 长时间满力矩顶着会发热（DRV8313/MS8313 有热保护）

## 📄 License

MIT
