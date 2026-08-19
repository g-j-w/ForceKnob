# Force Knob 力控旋钮

![Platform](https://img.shields.io/badge/Platform-STM32H743-blue)
![Motor](https://img.shields.io/badge/Motor-2804%20BLDC-green)
![Encoder](https://img.shields.io/badge/Encoder-AS5600-orange)
![Driver](https://img.shields.io/badge/Driver-SimpleFOC%20Mini%20(MS8313)-red)
![Mode](https://img.shields.io/badge/Mode-FOC%20Voltage-purple)

一个基于 **STM32H743IIT6** 的力反馈旋钮（Force Feedback Knob）项目。采用**电压模式 FOC**（无电流采样），三档模式：**自转 / 步进 / 边界挡块**，外接按键循环切换。

A force feedback knob based on **STM32H743IIT6**, using voltage-mode FOC (no current sensing). Three switchable modes: **Spin / Detent / Endstop**, cycled by an external button.

---




## ✨ 功能 Features

- ⚡ 电压模式 FOC（反 Park → 反 Clarke → 中心对齐 → 20kHz PWM）
- 🎛️ **三档模式**，按外接按键（PA0）循环切换：
  1. **SPIN 自转**（开路旋转）
  2. **DETENT 步进**（一圈 12 格咔哒）
  3. **ENDSTOP 边界**（转到头硬停）
- 📡 AS5600 磁编码器（I2C1），多圈角度跟踪 + 速度 EMA 滤波
- 📤 串口实时调试输出（USART1，115200-8-N-1）
- 🔔 上电自动对齐（编码器零点校准，上电"咔"一声属正常）
- 🛠️ 全参数集中可调，方便调手感

---

## 🎛️ 三档模式详解 Modes

> 默认进入 **SPIN 自转** 模式。按外接按键（**PA0**）循环切换：`自转 → 步进 → 边界 → 自转…`
> 切换顺序固定为 0→1→2→0，串口 `M:` 值实时显示当前档位。

### 1️⃣ SPIN 自转（默认）

- **现象**：电机**自动连续旋转**（约 0.5 圈/秒），不依赖旋钮位置。
- **原理**：开路旋转——让磁场匀速旋转（`spin_theta` 每 1ms 增加固定角度），电机跟随磁场转动。
- **用途**：演示驱动链路、确认电机/驱动/电源正常。

### 2️⃣ DETENT 步进（段落感）

- **现象**：旋钮被"锁"在一圈 **12 个固定档位**上。转动时一格一格地"咔哒"，每过一档都能明显感到一个阻力峰；松手会**自动停在最近的一档**。
- **原理**：借鉴 smartKnob——**比例弹簧 + 死区**：距最近档位的角度线性出力，档位中心 ±2° 死区内不出力（防振荡），靠近中心时阻尼加强（压残余振动）。
- **像什么**：音量旋钮、洗衣机/示波器的多档旋钮。

### 3️⃣ ENDSTOP 边界挡块

- **现象**：旋钮在中间**自由转动**，转到左右 **±180°** 极限被**硬挡**（推回），像限位旋钮。
- **原理**：只有超过边界时才输出强推力 `K_END · (超出角度)`，中间只留轻阻尼。结构最简单、最不易受电机齿槽/振荡影响。
- **像什么**：限位旋钮、档位开关的末端挡块。

> 各模式力矩都被 `VQ_LIMIT` 限幅（安全保护）。
>
> 硬件连接及视频演示
> 


https://github.com/user-attachments/assets/ad11e7ab-9a35-4232-80de-d53f98c73c72


> <img width="3072" height="4096" alt="2019ae9b1daad7814846be8cea0f04ba" src="https://github.com/user-attachments/assets/b623f335-d35c-4f88-8efe-16834d324b76" />


---

## 🔧 硬件清单 Hardware

| 元件 | 型号 |
|------|------|
| 主控 | 极客/WeAct STM32H743IIT6 核心板 V3（无 HSE 晶振，用 HSI） |
| 驱动板 | SimpleFOC Mini（主芯片瑞盟 MS8313，兼容 TI DRV8313） |
| 电机 | 2804 直流无刷电机（7 对极，DFRobot FIT1034，内置 AS5600） |
| 编码器 | AS5600 磁编码器（12 位，I2C 地址 0x36） |
| 调试 | CH340 USB-TTL（串口）+ ST-Link V2（烧录） |
| 电源 | 12V/2A+ 电源（给驱动板 VMOT） |

---

## 🔌 引脚分配 Pin Mapping

| 功能 | 引脚 | 说明 |
|------|------|------|
| PWM 三相 | PC6 / PC7 / PC8 | TIM8_CH1/CH2/CH3 → Mini IN1/IN2/IN3 |
| 使能 EN | PB0 | 拉高才使能驱动 |
| 编码器 I2C1 | PB8(SCL) / PB9(SDA) | → AS5600（各加 4.7kΩ 上拉） |
| 串口 USART1 | PB14(TX) / PB15(RX) | → CH340 |
| 按键(外接) | PA0 | 面包板按键，按下接 GND（切模式） |

> 注：V3 核心板排针未引出 PB6 / PF0 / PF1，故编码器用 PB8/PB9。

### 接线 Wiring

```
12V(+) → Mini VMOT        12V(-) → Mini GND
Mini GND ↔ 核心板 GND（必须共地）

PC6 → IN1    PC7 → IN2    PC8 → IN3    PB0 → EN
Mini M1/M2/M3 → 电机 U/V/W（方向反就交换任意两根相线）

PB8 → AS5600 SCL    PB9 → AS5600 SDA   （4.7kΩ 上拉到 3.3V）
PB14 → CH340 RXD    PB15 → CH340 TXD   （115200-8-N-1）
PA0 ── 外接按键一脚，按键另一脚 ── GND（切模式）

ST-Link: PA13(TMS)→SWDIO  PA14(TCK)→SWCLK  GND
```

---

## 🧠 工作原理 How It Works

```
1kHz 控制环（TIM2 中断，每 1ms）：
  读 AS5600 角度 → 算速度（EMA 滤波，α=0.1）
  → 按模式算 Vq 电压
  → 反 Park / 反 Clarke / 中心对齐 → 三相占空比 → TIM8 CCR
  模式 0（SPIN 自转）不读编码器，直接用旋转的电气角开路旋转
```

- 无电流采样，是 **SimpleFOC 的 voltage torque mode**，对力反馈旋钮够用
- 上电先**对齐**：A 相加 1.5V 保持 800ms，把转子吸到电气零度，再清零编码器
- 电气角 = 机械角 × 极对数（7）

---

## 🛠️ 编译与烧录 Build & Flash

- CubeMX 生成工程，CMake 构建（VS Code + STM32CubeCLT）
- 编译：`cmake --preset Debug && cmake --build build/Debug`
- 烧录：双击工程根目录 **`flash.bat`**（STM32CubeProgrammer CLI，**SWD 800kHz**）
  - ⚠️ 杜邦线接 ST-Link 时默认 4MHz 会报 `failed to erase memory`，必须 800kHz

---

## 🎛️ 手感调参 Tuning

所有参数集中在 `Core/Src/force_feedback.c` 顶部，**一次只调一个**：

```c
#define NUM_DETENTS      12       /* 段落数：一圈几档 */
#define K_DETENT_P       14.0f    /* 段落比例刚度 [V/rad]：越大越难拧 */
#define DEAD_ZONE        (2°)     /* 档位中心死区：防振荡的关键，别改成 0 */
#define B_DETENT_CENTER  0.25f    /* 档位中心额外阻尼：压残余振动 */
#define END_MAX_ANGLE    (_PI)    /* 边界挡块 ±180° */
#define K_END            12.0f    /* 边界推力 [V/rad] */
#define B_END            0.10f    /* 边界阻尼 */
#define VQ_LIMIT         4.5f     /* 限幅（安全上限，满力会发热） */
```

> ⚠️ `VQ_LIMIT` 是出力上限（2804 相阻 2.3Ω，4.5V ≈ 2A 峰值，长时间满力顶着会发热）。
> 段落实现借鉴 smartKnob：比例弹簧 + 死区（而非 sin 力），稳定性靠死区 + 比例项；速度滤波 α=0.1（encoder.c）。

---

## 📂 工程结构 Structure

```
Core/
  Inc/  foc_math.h  encoder.h  force_feedback.h  button.h  main.h ...
  Src/  main.c  encoder.c  force_feedback.c  button.c  i2c.c  tim.c ...
likong.ioc          # CubeMX 工程配置
CMakeLists.txt      # CMake 构建
flash.bat           # 一键烧录脚本
```

---

## ⚠️ 已知注意点 Notes

- 本板 **HSE 晶振用不了**，时钟只用内部 HSI 64MHz → PLL → 240MHz
- 电机上电"咔"一声对齐是正常现象
- 长时间满力矩顶着会发热（驱动板有热保护）
- 编码器 I2C1 总线只挂编码器，别挂其他 I2C 器件

## 📄 License

MIT
