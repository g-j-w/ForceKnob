# Force Knob 力控旋钮

![Platform](https://img.shields.io/badge/Platform-STM32H743-blue)
![Motor](https://img.shields.io/badge/Motor-2804%20BLDC-green)
![Encoder](https://img.shields.io/badge/Encoder-AS5600-orange)
![Driver](https://img.shields.io/badge/Driver-SimpleFOC%20Mini%20(MS8313)-red)
![Mode](https://img.shields.io/badge/Mode-FOC%20Voltage-purple)

一个基于 **STM32H743IIT6** 的力反馈旋钮（Force Feedback Knob）项目。采用**电压模式 FOC**（无电流采样），通过电机主动出力给旋钮提供三种可切换的"手感"：**段落感 / 阻尼 / 弹簧**。

A force feedback knob based on **STM32H743IIT6**, using voltage-mode FOC (no current sensing). The motor actively produces torque to give the knob three switchable haptic feels: **Detent / Damping / Spring**.

---

## ✨ 功能 Features

- ⚡ 电压模式 FOC（反 Park → 反 Clarke → 中心对齐 → 20kHz PWM）
- 🎛️ **三种手感模式**，按板上 K1 按键循环切换：
  1. **DETENT 段落感**
  2. **DAMPING 阻尼**
  3. **SPRING 弹簧**
- 📡 AS5600 磁编码器（I2C1），多圈角度跟踪 + 速度 EMA 滤波
- 📤 串口实时调试输出（USART1，115200-8-N-1）
- 🔔 上电自动对齐（编码器零点校准，上电"咔"一声属正常）
- 🛠️ 全参数集中可调，方便调手感

---

## 🎛️ 三种手感模式详解 Modes

> 默认进入 **DETENT** 模式。按板上 **K1（PC13）** 按键循环切换：`段落 → 阻尼 → 弹簧 → 段落…`
> 切到弹簧模式的那一刻，旋钮当前位置会被记为"回中零点"。

### 1️⃣ DETENT 段落感（默认）

- **现象**：旋钮被"锁"在一圈 **12 个固定档位**上。转动时一格一格地"咔哒"，每过一档都能明显感到一个阻力峰；松手会**自动停在最近的一档**。
- **原理**：控制环施加的力矩 = `-K_DETENT · sin(12·θ) - B_DAMPING · ω`。正弦势阱在 12 个位置形成稳定点，转子被吸向最近的稳定点。
- **像什么**：音量旋钮、洗衣机/示波器的多档旋钮。

### 2️⃣ DAMPING 阻尼

- **现象**：旋钮可以在**任意角度自由转动**，但转动时始终有**持续的粘滞阻力**——速度越快阻力越大，慢转则轻。松手会**停在任意位置**（不回弹、没有档位）。
- **原理**：力矩 = `-B_DAMPING · ω`，纯速度比例阻尼，像在油里转。
- **像什么**：在浓稠液体里搅拌，或汽车方向盘助力变重的手感。

### 3️⃣ SPRING 弹簧

- **现象**：进入该模式的位置记为"零点"。旋钮转到**任何位置松手都会自动弹回零点**，离零点越远拉力越大；慢慢松开会平滑回中。
- **原理**：力矩 = `-K_SPRING · (θ - θ₀) - B_DAMPING · ω`，线性弹簧 + 阻尼防振荡。
- **像什么**：相机云台自动回正、赛车游戏方向盘回中。

> 三种模式的力矩最后都经过 `TORQUE_GAIN` 放大、并被 `VQ_LIMIT` 限幅（安全保护）。

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
| 按键 | PC13 | 板上 User Key K1（切模式） |

> 注：V3 核心板排针未引出 PB6 / PF0 / PF1，故编码器用 PB8/PB9。

### 接线 Wiring

```
12V(+) → Mini VMOT        12V(-) → Mini GND
Mini GND ↔ 核心板 GND（必须共地）

PC6 → IN1    PC7 → IN2    PC8 → IN3    PB0 → EN
Mini M1/M2/M3 → 电机 U/V/W（方向反就交换任意两根相线）

PB8 → AS5600 SCL    PB9 → AS5600 SDA   （4.7kΩ 上拉到 3.3V）
PB14 → CH340 RXD    PB15 → CH340 TXD   （115200-8-N-1）

ST-Link: PA13(TMS)→SWDIO  PA14(TCK)→SWCLK  GND
```

---

## 🧠 工作原理 How It Works

```
1kHz 控制环（TIM2 中断，每 1ms）：
  读 AS5600 角度 → 算速度（EMA 滤波）
  → 按当前模式算目标力矩 → 力矩 × 增益 = Vq 电压
  → 反 Park / 反 Clarke / 中心对齐 → 三相占空比 → TIM8 CCR
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
#define NUM_DETENTS  12     /* 段落数：一圈几档 */
#define K_DETENT     0.8f   /* 段落硬度：越大越难拧过去 */
#define K_SPRING     0.5f   /* 弹簧刚度：越大回中越快/越硬 */
#define B_DAMPING    0.08f  /* 阻尼：越大越粘 */
#define TORQUE_GAIN  6.0f   /* 整体力度放大 */
#define VQ_LIMIT     3.5f   /* 限幅（安全上限） */
```

> ⚠️ `VQ_LIMIT` 才是真正的出力上限（2804 相阻 2.3Ω，3.5V ≈ 1.5A 峰值）。
> K 和 TORQUE_GAIN 再大也会被它卡住，改不动时先看这里。

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
