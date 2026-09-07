# STM32H743IIT6 嵌入式项目合集

<p align="center">
  <a href="#"><img src="https://img.shields.io/badge/芯片-STM32H743IIT6-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white" alt="STM32H743IIT6"></a>
  <a href="#"><img src="https://img.shields.io/badge/内核-Cortex--M7_480MHz-orange?style=for-the-badge&logo=arm&logoColor=white" alt="Cortex-M7"></a>
  <a href="#"><img src="https://img.shields.io/badge/IDE-Keil_MDK--ARM-green?style=for-the-badge&logo=arm&logoColor=white" alt="Keil MDK"></a>
  <a href="#"><img src="https://img.shields.io/badge/库-ST官方HAL库-red?style=for-the-badge" alt="HAL"></a>
  <a href="#"><img src="https://img.shields.io/badge/语言-C99-9cf?style=for-the-badge&logo=c&logoColor=white" alt="C"></a>
  <a href="#"><img src="https://img.shields.io/badge/方向-电子设计竞赛-blueviolet?style=for-the-badge" alt="电赛"></a>
</p>

<p align="center">
  <b>面向「全国大学生电子设计竞赛」的 STM32H7 高性能嵌入式信号采集与处理项目合集</b>
</p>

---

## 📖 项目简介

本仓库汇集了基于 **STM32H743IIT6** 高性能微控制器的一系列嵌入式工程，覆盖 **信号采集、信号发生、数字信号处理（DSP）、闭环控制、频率/相位测量、仪器仪表** 等方向，是准备电子设计竞赛（电赛）过程中沉淀的完整代码库。

芯片基于 **ARM Cortex-M7** 内核，主频 **480MHz**，内置双精度浮点单元（FPU）与 DSP 指令集，非常适合高速 ADC 采样、FFT 频谱分析、数字滤波等实时信号处理场景。

> 💡 本仓库包含 **40 余个**独立的 Keil MDK 工程，均可直接编译、下载、验证。

---

## ✨ 核心技术亮点

| 方向 | 关键技术 |
|------|----------|
| **高速采集** | AD7606（8 通道 16bit 200Ksps）、AD9226（12bit 65Msps）、DMA + 定时器触发、双 ADC 交替采样、过采样 |
| **信号发生** | AD9833 / AD9910（DDS 芯片）、DAC904 高速 DAC、软件 DDS、DAC 任意波形输出、钢琴音色合成 |
| **数字信号处理** | 基 4 / 32768 点 FFT、加窗（Hann）、FIR / IIR 滤波、FFT 扫频、最小二乘法拟合、谐波分析 |
| **闭环控制** | PID 锁相环、方波相位差 PID 调节 |
| **频率/相位测量** | 定时器输入捕获测频、输入捕获 + DMA 测频、FFT 鉴相、李萨如（Lissajous）图形相位差测量 |
| **仪器仪表** | LCR 阻抗测量、THD 总谐波失真测量、激光测距 |
| **显示与音频** | RGB-LCD（LTDC）波形显示、UART 串口通信、音乐频谱分析 |

---

## 🛠 硬件平台

| 项目 | 说明 |
|------|------|
| **主控芯片** | STM32H743IIT6（Cortex-M7 @ 480MHz，2MB Flash，1MB RAM，双精度 FPU + DSP） |
| **开发板** | 慧勤智远 STM32H743IIT6 小系统板 V1.3 |
| **外设资源** | LED（PB0/PB1）、独立按键（PA0/PA1）、板载 CH340 USB 转串口、RGB-LCD 接口（LTDC）、SDRAM |

### 常用引脚分配

| 资源 | 引脚 |
|------|------|
| LED0 / LED1 | PB0 / PB1 |
| KEY0 / WK_UP | PA1 / PA0 |
| USART1（CH340） | PA9 / PA10 |
| RGB-LCD | LTDC 接口 |

---

## 🧰 开发环境

- **IDE**：Keil MDK-ARM 5（uVision 5）
- **固件库**：STM32 HAL 库 + CMSIS-DSP 库
- **时钟**：HSE 经过 PLL 配置到 **480MHz**（`sys_stm32_clock_init(192, 5, 2, 4)`）
- **初始化配置**：STM32CubeMX
- **下载调试**：ST-Link / J-Link
- **串口助手**：波特率 115200bps（需勾选 DTR）

---

## 📁 项目结构

```
H7IIT6
├── 📡 信号采集
│   ├── AD7606/                    # 8 通道 16bit ADC 采集
│   ├── AD7606 200K多通道/         # AD7606 200Ksps 多通道采集
│   ├── AD7606 200K成功版/         # AD7606 稳定版（200Ksps）
│   ├── AD9226/                    # 12bit 65Msps 高速 ADC
│   ├── AD9226定时器采集/          # 定时器触发高速采集
│   ├── AD9226采集AD904输出/       # 采集信号源输出
│   ├── ADC过采集/                 # ADC 过采样提高分辨率
│   ├── 三路adc/                   # 三路 ADC 采集
│   └── 双adc交替/                 # 双 ADC 交替采样（提高采样率）
│
├── 🔊, 信号发生
│   ├── AD9833/                    # DDS 可编程波形发生器
│   ├── AD9910/                    # 1GSPS 高性能 DDS
│   ├── DAC904/                    # 高速并行 DAC
│   ├── DDS/                       # 软件 DDS 实现
│   ├── DAC输出波形/               # DAC 输出正弦等任意波形
│   └── DAC钢琴音色/               # DAC 合成钢琴音色
│
├── 📊 数字信号处理
│   ├── FFT模板/                   # FFT 基础工程
│   ├── FFT鉴相/                   # FFT 鉴相
│   ├── FFT扫频/                   # FFT 扫频（幅频响应）
│   ├── FFT扫频Deepseek IIR/       # FFT 扫频 + IIR 滤波
│   ├── AD7606FFT相位差/           # AD7606 采样 + FFT 求相位差
│   ├── AD7606FFT相位差李沙育图/   # 相位差测量 + 李萨如图形
│   ├── FIR滤波完全版/             # FIR 数字滤波完整实现
│   ├── IIR低通滤波/               # IIR 低通滤波
│   ├── IIR系数滤波/               # IIR 系数滤波
│   ├── IIR双缓冲/                 # IIR 双缓冲实时滤波
│   └── IIR最小二乘法/             # IIR 滤波 + 最小二乘法拟合
│
├── 📏 测量仪器
│   ├── 定时器测量频率/            # 定时器输入捕获测频
│   ├── 捕获+dma测频/              # 输入捕获 + DMA 测频
│   ├── 方波频率相位差/            # 方波频率 / 相位差测量
│   ├── LCR/                       # LCR 阻抗测量仪
│   ├── thd测量/                   # THD 总谐波失真测量
│   └── 激光测距/                  # 激光测距
│
├── 🔁 闭环控制
│   ├── PID锁相/                   # PID 锁相环
│   ├── 方波相位差pid/             # 方波相位差 PID 控制
│   └── pwm输出/                   # PWM 输出
│
├── 🖥 显示 / 音频 / 通信
│   ├── 串口二/                    # USART2 串口通信
│   ├── 音乐/                      # 音乐播放
│   └── 音乐频谱/                  # 音乐频谱显示
│
├── 🏆 电赛真题
│   ├── 电赛真题/2018 电流检测/    # 2018 年电赛真题
│   ├── 电赛真题/2021归一化幅值/   # 2021 年电赛真题
│   ├── 23年H题/                   # 2023 年电赛 H 题
│   ├── 25年电赛/                  # 2025 年电赛
│   ├── 25年电赛G题滤波器拟合/     # 2025 年 G 题滤波器拟合
│   └── 2026年电赛32768FFT/        # 2026 年电赛 32768 点 FFT
│
└── 📁 模板/                       # 基础工程模板
```

> 每个子工程内部结构统一为：`Drivers`（驱动）/ `Middlewares`（中间件）/ `User`（应用代码）/ `Projects/MDK-ARM`（工程文件）。

---

## 🏆 电赛真题

| 题目 | 年份 | 说明 |
|------|------|------|
| **电流检测** | 2018 | ADC + DMA 采集 + FFT 频谱分析，电流信号参数测量 |
| **归一化幅值** | 2021 | 加窗 FFT + 谐波分析，幅值归一化测量 |
| **H 题** | 2023 | 电赛 H 题真题工程 |
| **电赛** | 2025 | 2025 年电赛真题工程（含软件流程说明） |
| **G 题滤波器拟合** | 2025 | IIR 滤波器拟合，含 Python 拟合/验证脚本 |
| **32768 点 FFT** | 2026 | 32768 点大点数 FFT 频谱分析 |

真题工程均演示了完整的信号处理链路：

```
ADC 采集 ──▶ DMA 搬运 ──▶ 时域加窗 ──▶ FFT ──▶ 幅度谱 ──▶ 谐波提取
```

---

## 🚀 快速上手

1. **克隆仓库**
   ```bash
   git clone https://github.com/Nomad-joe/STM32H743IIT6.git
   ```

2. **打开工程**：进入任意子工程的 `Projects/MDK-ARM` 目录，双击 `.uvprojx` 用 Keil MDK 打开。

3. **编译下载**：编译通过后，通过 ST-Link / J-Link 下载到开发板。

4. **观察结果**：
   - LED0 闪烁表示程序运行正常
   - 串口助手（115200bps，勾选 DTR）查看 FFT / 测量结果
   - 部分工程在 RGB-LCD 上显示波形与图形

> ⚠️ 若使用 8 寸 RGB 屏，需在 `ltdc.h` 中将宏 `RGB_80_8001280` 置为 1。

---

## 📝 关键代码片段

以电赛真题为例，展示 ADC + DMA + 加窗 FFT 的处理流程：

```c
/* 1. ADC 校准并启动 DMA 采集 */
HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_Buffer, FFT_LEN);

/* 2. 时域加窗（Hann 窗），抑制频谱泄漏 */
for (uint16_t i = 0; i < FFT_LEN; i++) {
    ADC_float[i] *= 0.5f * (1 - arm_cos_f32(2 * PI * i / (FFT_LEN - 1)));
}

/* 3. 复数 FFT 与幅度谱 */
arm_cfft_radix4_f32(&s, FFT_input);
arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
```

---

## 📄 致谢

- 慧勤智远 STM32H743IIT6 小系统板提供的工程模板与资料
- STMicroelectronics HAL 库与 CMSIS-DSP 库
- ARM Cortex-M7 DSP 指令集

## 🧹 仓库范围说明

为保持仓库轻量、干净，以下内容**不纳入版本管理**（已通过 `.gitignore` 排除）：

- **编译产物**：各子工程的 `Output/`、`Listings/` 及 `.o`/`.axf`/`.hex`/`.map` 等中间文件
- **第三方资料**：`外部模块资料/`（数据手册、模块驱动、网盘下载的参考代码等）
- **大体积二进制**：`*.pdf`、`*.zip` 等资料文件

> 编译时在 Keil 中重新 Build 即可生成产物；如需本地清理，可运行各子工程内的 `keilkill.bat`。

---

## 📜 许可

本项目采用 [MIT License](LICENSE) 开源，仅用于学习与竞赛交流。

- 应用层代码（`User/` 等目录）遵循 MIT 许可
- STM32 HAL / CMSIS-DSP 库版权归 STMicroelectronics（BSD-3-Clause）
- 慧勤智远工程模板版权归原厂，仅供学习交流

如引用代码，请注明出处。

---

<p align="center">
  <sub>Built with ❤️ for 电子设计竞赛 · STM32H743IIT6</sub>
</p>
