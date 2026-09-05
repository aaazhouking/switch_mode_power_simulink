# 开关电源仿真

开关电源的闭环仿真工程。主电路用 Simulink 搭建，**控制器用 C 代码实现**，通过
`C Caller` 块接入仿真，同一份 C 代码可直接移植到 MCU / DSP 使用。

> 目前仅实现了 Buck（降压）拓扑，后续会继续优化与扩展。目前是初始demo版本 会有点粗糙

## 目录结构

```
buck_simulink/
├── buck.slx      # Simulink 模型：Buck 主电路 + PWM + C Caller 控制环
├── pid.h         # PID 控制器接口（extern "C"）
└── pid.cpp       # PID + 过流保护实现（纯 C，无外部依赖）
```

## 主电路参数（Buck）

| 参数 | 数值 |
| --- | --- |
| 输入电压 | 100 V |
| 输出电压给定 | 20 V |
| 开关频率 | 20 kHz |
| 滤波电感 L | 1 mH |
| 输出电容 C | 1 µF |
| 负载 R | 10 Ω |

## 使用方法

1. 打开 `buck.slx`，确认 `C Caller` 块指向 `pid.cpp` / `pid.h`。
2. 运行仿真，在 `Waveforms` 里观察输出电压是否稳定在 20 V。

## 控制接口

```c
double duty = pid_step(vref, vout, iout, dt);  // 返回占空比 [0, 0.95]
```

`pid.h` 里包含 PID 参数设置、限幅、过流保护等接口。MCU 移植时，在 PWM 中断里采样
电压/电流后调用 `pid_step()`，把返回的占空比写入比较寄存器即可。
