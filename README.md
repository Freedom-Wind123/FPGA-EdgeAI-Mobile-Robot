# FPGA × 边缘 AI 移动机器人（GW5A + RK3588）

一套「FPGA 硬实时控制 + 边缘 AI 认知」的移动机器人工程：高云 GW5A（小梅哥 ACG720）承担 1 kHz 控制环、图像流水线与避障状态机，RK3588 承担 YOLO 目标识别与语义任务分解，两者通过串口帧协作。

> 本仓库只包含源码与工程文件，不含模型权重、不含 Gowin/IP 生成的中间产物；综合结果不代表时序收敛或上板验证。

## 目录结构

```
baseline/            保留的原始基线（未改动的底盘 RTL、摄像头/HDMI 通路、约束与 Gowin 工程）
  chassis/rtl/       底盘控制 RTL（PID、编码器、IMU、避障、遥测）
  rtl/               摄像头 + 显示顶层
  src/               OV5640 / DDR3 / DVI 等厂商 IP 与参考代码
  constraints/       原始 CST / SDC
fpga/                由 tools/prepare_board.py 生成的整合工程
  rtl/               自主导航顶层与各功能模块（生成物）
  constraints/       合并后的 robot.cst / robot.sdc（生成物）
  tests/             Verilog testbench（手写，非生成）
  robot_board_top.gprj / synth.tcl（生成物）
reference/           仿真用 IP 桩（ip_stubs.sv），替代厂商 IP 以便 iverilog 展开
rk3588/
  vision/            C++ 视觉与语义节点（CMake，依赖 OpenCV + rknnrt）
  fpga_link.py       上位机串口链路工具
  mission_console.py 任务控制台
  observation_export.h  与 vision/include/ 下同名文件一致（历史遗留，两处都在用）
tools/
  prepare_board.py   由 baseline 生成 fpga/ 下的工程、RTL 与约束
  run_tests.py       iverilog 仿真 + 板级展开 + Python 协议测试
  build.py           Gowin 综合（默认不烧录）
```

## 硬件平台

| 部件 | 型号 |
|---|---|
| 主控 FPGA | 高云 GW5A（小梅哥 ACG720 开发板） |
| 边缘计算 | RK3588（6 TOPS NPU） |
| 底盘 | 四轮麦克纳姆轮套件，AT8236 驱动 |
| 相机 | OV5640（DVP，FPGA 侧）/ USB 免驱相机（AI 侧） |
| 雷达 | RPLIDAR C1（UART 460800） |
| 姿态 | ICM-42688-P（SPI 1 kHz） |
| 测距 | 5× VL53L1X（I²C，TCA9548A 复用）+ 超声波 |

## 工具链

| 用途 | 依赖 | 默认路径（需按本机修改） |
|---|---|---|
| 综合 | Gowin EDA V1.9.12 | `F:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe` |
| 仿真 | Icarus Verilog | `F:/iverilog/bin/iverilog.exe`、`vvp.exe` |
| 脚本 | Python 3.10+ | — |
| RK3588 视觉 | CMake ≥ 3.16、OpenCV、librknnrt | 见 `rk3588/vision/CMakeLists.txt` |

> `tools/build.py` 与 `tools/run_tests.py` 里的路径是硬编码的，换机器要改这两处；`build.py` 还要求 `--stage` 是 `F:/27_qiansai/build_runs` 下的 ASCII 路径。

## 快速开始

```bash
# 1. 由 baseline 生成 fpga/ 工程（会覆盖 fpga/rtl、fpga/constraints、.gprj、synth.tcl）
python tools/prepare_board.py

# 2. 仿真与协议测试（产物写入 evidence/）
python tools/run_tests.py

# 3. 综合（只综合，不烧录）
python tools/build.py --stage F:/27_qiansai/build_runs/run01
#    加 --resource-check 仅做资源评估；加 --pnr 额外跑布局布线并解析时序违规数

# 4. RK3588 侧视觉节点
cd rk3588/vision
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
```

## 通信协议（摘要）

- FPGA ← RK3588：`0xAA55` 视觉目标帧（74 B）、`0xAA56` 语义命令帧（24 B），921600 8N1，CRC-16/X25
- FPGA → RK3588：以 `$` 开头、逗号分隔的文本遥测行（位姿、姿态、轮速），115200 8N1
- 视觉目标超过 200 ms 未更新即判失效；速度指令须通过最大线速度/角速度与障碍距离对应的允许速度裁剪

## 状态与限制

- `HARDWARE_CONFIRMED` 参数默认 0，综合通过 ≠ 时序收敛 ≠ 上板可用
- 视觉侧强逆光下红色分割会失效，遮挡时跟踪会丢
- 串口设备名为硬编码，掉线需人工重连

## 许可证

[MIT](./LICENSE) © 2026 Freedom-Wind123
