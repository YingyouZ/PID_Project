# J280 倒立摆控制系统

基于高云 GW2A-55 FPGA 与 J280 姿态控制套件的倒立摆控制系统，包含：

- 自动起摆、角度/位置串级 PID、安全保护；
- ESP32-S3 手机网页、WebSocket 实时波形；
- ESP-SR 离线语音控制（启动 / 向左 / 向右 / 停止）；
- FPGA ↔ ESP32 二进制遥测/远控链路。

## 目录

```text
PID_Project/
├─ src/                         主工程 Verilog / 约束
├─ PID_Project.gprj             主工程 Gowin 工程
├─ PID_Project.fs               主工程 bitstream
├─ build_main.tcl               主工程命令行构建脚本
├─ esp32_extension/
│  ├─ offline_link/
│  │  ├─ rtl/                   link / top / control_loop_ctrl
│  │  ├─ web/index.html         手机网页源文件
│  │  ├─ PID_Link.fs            PID_Link bitstream
│  │  ├─ PID_Link.gprj          PID_Link Gowin 工程
│  │  ├─ build_link.tcl         命令行构建脚本
│  │  ├─ PID_Link.cst           引脚约束
│  │  └─ PID_Link.sdc           时序约束
│  ├─ esp32_voice/              ESP32 源码与配置
│  ├─ firmware/                 ESP32 可烧录固件
│  ├─ src/                      当前 PID 源码副本
│  ├─ tools/                    串口抓包工具
│  ├─ build_esp32.ps1           ESP32 构建脚本
│  └─ README.md                 扩展部分说明
└─ 说明.txt                     烧录/中心角注意事项
```

## 构建

### 主工程

```powershell
& "D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build_main.tcl
```

### PID_Link

```powershell
cd esp32_extension\offline_link
& "D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build_link.tcl
```

### ESP32

```powershell
cd esp32_extension
.\build_esp32.ps1
```

## 烧录

```text
主工程：   PID_Project.fs  -> J280 FPGA
扩展工程： PID_Link.fs     -> J280 FPGA 远控工程
ESP32：    esp32_extension/firmware/ -> ESP32-S3
```

详细接线、工具链和验收说明见 [esp32_extension/README.md](esp32_extension/README.md)。

## 注意

`说明.txt` 中记录的物理中心角设置需按实际测试工具确认；烧录 `.fs` 前先阅读该文件。
