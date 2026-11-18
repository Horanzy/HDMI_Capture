# IT68051 + GW5A-25 + FT601 USB HDMI 采集卡

本工程是**尚未完成上板验收**的低延迟采集实现。默认通路不攒整帧：IT68051 统一输出 10bit YUV422，FPGA 动态居中裁剪 640×640、无损紧凑打包，经 FT601/USB3 送到 Linux，由 PC 解包。

## 1. 编译期模式

顶层 `rtl/it68051_capture_top.v`：

```verilog
parameter AUTO_FORMAT = 0
```

| 值 | IT68051 输出策略 | FPGA 协议 | 带宽状态 |
|---|---|---|---|
| `0`（默认） | 所有输入统一为 10bit YUV422 | fmt 5，1600B/行，无损 packed YUV42210 | 连续行速率低于 FT601 400MB/s 理论峰值，仍待真机验证 |
| `1` | RGB→RGB10；YUV444→YUV44410；YUV422/YUV420→YUV42210 | fmt 0–4，30bit 槽放入 32bit 容器，2560B/行 | 高刷新连续行可超过 USB 排水能力，不能视为已解决 |

这是**综合期参数**，不是运行时主机命令。切换后必须重新综合、P&R 和生成 bitstream。

IT68051 没有原生 YUV420 TTL 输出枚举；YUV420 输入由芯片转换为 YUV422。实际输入格式从 bank2 AVI DB1 `0x15[6:5]` 读取，不能再使用旧版误读的 bank0 `0x68[7:6]`。

固定 422 仍保留 AVI/INT 服务：

- `0x6B=0x05`：10bit YUV422 输出；
- RGB 输入时 `0x6C[1:0]=2`，YUV 输入时 bypass；
- 手动 CSC 前清 `0x6E[7]`；
- 按 AVI colorimetry 和 RGB 量化范围选择 EVB 的 BT.601/BT.709、full/limited 矩阵，写 bank1 `0x70..0x85`；
- 服务结束清 `0x12/0x11/0x10`；
- INT# 按低有效、开漏处理，以低电平触发服务，避免持续低期间的新事件被遗漏。

BT.2020/扩展色域的 RGB→YUV 矩阵尚未实现，不能宣称支持；YUV 输入保持 CSC bypass。实际颜色和量化范围仍需色条验收。

## 2. 动态居中裁剪

每次视频服务顺序读取 AVI DB1/DB2/DB3/DB4、HActive 和 VActive：

- HActive：bank0 `0x9D + 0x9E[5:0]`；
- VActive：bank0 `0xA4 + 0xA5[5:0]`；
- YUV420 输入时 HActive 寄存器宽度乘 2；
- `x0=(W-640)/2`，`y0=(H-640)/2`；
- 仅在服务完成后、帧边界锁存，不在线接受 PC crop 命令。

当前承诺常见、偶数宽高且不小于 644×644 的输入，例如：

| 输入 | x0 | y0 |
|---|---:|---:|
| 2560×1440 | 960 | 400 |
| 2560×1600 | 960 | 480 |
| 1920×1080 | 640 | 220 |
| 1920×1200 | 640 | 280 |
| 3840×2160 | 1600 | 760 |

RTL 保留四像素对齐，不增加两像素色度相位、边缘补像素或 PC 后处理。非常规分辨率、YUV420 实际映射和色度位置仍需硬件确认。

本地 EDID 仍固定面向 2560×1440@144，通用 EDID 协商未完成。EDID 只参与源端协商，不用来限制实测输入。采集卡前接分配器时，源可读取分配器的 2K120 EDID；只要实际有效画面仍为 2560×1440，crop 坐标相同。分配器的 EDID 合并、HPD、SCDC 和 HDCP repeater 行为必须按具体型号实测。

## 3. 默认视频通路与 YUV422 引脚映射

```text
IT68051 FullBusDDR 60bit
  -> IDDR 输入与 DE 对齐
  -> 动态居中 640×640
  -> 每两个像素保留 Cb/Y0/Cr/Y1 四个 10bit 样本，共 40bit
  -> 80bit -> 128bit 打包
  -> 1024×128bit 异步 FIFO
  -> 四个低字优先的 32bit FT601 总线字
  -> USB3 Bulk IN
  -> Linux 解包为 16bit 容器，保留全部 10bit 样本
```

YUV422 的引脚解释与 RGB 不同。依据 IT68051 数据手册 Table 8-10 / Figure 8-23：

- `QE[29:20]`：Cb；
- `QE[19:10]`：Y0；
- 另一像素的 `QE[59:50]`：Cr；
- 另一像素的 `QE[49:40]`：Y1；
- `QE[9:0]` 和 `QE[39:30]` 在该模式下未使用。

因此默认综合只保留 40 个有效 IDDR，20 个 NC 通道被优化是预期行为。RGB/YUV444 自动分支仍保留 30bit 槽的原始通道含义。

`video_stream.v` 中的 `FPGA_IMAGE_PROCESSING_INSERTION_POINT` 仅为未来扩展标记；当前 FPGA 不做 CSC、缩放、色度重采样或截位。

## 4. USB 协议

所有整数均为小端。LINE/STATUS 可跨任意 USB/URB 边界，主机不得依赖 USB 包边界。

| 字节 | 含义 |
|---|---|
| 0..3 | `frame_idx` |
| 4..5 | LINE：`line_idx=0..639`；STATUS：`0xFFFF` |
| 6..7 | 保留 0 |
| 8 | bit0 EOLF，bit1 OVERRUN，bit4:2 格式，bit7:5 类型 |
| 9..11 | 固定同步标记 `C2 51 A5` |
| 12..13 | FPGA 累计丢行计数 `drop_cnt` |
| 14..15 | 设备状态字 |
| 16.. | LINE 载荷；STATUS 无载荷 |

| fmt | 内容 | 每行载荷 | PC 输出 |
|---|---|---:|---|
| 0 | raw YUV422 槽 | 2560B | UYVY8 |
| 1/2 | raw RGB/YUV444 源模式 | 2560B | 8bit 四字节格式 |
| 3/4 | raw RGB10/YUV44410 | 2560B | 打包 10bit 四字节格式 |
| 5 | packed YUV42210 | **1600B** | 每样本 little-endian u16，低 10bit 有效，顺序 Cb/Y0/Cr/Y1 |

默认 fmt 5 每行含 16B 头共 1616B。160 个 PCLK 恰好生成 100 个 128bit 载荷字，最终无残留。`drop_cnt` 是累计值；OVERRUN 标记下一成功接收行，接收器还会用完整行位图和缺行检查拒绝不完整帧。

STATUS 是 FPGA 每帧主动发送的 16B header-only 包。OUT 命令仅保留 SOFT_RESET；SET_CROP 已删除，GET_STATUS 不需要主动请求。

## 5. FIFO 与带宽结论

GW5A-25 的 BSRAM 总量为 1008Kbit=126KiB，**存不下 640×640 整帧**：

- 旧 32bit 容器：1,638,400B（1.5625MiB）；
- 无损 packed YUV42210：1,024,000B（1000KiB）；
- 8bit UYVY：819,200B（800KiB）。

板卡没有外部帧存，因此不能靠“加大 FIFO 存一帧”解决问题。当前 FIFO 为 1024×128bit=16KiB，只吸收短期时钟/USB 抖动。

对本地 2K144 EDID 的 580MHz TMDS、HTotal=2720、VTotal=1481 时序：

- FullBusDDR PCLK=145MHz，行周期约 4.68966µs；
- 旧 2576B 连续行输入约 549.29MB/s，理论窗口缺口可超过片上 RAM；
- 新 1616B 连续行输入约 **344.59MB/s**，低于 FT601 32bit/100MHz 的 400MB/s 理论峰值；
- 新格式含行头的整帧平均速率约 **148.91MB/s@143.981Hz**。

连续行速率低于峰值只是必要条件，不是持续吞吐保证。FT601 TXE 背压、USB 主机、调度、URB 数量和 libusb 开销仍需长时间真机测量。自动 fmt 0–4 分支仍有高刷新连续行溢出风险。

## 6. Linux 接收与离线测试

```bash
python sw/test_linux_rx.py
python model/check_packed422.py
python model/check_csc_rom.py
python edid/verify_edid.py
python sw/linux_rx.py --frames 10 --save capture.bfra
python sw/linux_rx.py --urb-lines 2 --stats-every 1
```

离线测试覆盖六种格式、fmt 5 的 1616B 分块、任意字节分包、垃圾前缀重同步、STATUS、缺行丢帧、NumPy/纯 Python 解包，以及 YUV422 正确 Cb/Cr 槽位。

`BFRA` 文件每帧有 16B 头。fmt 5 随后为 Cb/Y0/Cr/Y1 四个 u16 容器，保留 10bit，不再默认截成 UYVY8。

raw libusb 路径使用异步 IN URB；取消后等待回调，未确认不再引用前不释放 transfer，STALL 先 `clear_halt`。Python/libusb 不能保证严格 8kHz 命令抖动；当前接收也不依赖主机周期轮询。

## 7. 可复现构建与当前验证结果

`test.gprj` 是 GW5A-LV25UG324C2/I1 工程，Top 为 `it68051_capture_top`。SSPI/I2C/CPU 必须作为 regular IO。`run_pnr.tcl` 每次显式 `run syn` 后 `run pnr`，避免复用另一参数分支的旧网表。

```powershell
python run_flow.py --name packed422_board_est --timeout 100 --stall 30
```

隔离自动测试使用 100 秒总超时、CST 解析后 30 秒无进展判定 stalled，只终止本轮 `gw_sh` 进程树。

当前 `constr/timing.sdc` 是 **100MHz 板级估算约束**，采用 150ps/in、70ps 残余失配和约 600ps FT601 往返估算，不是实际叠层/走线 signoff。FT601 DATA 仅施加入延迟，DATA/BE 均施加输出延迟；BE 不采入是因为命令 RTL 不采 BE。

最终默认固定 422 候选：

- 新鲜综合确认 `PACK_422=1`；`AUTO_FORMAT=1` 分支也单独综合通过；
- 40 IDDR、39 IODELAY、80 个 I/O register FF、12/56 BSRAM；
- 0 setup、0 hold；最差 setup `+0.062ns`，最差 hold `+0.182ns`；
- bitstream：6,389,016 bytes；
- SHA256：`61520C99F7E2305AFB2B9C937A1C39F7F16222B567E34D0E9227B17EDA3926D6`；
- 证据：`validation/packed422_board_est/`；bitstream：根目录 `test_packed422.fs`。

综合中的 A_B/B_B 未使用警告对应 YUV422 NC 通道；RX_HSYNC、SIWU_N、WAKEUP_N 未使用是当前功能取舍。`csc_matrix_rom` 的层级 swept 提示已用常数输出对照综合证明会改变网表，不代表矩阵功能被整体删除。上述结果仍不是上板验收。

## 8. 上板验收顺序

1. I2C ACK、EDID 写完后 HPD 才有效，源端能读到 EDID。
2. 用灰阶、RGB、黄青品红和高饱和色条核对 Cb/Cr、Y、奇偶像素和行顺序。
3. 覆盖 1080p、1200p、2K、4K 常见输入及分配器 2K120 工况，检查动态居中和热切换。
4. 检查 BT.601/709、full/limited、YUV420→422；BT.2020 不支持项不得混入通过结论。
5. 长时间记录 USB 吞吐、URB 错误、`drop_cnt`、缺行、重同步、CPU 和 FIFO 溢出。
6. 用可视计时器/LED 或主机时间戳实测端到端延迟，不能由行间隔代替。
7. 拔插 HDMI、源重启、I2C NACK、USB 重新枚举、SOFT_RESET，检查恢复。
8. 自动分支需另行构建并验证格式切换；不能把默认分支结果套用到 fmt 0–4。

完整 RTL/门级仿真、真实 USB 持续吞吐、颜色、热插拔和端到端延迟尚未完成，当前 bitstream 只能称为**估算约束下的候选固件**。
