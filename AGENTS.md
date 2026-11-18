# USB HDMI 采集卡工程索引

## 当前架构

IT68051 FullBusDDR → GW5A-25 动态居中 640×640 → 128bit 异步 FIFO → FT601 245 同步 FIFO → Linux/PC 解包。

- 默认 `AUTO_FORMAT=0`：IT68051 固定输出 10bit YUV422，FPGA 使用 fmt 5 无损 packed YUV42210，16B 头 + 1600B 载荷/行。
- `AUTO_FORMAT=1`：保留 RGB/YUV444/YUV422 自动输出与 fmt 0–4 raw-slot 协议，2560B 载荷/行；高刷新连续行溢出问题仍未解决。
- 输入格式来自 bank2 AVI DB1 `0x15[6:5]`，有效尺寸来自 bank0 `0x9D/0x9E`、`0xA4/0xA5`。
- crop 由 FPGA 按 `(W-640)/2`、`(H-640)/2` 自动计算，不再接受 SET_CROP。
- INT# 低有效、开漏，按低电平服务；清 `0x12/0x11/0x10`。
- STATUS 为 16B header-only；同步标记 `C2 51 A5`；头内低 16bit 保留字现携带累计 `drop_cnt`。

完整协议、带宽、构建和验收说明见 `docs/README.md`。

## 权威目录

后续只修改 `C:\Users\i1761\Documents\hdmi_capture`。`hdmi_capture_auto_test` 是隔离实验/审查区，Gowin 生成目录不是权威源码。

### RTL（全部加入 `test.gprj`）

- `it68051_capture_top.v`：顶层、DCE、复位、CDC、FT601 三态、`AUTO_FORMAT`。
- `iddr_bus_in.v`：60bit DDR 输入；默认 YUV422 仅 40 个有效通道，20 个 NC 会被优化。
- `video_stream.v`：动态 crop、fmt 0–5、40bit YUV422 样本对、80→128bit 打包、行/状态包。
- `csc_matrix_rom.v`：EVB BT.601/709、full/limited 四组 RGB→YUV 矩阵。
- `async_fifo_dc.v`：1024×128bit 双时钟 FIFO，不是整帧缓冲。
- `ft601_master.v`：128bit 拆成四个低字优先的 32bit 总线字。
- `ft601_cmd_rx.v`：共享总线换向；仅 SOFT_RESET 有效。
- `i2c_master_od.v`：开漏 I²C 位引擎。
- `init_ctrl.v`：初始化、EDID、HPD、AVI/尺寸服务、CSC、动态 crop、INT 与热插拔。
- `it68051_init_rom.v`：当前 221 项初始化 ROM。
- `edid_init.v`：固定 2K144 EDID ROM 与写入脚本。

### 约束与构建

- `constr/capture.cst`
- `constr/timing.sdc`：100MHz 板级估算约束，不是 signoff。
- `run_flow.py`、`run_syn.tcl`、`run_pnr.tcl`：隔离运行、超时/停滞处理；P&R 前强制重新综合。

```powershell
python run_flow.py --name packed422_board_est --timeout 100 --stall 30
```

### 主机、测试与 EDID

- `sw/linux_rx.py`：异步 IN URB、六种格式、任意分包重同步、完整帧校验、10bit PC 解包。
- `sw/test_linux_rx.py`：无硬件协议回归，含 NumPy 和纯 Python 路径。
- `model/check_packed422.py`：YUV422 引脚映射、160 PCLK→100×128bit、动态居中模型。
- `model/check_csc_rom.py`：四组 CSC 矩阵逐字节对照 EVB。
- `edid/verify_edid.py`：EDID 独立校验。

```powershell
python sw/test_linux_rx.py
python model/check_packed422.py
python model/check_csc_rom.py
python edid/verify_edid.py
```

## 当前验证边界

最终默认候选的板级估算 P&R 为 0 setup/0 hold；资源、哈希和报告见 `docs/README.md` 与 `validation/packed422_board_est/`。自动分支仅单独综合通过。

完整 RTL/门级仿真、真实 USB 持续吞吐、色条/CSC、YUV420、热插拔、分配器、自动模式切换和端到端延迟尚未完成。候选 bitstream 不得称为已硬件验收固件。

## 已清理

早期 `rgb2yuv422.v`、`timing_debug.sdc`、不同步的 `init/it68051_init_table.txt` 和分层诊断 SDC 不保留，不要从旧工程恢复。`.gitignore` 中的构建产物和 Python 缓存规则继续有效。
