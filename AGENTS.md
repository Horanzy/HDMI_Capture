# USB HDMI 采集卡工程索引

## 分支状态

当前文件描述 `feat/ddr3-line-buffer`。`main` 仍是无 DDR3 的固定 422 基线
（`32917c8`），不要把本文件的 DDR 通路、IP 或引脚要求套用到 `main`。

当前选定 DDR 候选证据为 `validation/ddr3_mdrp_over5/`；它仍是板级候选，
不是已通过 DDR 校准或硬件吞吐验收的固件。

## 当前架构（feature 分支）

```text
IT68051 FullBusDDR -> iddr_bus_in -> video_stream
    -> 16KiB 异步 FIFO -> ddr3_line_buffer -> 板载 2Gbit DDR3 环形缓冲
    -> 16KiB 异步 FIFO -> FT601 -> Linux/PC
```

- 默认 `AUTO_FORMAT=0`：固定 10bit YUV422，fmt 5 无损 packed YUV42210，
  16B 头 + 1600B 载荷/行。
- `AUTO_FORMAT=1`：保留 fmt 0–4 raw-slot 分支；高刷新连续行不能沿用 DDR
  默认通路的带宽结论。
- DDR3 **不攒整帧**。`ddr3_line_buffer` 把 2Gbit 空间当作 128bit 字环形队列；
  默认 `RING_WORDS_LOG2=24`、读水位 64 字、逻辑积压上限
  `MAX_BACKLOG_WORDS=256`（4KiB）。
- 写侧两项预取、读侧多 outstanding、输出 FIFO 空间预留；双向同时工作时
  读写交替，单向可占满命令槽。
- `backlog_throttle` 同步回视频域阻止新行，整行丢弃计入 `drop_cnt`；
  已进入 FIFO/ring 的数据继续排空，写指针不得覆盖未读数据。
- `init_calib_complete` 同步到 USB 域门控流输出。DDR3 只吸收瞬时速率差和
  短时 USB 抖动，不突破长期平均带宽守恒，不承诺无限缓冲。

## 冻结决策

### 时钟与 PLL

- DDR PLL 输入只能是 T9 50MHz 有源晶振；禁止使用 FT601 100MHz。
- PLL：VCO=800MHz，`clkout0=50MHz` 控制时钟，`clkout2=400MHz` memory clock；
  PHY 1:4 分频得到 100MHz UI。
- 只使用 GW5A-25 官方**变体 B**外引 MD 总线 PLL wrapper。禁止恢复含内部
  `PLL_INIT` 的变体 A；MD 总线只能有一个驱动者。
- 加密 `ip/pll_mDRP_intf.v` 的实际端口为
  `clk/rst_n/pll_lock/wr/mdrp_inc/mdrp_op/mdrp_wdata/mdrp_rdata`。以该文件
  端口为源，不要照抄只有 `pll_stop/mdrdo` 等旧说明。
- `pll_stop` 从 PHY 域进 50MHz 域先过两级同步；同步 stop 的相对边沿产生
  一拍 `wr`，并在 PLL lock 未建立时保持 `wr=0`。
- PHY 的 `pll_lock` 使用原始 lock；mDRP 控制使用同步后的 lock。
- `pll_mDRP_intf.rst_n` 固定为 1，PLL wrapper `reset` 固定为 0，与官方
  参考流程一致。
- PLL wrapper 和 `.ipc` 是已核对资产。Gowin 1.9.11.03 对旧 `.ipc` 的
  `read_ipc` 会崩溃，`ip/gen_ips.tcl` 只重新生成 DDR PHY，禁止自动覆盖 PLL。

### DDR PHY 与缓冲

- DDR3-800：`Memory_Clock=400MHz`、1:4、x16、BL8、128bit 用户字。
- 每次用户命令搬一个 16B 字，DDR 地址步进 `+8`；不要退回每字等读返回的
  旧 FSM，那会把双向吞吐限制在约 200MB/s。
- DDR 球位以 `constr/ddr3.cst` 和已核对的 ACG525 `DDR3.csv` 为准：单端
  `SSTL15_I`、差分 `SSTL15D_I`。不要再引用早期错误球位。
- UI 域复位释放必须经 `ddr_ui_rst_sync`，不能直接把 USB 域释放当 UI 异步复位。

### 时序与构建

- 实际硬件时钟固定为视频不超过 148.5MHz、T9/PLL/DDR/UI=50/400/100MHz。
- `constr/timing.sdc` 有意过约束为视频 150.602MHz、T9 52.499MHz
（派生 UI 104.998MHz），用于给优化器留余量。它是优化目标，不是硬件超频。
- 板级 I/O 数值是估算约束，不是叠层/长度 signoff。
- `run_pnr.tcl` 必须显式固定 `place_option=0`、`route_option=0` 和
  `correct_hold_violation=1`，避免复用上一次试验的工程状态。
- 当前选定结果：0 setup/0 hold，最差 `+0.032ns/+0.143ns`，
  实际 Fmax 视频 151.334MHz、UI 105.526MHz。不得把该余量当成硬件实测结论。

## 权威目录

后续只修改 `C:\Users\i1761\Documents\hdmi_capture`。
`hdmi_capture_auto_test` 是隔离实验/参考资料区，Gowin 生成目录不是权威源码。

### RTL

- `it68051_capture_top.v`：顶层、DCE、复位、CDC、FT601、DDR/mDRP 接线。
- `iddr_bus_in.v`：60bit DDR 输入；默认 YUV422 有 20 个 NC 通道。
- `video_stream.v`：动态 crop、fmt 0–5、80→128bit 打包、行/状态包、
  `ddr_throttle` 整行门控。
- `ddr3_line_buffer.v`：DDR3 ring、预取、多 outstanding、积压水位和背压。
- `async_fifo_dc.v`：1024×128bit 双时钟 FIFO，不是整帧缓冲。
- `csc_matrix_rom.v`、`init_ctrl.v`、`it68051_init_rom.v`、`edid_init.v`：
  CSC、初始化、EDID、HPD/热插拔。
- `ft601_master.v`、`ft601_cmd_rx.v`、`i2c_master_od.v`：主机链路与板级控制。

### IP（全部保留，不要当垃圾删）

- `ip/gowin_pll_ddr3/gowin_pll_ddr3.v`：变体 B 外引 MD wrapper；
  `PLLA_inst` 直接位于该层，SDC 路径是 `u_ddr_pll/PLLA_inst/...`。
- `ip/gowin_pll_ddr3/gowin_pll_ddr3.ipc`、`gowin_pll_ddr3_tmp.v`：
  已核对参数和例化模板。
- `ip/pll_mDRP_intf.v`：官方加密 mDRP 适配器，可综合。
- `ip/ddr3_memory_interface/`：DDR PHY `.ipc`、生成 `.v`、`*_tmp.v` 和
  `.vo` 仿真网表。当前工具生成不稳定，已提交的是权威副本。
- 已删除的 `pll_init.v`、`gowin_pll_ddr3_mod.v`、`_mod_tmp.v`、
  `gowin_pll_ddr3.mod` 属于变体 A，不要从旧工程恢复或重新加入 `test.gprj`。

### 约束与构建

- `constr/capture.cst`、`constr/ddr3.cst`、`constr/timing.sdc`
- `test.gprj`：当前 RTL 加 `ip/pll_mDRP_intf.v`、新 PLL wrapper、DDR PHY
  和三个约束文件；不得引用已删除的变体 A 文件。
- `run_flow.py`、`run_syn.tcl`、`run_pnr.tcl`：隔离运行、超时/停滞处理；
  P&R 前强制重新综合并固定选定策略。

```powershell
python run_flow.py --name ddr3_mdrp_over5_release --timeout 180 --stall 30
```

### 主机、测试与 EDID

- `sw/linux_rx.py`、`sw/test_linux_rx.py`：六格式、任意分包、完整帧校验。
- `tb/ddr3_line_buffer_tb.v`：4000 字随机 ready/延迟/回卷自校验。
- `tb/ddr3_line_buffer_throughput_tb.v`：20000 字持续满速自校验。
- `model/check_packed422.py`、`model/check_csc_rom.py`、`edid/verify_edid.py`。

```powershell
python sw/test_linux_rx.py
python model/check_packed422.py
python model/check_csc_rom.py
python edid/verify_edid.py
```

## 验证边界与证据

`validation/README.md` 是目录索引。当前选定是
`validation/ddr3_mdrp_over5/`；`ddr3_mdrp_release/` 是原始约束基线；
`packed422_board_est/` 只属于 `main` 基线。

通用缓冲仿真和 P&R 通过不等于真实 DDR 校准、DRAM 总线效率、USB 持续
吞吐或端到端延迟通过。`result.json` 的 `passed` 只表示工具流程跑完。
已删除的 `ddr3_bringup/` 和 `ddr3_800_pipelined/` 仅保存在 Git 历史中，
不得作为当前或合格的验证依据。

## 上板验收顺序

1. 确认 PLL lock、`pll_stop` 双边沿与 mDRP `wr`、MD 写事务和
   `init_calib_complete`；校准失败时先查这条链，不要先怀疑视频通路。
2. 用完整 DDR 读写图形/地址游走验证数据完整性和环回卷，再试持续吞吐。
3. 2K120/2K144 长跑，记录 `drop_cnt`、缺行、重同步和 FIFO 水位；
   这是当前吞吐天花板是否足够的直接判据。
4. 记录 USB 持续吞吐、TXE 背压、URB 错误、热插拔和 SOFT_RESET 恢复。
5. 用色条核对 Cb/Cr、Y、奇偶像素和行顺序；覆盖 1080p、1200p、2K、4K
   及分配器工况。
6. 用可视计时器或主机时间戳测端到端延迟；不能用行周期代替。
7. 未完成上述上板项前，只能称候选固件，不能称已验收固件。

## 清理规则

- `.gitignore` 已覆盖 Gowin 构建目录、`.fs/.bit`、`.html/.log` 和
  macOS `.DS_Store`；不要提交生成物。
- Gowin IP 三元组、加密 mDRP、PLL wrapper 和 `.ipc` 是源码资产，不是垃圾。
- 删除验证目录前先更新索引并确认 Git 历史可恢复，不要只删文件不留说明。
- 早期 `rgb2yuv422.v`、`timing_debug.sdc`、不同步的初始化表和历史诊断
  SDC 已被取代，不要从旧工程恢复。
