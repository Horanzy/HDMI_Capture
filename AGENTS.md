# AGENTS.md — USB HDMI 采集卡固件交接文档

## 1. 项目一句话

USB3.0 HDMI 采集卡（IT68051 → GW5A-25 FPGA → FT601Q）的 Gowin 固件：把 2K144
HDMI 流裁剪成 640×640、RGB 转 YUV422、按**行原子包**推过异步 FIFO，从 FT601
245 同步 FIFO 以 USB3.0 Bulk IN 吐给主机；PCB 已投产，本仓库是**已修好综合洪水
根因的干净重建版**（旧工作目录 `C:\Users\i1761\Documents\fpga_fw\` 已删除，本目录
`C:\Users\i1761\Documents\hdmi_capture\` 是唯一项目目录）。

### 数据流架构

```
IT68051(U5) 双像素DDR TTL输出, PCLK≈148.5MHz @2560x1440 CVT-RB
  → 60路 QD[59:0] DDR采样（iddr_bus_in，RTL推断IOLOGIC输入寄存器，Q0上升沿/Q1下降沿）
  → video_stream：裁剪640×640（x0/y0默认960/400，带内可设）
      RGB888→YUV422（BT.601/709，UYVY）
      行原子包 1296B = 16B包头 + 1280B载荷（162×64bit字）
  → async_fifo_dc：2048×64bit BSRAM异步FIFO，格雷码指针，148.5M ↔ 100M
  → ft601_master：245同步FIFO突发写（lo 32bit先发，1字/时钟=400MB/s峰值）
  → FT601Q(U2) → USB3.0 Bulk IN（MPS=1024，burst 15；1296B行包=1个1024B全包+1个272B短包）
```

协议要点：**行原子、永不攒帧**——一行凑齐1296B立即写FIFO，绝不缓存整帧；
FIFO空闲不足（行包162字+EOLF行额外162字STATUS包余量）时**整行丢弃**并在下一
行包头打 OVERRUN 位（bit1），绝不反压视频通路；每帧最后一个窗口行（EOLF，
line_idx=639）之后追加一个 STATUS 包：与行包同样 1296B/162 字，**完整 16B 头**
（w1 = line_idx=16'hFFFF，w2 flags=8'h21 即 type=STATUS+EOLF），载荷 160 字全零
（主机按 hdr=None 丢弃 STATUS 载荷）。

### 包格式（权威实现 = sw/linux_rx.py Receiver.feed，RTL已与其逐位对齐）

线序字节流（无包边界依赖），按小端 u32 解出 w0..w3：

| 线上字节 | 字段 | RTL 对应 |
|---|---|---|
| 0..3   | w0 = frame_idx (u32)             | w64[0][31:0] |
| 4..5   | w1低16位 = line_idx（0..639）    | w64[0][47:32] |
| 6..7   | rsv=0                            | w64[0][63:48] |
| 8      | w2低8位 = flags（bit0=EOLF, bit1=OVERRUN, [7:5]=type: 0=LINE/1=STATUS）| w64[1][7:0] |
| 9..11  | rsv=0                            | w64[1][31:8] |
| 12..13 | rsv=0                            | w64[1][47:32] |
| 14..15 | w3高16位 = status（设备状态字）  | w64[1][63:48] |
| 16..1295 | 载荷1280B = 320×UYVY对 (Cb,Y0,Cr,Y1)；STATUS 包此处为全零 | w64[2..161] |

64bit FIFO 字的线序：**低32bit先发**（ft601_master 拆字），FT601 总线上
DATA[7:0] 先走 → 主机端按小端解出。STATUS 包首字 flags=8'h21
（type=1 + EOLF=1，主机只看 type，EOLF 位无害）。

status_bits（init_ctrl，w3高16位）：[15:12] init_err_cnt，[11:8] i2c_nack_cnt
（含EDID回读不一致），[7] stream_ok，[6] htot_ok，[5] vid_stable，
[4] init_done（含EDID上载），[3] hpd_active，[2] online，[1] i2c_fail，
[0] replay_active。

主机→设备命令（Bulk OUT 16字节，ft601_cmd_rx，EN_CMD_PATH 参数门控）：
`'BFCP' | cmd | len=8 | x0(u32,%4) | y0(u32) | rsv(2)`；
cmd 1=SET_CROP 2=SOFT_RESET 3=GET_STATUS（被动）。

控制流：init_ctrl（100M USB域）→ 复位IT68051 → I2C回放EVB初始化表（ROM）→
EDID 上载（自制 2560×1440@144 EDID，经 0xA8 从机写入 IT68051 EDID SRAM，
镜像 EVB iTE6805_EDID.c 流程；**整个阶段在 HPD 拉低之前完成**）→ 等视频稳定
→ HPD 拉低（板 Q3 反相后源端看到有效）。流丢失或主机 SOFT_RESET → 释放 HPD
→ 完整重初始化（含重写 EDID）。

## 2. 硬件事实速查（PCB 已投产，不可再改板）

- FPGA：**GW5A-LV25UG324C2/I1**（Gowin 教育版 V1.9.11 编译）。
- RX_PCLK = 球 **M10** = IOB26A/**GCLKT_9B**（专用全局时钟输入，DCE 缓冲）。
- FT601_CLK = 球 **H4** = IOT1A/**GCLKT_15**（同上，DCE）。
- I2C 从机地址：**0x90 写 / 0x91 读**（7位0x48）。PCADR 拉低：R8(0Ω，上拉
  OVDD33) 为 **DNP 不焊**，R6(10k) 下拉到 GND（误焊 R8 会变 0x92 → NACK 先查这个）。
- HPD：FPGA 引脚 it68051_hpd(N1) **驱低有效**——低 = Q3 关断 = HDMI HPD 高
  （源端看到有效）。上电未初始化前脚为高（HPD 低）。
- FT601Q：**245 同步 FIFO 模式**（GPIO0/1=00 下拉配置），CLK=100MHz、32bit、
  IN MPS=1024 / burst 15；BE[3:0] **高有效**；VID/PID 默认 0403:601F。
- GW5A 八球复用：SSPI/I2C/CPU 双功能脚（**N6/P7/N5/R3/T3/R5/U5/V5**）已在工程
  勾选 **"as regular IO"**（Dual-Purpose Pins）；不勾会吃掉 A_B3/A_G11/A_B9/
  A_G9/A_G7/A_G6/A_G5/A_G8 等 QD 输入。
- bank0/1 = 1.5V DDR3 域**整域禁用**（一个脚都没用）；其余 bank 全部
  **LVCMOS33 @3.3V**。
- JTAG 球（A94/A96/A98/A100）为器件专用，不在 .cst 内。

## 3. 工具坑清单（教育版 V1.9.11 实测）

1. **GW5A 无 BUFG 原语**：裸例化 BUFG 直接失败（EX3937）。全局时钟缓冲必须用
   **DCE**（UG306 3.1），端口 CLKIN/CE/CLKOUT，**CE 高有效**，常开接 `1'b1`。
   顶层 `u_buf_vid`/`u_buf_usb` 已如此。
2. **显式 IDDR 原语会被静默丢弃**（本次洪水根因之一）：直接例化
   `IDDR u_iddr(.Q0,.Q1,.D,.CLK)` 综合器不报错、不警告，把原语丢掉 →
   word_r/word_f 无驱动 → 整条视频通路反向剪光 → CV0016 洪水
   （"Input A_B2 is unused"×60）。**当前版本用 RTL 推断写法**（iddr_bus_in.v
   内 posedge/negedge 寄存器对 + `(* syn_useioff=1 *)`，文件头注释有完整记录
   与回切判据）。若推断没落 IOLOGIC 或 148.5M 输入时序不过：换 **IP Core
   Generator 的 IDDR IP**，不要回裸原语。
3. **(* keep *) 对寄存器无效**：顶层 por_vid/por_usb、act_q 等仍标注 keep，
   但教育版会照常优化；复位计数器靠比较器自身结构保活，目前够用，别依赖该属性。
4. **set_clock_groups 疑似后端问题**：P&R 阶段异常（历史卡死疑云之一）。
   跨时钟豁免一律写两条
   `set_false_path -from [get_clocks A] -to [get_clocks B]` + 反向一条。
5. **SDC 端口名必须与 RTL 一致**：Verilog 标识符不能数字开头，网表网络
   `68051_*` 已改名小写 `it68051_*`（pcscl/pcsda/sysrstn/hpd/int）。
   约束里写 `68051_PCSCL` 会静默落空。
6. **create_clock 挂端口穿不过 DCE**：时钟挂 RX_PCLK/FT601_CLK 端口时，内部
   时序实际未被检查（TA1132 警告会把缓冲后网名报出来）。**正式约束挂内部网名
   `clk_vid_gowin` / `clk_usb_gowin`**（get_nets 写法，见 constr/timing.sdc）。
7. **P&R 配置必须勾选 Dual-Purpose Pin**：SSPI / I2C / CPU 三组 →
   "as regular IO"。每次新建工程都要重勾（工程属性不随源码走）。
8. 综合开关：Use IO Register (syn_useioff)=1（顶层与 iddr_bus_in 已带属性，
   全局开关也建议打开）。

## 4. 当前状态与验收标准

**异步 FIFO mem 写修复后的首次综合尚未运行。** 本仓库修复清单见 §8。下一人
接手第一件事就是跑综合/P&R/TA，并按下面的标准验收：

- CV0016 的 60 条 QD "unused" 警告**消失**（允许 hs_s/RX_HSYNC 一条良性 unused）。
- BSRAM ≈ **12 块**：8 块 FIFO（2048×64）+ 1 块初始化表 ROM + 3 块 EDID
  （256B EDID ROM + 528 字脚本 ROM，教育版可能给 EDID ROM 分 1~2 块）。
- LUT 量级 3~5K（视频通路重新出现后的正常规模）。
- I-O Register 用量**跳增**（60 位 DDR 输入寄存器落 IOLOGIC；修复前报告为
  43 个 "I/O Register as FF" 且无 IOLOGIC 输入寄存器）。
- 时序报告**首次出现 148.5MHz 路径检查**（clk_vid 域 setup/hold 有数字可看）。

## 5. 未决问题

1. **148.5MHz 路径时序验证**：首跑 TA 看 WNS。输入 delay 约束裕量本来就
   ~1ns，若不过先查 IOLOGIC 是否落地，再议 PLL/相移。
2. **IDDR 推断是否落 IOLOGIC**：看 "I/O Register as FF" 是否从 43 跳增、
   Device 视图 IOLOGIC 时钟来源；若落 fabric → 换 IDDR IP（不要回裸原语）。
3. **timing.sdc（完整版）未测试**：net 级 `get_nets` 写法是按 TA1132 报出的
   网名推断的，教育版 SDC 解析器若拒绝 get_nets → 记录报错原文到本节，
   constr/timing.sdc 退回端口级 create_clock（并标注 TA1132 限制），
   同时用 timing_debug.sdc 保底出图。
4. **SDC-full 历史卡死疑云**：此前 P&R 卡死与 set_clock_groups + 洪水剪光
   叠加有关，洪水根因已修，**重测后再定性**。
5. **PC 枚举 2560×1440@144 验证**：EDID 生效判据（显示设置/xrandr 出首选
   144Hz 档；制造商 ZFW、名称 ZFW-CAP-2K）。
6. **0xA9 回读 ACK 实测**：EDID RAM 可读性无 EVB 源码证据，NACK 按软跳过
   （rb_jmp 直达脚本 END）；能 ACK 时回读不一致计入 status[11:8]。

## 6. 构建步骤（Gowin 教育版 V1.9.11）

1. New Project → Device **GW5A-LV25UG324C2/I1**（Education 版系列选 GW5A-25）。
2. 加入 `rtl/` 全部 11 个 .v（Top = `it68051_capture_top`）。**不要**加
   `init/it68051_init_table.txt`（纯文档，ROM 已内嵌）。
3. 约束：加 `constr/capture.cst` + `constr/timing.sdc`。
   - timing.sdc = **完整时序版**（全部 input/output delay + net 级
     create_clock + 跨时钟两条 false_path）。文件头注明"mem写修复后首次使用"。
   - timing_debug.sdc = **最小回退版**（端口级 create_clock + 跨时钟
     false_path + 仅慢速引脚 false_path，**无任何 delay**）。仅当完整版解析
     报错或 P&R 卡死时在工程里换成它出图，报错原文记入 §5-3。
   - 切换方法：Project → Configuration → Placement & Route / Timing constraints
     里把 SDC 路径换成对应文件后重新 Place & Route。
4. **Project → Configuration → Place & Route → Dual-Purpose Pin**：
   SSPI / I2C / CPU 三项全部勾 **"as regular IO"**（否则八球复用脚被吃掉，
   60 路视频输入缺 8 位）。
5. 综合 → P&R → Timing Analysis（TA）。综合选项确认 Use IO Register=1。
6. 查：综合日志 CV0016 计数、资源报告 BSRAM/LUT/IO Register、TA 的 WNS
   （重点 clk_vid 域与 FT601 negedge 发射的 output min-delay）。
7. 生成 .fs，JTAG 下载（H2 排针）。

## 7. 文件清单与模块职责

```
hdmi_capture/
├── AGENTS.md                       本文件（交接文档，接手先读）
├── rtl/
│   ├── it68051_capture_top.v      顶层：DCE时钟缓冲、POR复位、CDC、三态总线、I2C垫层
│   ├── iddr_bus_in.v              60位DDR视频输入（RTL推断IOLOGIC；文件头有禁止回退裸IDDR的完整记录）
│   ├── video_stream.v             裁剪窗口+行/帧计数+行原子包组装+OVERRUN策略
│   ├── rgb2yuv422.v               RGB888→YUV422(UYVY)，BT.601/709可配，3拍流水×2实例
│   ├── async_fifo_dc.v            2048×64双时钟FIFO，推断BSRAM+格雷码指针（已补mem写）
│   ├── ft601_master.v             FT601 245同步FIFO写主机：64b→2×32b、negedge IOB发射
│   ├── ft601_cmd_rx.v             主机→设备带内命令接收（EN_CMD_PATH 门控）
│   ├── i2c_master_od.v            开漏I2C主机位引擎，200kHz，容忍时钟拉伸
│   ├── init_ctrl.v                上电FSM：复位→表回放→EDID阶段→流稳定→HPD→监督重初始化
│   ├── it68051_init_rom.v         191字初始化表ROM（自动生成，勿手改）
│   └── edid_init.v                256B EDID ROM + 528字写入脚本ROM（自动生成，勿手改）
├── constr/
│   ├── capture.cst                114端口引脚约束（LVCMOS33，与RTL端口1:1）
│   ├── timing.sdc                 完整时序版（net级create_clock + 全部delay，mem写修复后首用）
│   └── timing_debug.sdc           最小回退版（端口级create_clock，无delay，仅P&R卡死时用）
├── sw/
│   └── linux_rx.py                主机接收端：libusb异步URB + pyusb回退；包协议权威解析实现
├── docs/
│   └── README.md                  架构/带宽/延迟账本/EDID推导/首次点屏检查项（历史文档）
├── init/
│   └── it68051_init_table.txt     EVB初始化表文本（124条+固定后序，ROM的生成源）
└── edid/
    ├── edid_2k144.bin             自制2560×1440@144 EDID（verify_edid.py 校验通过）
    ├── edid_2k144_annotated.txt   逐字节注释+时序推导
    └── verify_edid.py             独立EDID校验器（python -I verify_edid.py）
```

## 8. 本次重建的修复清单（相对污染前基线）

1. **async_fifo_dc.v**：补上缺失的 `mem` 写端口（根因修复#1）。原文件
   `wr_data` 零引用 → 整条视频通路被反向剪光（CV0016×60、BSRAM只剩ROM）。
   现于写指针提交逻辑后新增：
   `always @(posedge wr_clk) if (wr_commit) mem[wbin[AW-1:0]] <= wr_data;`
2. **init_ctrl.v — tms 多驱动**（EX1999/EX2000 实锤）：节拍 always 块删掉对
   tms 的两处驱动（只留 tcnt 计数与 tick_ms）；FSM 块内 iss 发包逻辑之后、
   `case (main_st)` 之前插入默认自增 `if (tick_ms) tms <= tms + 8'd1;`
   （同块内后写覆盖，状态转换清零语义不变）。
3. **video_stream.v — 包头 w64[0] 位错**：原 `{16'h0000, line_idx, frame_idx}`
   只有 58 位且 line_idx 落在 [41:32]。已按 linux_rx.py 解析对齐为
   `{16'h0000, 6'h0000, line_idx, frame_idx}`（line_idx 在 [47:32]、16位补零，
   frame_idx 在 [31:0]）。w64[1] 核对本来正确，只补了注释。
3b. **video_stream.v — STATUS 包缺 w64[0] 头字**（本次通读新发现的真错误）：
   原 in_status 只写 1 个头字（flags 字）+161 个零字。主机对**每个**包解析
   16B 头（w0..w3）：flags 字落到 w0、w2 读到 0 → type=LINE → 每帧把一行
   载荷写成全零并制造 line_idx gap。已改为完整包头对：stat_cnt==0 写
   `{16'h0000, 16'hFFFF, frame_idx}`（line_idx=0xFFFF 标记），==1 写 flags
   字 8'h21，2..161 零载荷——与模块头注释"header pair + 160 zero pairs"
   及 linux_rx.py 的 TYPE_STATUS 分支对齐。
4. **init_ctrl.v — EDRD 回读多写一字节**（本次通读新发现的真错误，EN_EDID_RB=1
   时必中）：OP_EDRD 的步进原本走 T_START→T_AW→T_BREG→**T_BVAL**→T_BSTOP，
   而 issuer 在 T_BVAL 对 EDRD 发的是 opaddr[11:8]（索引高4位）——即向 EDID
   SRAM 当前指针**多写一个杂散字节**：既破坏刚写好的 EDID，又把从机指针推到
   idx+1，回读比较全错 → ack_cnt 涨满 8 → 永远 M_ERR，HPD 永不拉低。
   修复：OP_EDRD 步进加 `T_BREG: step <= T_BSTOP;`（指针设置事务 =
   START/0xA8/index/STOP，无数据字节）。EDWR（真写数据）不受影响。
5. **iddr_bus_in.v — 裸 IDDR 原语改 RTL 推断**：显式 IDDR 被教育版综合器静默
   丢弃（CV0016×60 的另一半根因）。现用 posedge/negedge 寄存器对推断 IOLOGIC
   DDR 输入（`(* syn_useioff=1 *)`）；DDR 捕获寄存器**故意不带复位**（负沿
   寄存器带复位会阻止 IOLOGIC 落位）；文件头含"禁止回退裸原语"的完整记录
   与 IOLOGIC 落位判据、IDDR IP 替代方案。
6. **死代码清理**（逐项核实为只写不读/悬空后删除，行为不变）：
   - video_stream：`hs_d1`（hs_s 无边沿逻辑，RX_HSYNC 因此被判 unused；
     端口保留作 bring-up 探针）、`li_q`、`hdr_cnt`、`fi_q`。
   - video_stream 端口：`frame_tick`、`htot_pclk`、`de_pclk` 输出删除；
   - 顶层对应悬空线 `frame_tick_v`/`htot_pclk_v`/`de_pclk_v` 删除。
   - 保留的良性项：video_stream 的 `wr_full` 输入按设计不用（永不反压），
     顶层 `unused_int/siwu/wakeup` 垫_regs（保 .cst 引脚有效）。
7. **constr/timing.sdc 重写为完整版**：net 级 create_clock（clk_vid_gowin /
   clk_usb_gowin，来自 TA1132 实报网名）、跨时钟两条 set_false_path 替代
   set_clock_groups、全部 input/output delay 保留、慢速引脚端口名改为
   it68051_*；**constr/timing_debug.sdc 新增**为已知能跑通 P&R 的最小回退版。

## 9. 注意事项（烧板/调试前必读）

- **烧板前确认 Dual-Purpose 勾选**（SSPI/I2C/CPU → as regular IO），每次新建
  工程都要重新勾。
- boot 不受 SSPI 勾选影响：MSPI 自启动走的是 MSPI 专用引脚（UG714），勾
  "as regular IO" 只影响复用脚的普通 IO 功能，自启动照常。
- 本设计**不用音频**：IT68051 的 MCLK/WS/SCK/I2S 悬空是正常的，不要接。
- 若误焊 R8（PCADR 上拉），I2C 地址变 0x92 → 全表 NACK；先用逻辑分析仪在
  K1/L1/L2 确认 ACK 的是 0x90。
- IDDR 极性（word_r=上升沿字/word_f=下降沿字）是按 UG304 4.2.1 推定的；
  上电若见"半行错位/蓝绿互换"，把 iddr_bus_in.v 两个 always 块对调。
- FT601 输出 hold（T4=4.8ns）由 negedge 发射结构保证；若 TA 报 output
  min-delay 违例，先看 IOB tCO ≤ 4ns 是否满足（docs/README.md §5、§7-1）。
- EDID 阶段顺序 = EVB 惯例：表回放 → EDID 上载 → 等稳定 → HPD。源端在
  HPD 低→高之间必须读到新 EDID，重初始化流程保证了这一点，勿改顺序。
- 实验残留（yuv旁路、FIFO旁路、旧备份 .bak）只存在于已删除的旧目录
  `Documents\fpga_fw\`，本目录为干净重建版，**不要**从任何备份/回收站把它们
  带回来。
