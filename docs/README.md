# USB HDMI 采集卡 FPGA 固件（GW5A-LV25UG324）

版本：v1.0（对应冻结规格 / V1.4 网表 / ITE68051_MEGAWIN_EVB_1.57.20200928）

```
fpga_fw/
├── rtl/                          可综合 Verilog-2001+（同步复位、显式 IDDR 原语）
│   ├── it68051_capture_top.v     顶层（端口名 = V1.4 网表 U1 网络名）
│   ├── iddr_bus_in.v             60×IDDR + DE/HS/VS 单沿采样
│   ├── video_stream.v            裁剪窗口 + 行/帧计数 + 64b行包组装 + OVERRUN 策略
│   ├── rgb2yuv422.v              RGB888→YUV422 (UYVY)，BT.601/709 可配，2~3 拍流水
│   ├── async_fifo_dc.v           跨时钟域行 FIFO（64位，推断 BSRAM + 格雷码指针）
│   ├── ft601_master.v            FT601 245 同步 FIFO 写主机（64b→2×32b，400MB/s）
│   ├── ft601_cmd_rx.v            （可选）主机→设备带内命令接收（OUT FIFO）
│   ├── i2c_master_od.v           开漏 I2C 主机位引擎（200kHz，RMW 支持）
│   ├── init_ctrl.v               上电时序 + 初始化表回放 + EDID 上载 + 流稳定监测 + HPD
│   ├── it68051_init_rom.v        初始化表 ROM（脚本自动生成，勿手改）
│   └── edid_init.v               自制 EDID 256B ROM + IT68051 写入脚本 ROM（自动生成，勿手改）
├── constr/
│   ├── capture.cst               引脚约束（114 端口，球脚名，LVCMOS33）
│   └── timing.sdc                时序约束
├── sw/linux_rx.py                libusb-1.0 异步接收端（协议解析 + 延迟统计）
├── init/it68051_init_table.txt   从 EVB 固件源码提取的 I2C 初始化表（文本）
├── edid/
│   ├── edid_2k144.bin            自制 EDID（128B EDID 1.4 基块 + 128B CTA-861，共 256B）
│   ├── edid_2k144_annotated.txt  逐字节注释 + 时序计算过程 + EVB 写入机制
│   └── verify_edid.py            独立校验器（python -I verify_edid.py）
└── docs/README.md                本文件
```

---

## 1. 架构

```
                     IT68051 (U5)                          FPGA GW5A-25 (U1)                        FT601Q (U2)      USB3 主机
        ┌────────────────────────────┐        ┌──────────────────────────────────────────┐      ┌──────────┐
HDMI →  │ TMDS Rx → HDCP → CSC旁路   │  QD    │ 60×IDDR ─→ video_stream ─→ async FIFO ─→ │ 245  │ 16KB TX  │ Bulk IN
2K144   │ 双像素DDR输出              │ [59:0] │ (M10)    裁剪640×640      2048×32   ft601 │同步  │ 4KB×2    │ MPS1024
        │ PCLK=148.5MHz (M10→FPGA)   │        │ RGB→YUV422(BT.601/709)    BSRAM格雷  主机 │FIFO  │ FIFO     │ burst15
        │ DE/HS/VS 单沿 (P1/P2/M8)   │        │ 行包头16B+载荷1280B       格雷码CDC      │ 100M │          │
        └──────────┬─────────────────┘        │            ↑裁剪参数/使能(2FF CDC)       │ CLK→ │ ┌──────┐ │
                   │ I2C 0x90/0x91            │ init_ctrl (100M域) ← 命令 ← ft601_cmd_rx─┘ H4   │ │ EP0  │ │←枚举
        ┌──────────┴──────────┐               │  SYSRSTN(K2) INT(K1)                    │      │ └──────┘ │
        │ FPGA I2C主机(RMW回放)│←──────────────┤  HPD(K1→B47→Q3)                         │      └──────────┘
        └─────────────────────┘               └──────────────────────────────────────────┘
```

数据流（视频域 148.5MHz，每 PCLK 周期 2 个双像素字 = 4 像素）：

1. `iddr_bus_in`：60 位 QD 用 **IDDR 原语**捕获——Q0=PCLK 上升沿字（偶序双像素字），
   Q1=下降沿字；DE/HS/VS 单沿、同沿采样（IT68051 保证 setup 2.88ns/hold 1.88ns）。
2. `video_stream`：字序号 r（DE 内上升沿计数），falling 字基准 4r−2、rising 字基准 4r；
   裁剪窗口 [x0, x0+640)（x0 强制 %4==0），行窗口 [y0, y0+640)。
3. `rgb2yuv422` ×2：每 2 像素字输出 4 字节 UYVY（Cb,Y0,Cr,Y1），色度取像素对平均；
   两路流水线天然对齐，**每 PCLK 周期合成一个 64 位字** {hi=f字, lo=r字}
   （lo=较早像素，FT601 先发低 32 位 → 像素顺序天然正确）。
4. 行包组装：2 个 64 位包头字（=16B：frame_idx/line_idx/flags/status）+ 160 个 64 位
   载荷字 = 162×8B = 1296B/行；FIFO 空闲不足（行 166 字、EOLF 行另加 STATUS 162 字）
   时**整行丢弃**并在下一行包头打 OVERRUN 位（绝不反压视频通路）。
   每帧最后一行（EOLF）后追加一个 STATUS 包（line_idx=0xFFFF）。
5. `async_fifo_dc`：2048×64 位（16KiB ≈ 12.6 行包），格雷码指针双 2FF 同步。
6. `ft601_master`：64 位字拆成两个 32 位总线字（lo 先），TXE_N 低时逐字写入，
   1296B 行 = 324 字连续突发（单总线事务），峰值 1 字/时钟 = 400MB/s。

### 带内控制面（设计决策）

FT601 是纯 FIFO：USB EP0 厂商请求终结在 FT601 内部，**FPGA 永远看不到 EP0**（EP0 仅用于
USB 枚举，由 FT601 固件/驱动承担）。因此协议中的 SET_CROP / GET_STATUS / SOFT_RESET 改为
**同一 bulk 流的带内控制**：

* 设备→主机：STATUS 包（flags[7:5]=type 1，line_idx=0xFFFF），每帧一个，含 16 位设备状态。
* 主机→设备：16 字节命令包写入 Bulk OUT（FT601 OUT FIFO → `ft601_cmd_rx` 读出解析）。
  选带内而不是"独立行号空间"：行号空间保持 0..639 语义干净，用 flags 的 type 域区分
  零成本且解析器无需多路分叉。
* **`EN_CMD_PATH=0`（参数）可整体关闭命令路径**：OE_N/RD_N 恒高、DATA 恒驱动，恢复
  冻结规格"读方向不用"的行为（代价：裁剪位置只能用编译期默认 960/400）。

### 包格式（冻结，与 sw/linux_rx.py 一致）

| 字节   | 字段                                        |
|--------|---------------------------------------------|
| 0..3   | frame_idx (u32 LE)                          |
| 4..5   | line_idx (u16 LE, 0..639；0xFFFF=STATUS)    |
| 6..7   | rsv=0                                       |
| 8      | flags：bit0=EOLF, bit1=OVERRUN, [7:5]=type  |
| 9..11  | rsv=0                                       |
| 12..13 | status（设备状态字，见下）                   |
| 14..15 | rsv=0                                       |
| 16..1295 | 载荷 1280B = 320×UYVY 像素对 (Cb,Y0,Cr,Y1)|

status 位定义（init_ctrl）：[15:12] init_err_cnt，[11:8] i2c_nack_cnt
（EDID 阶段的回读字节不一致也计入此计数），[7] stream_ok，[6] htot_ok，
[5] vid_stable，[4] init_done（含 EDID 上载完成），[3] hpd_active，
[2] online，[1] i2c_fail，[0] replay_active（初始化表或 EDID 阶段进行中）。

主机命令包：`'BFCP' | cmd(1) | len=8 | x0(u32) | y0(u32) | rsv(2)`；
cmd：1=SET_CROP，2=SOFT_RESET，3=GET_STATUS（被动，等下一帧 STATUS 即可）。

---

## 2. 带宽与延迟计算

### 带宽

* 源：2560×1440@144 CVT-RB（冻结：PCLK=148.5MHz，双像素 DDR，HTotal%4==0，如 2720）。
* 采集载荷：640 行 × 1296B = 829,440 B/帧；144fps → **119.4 MB/s**（含 16B/行包头）。
* FT601 写通路峰值：100MHz × 4B = **400MB/s**（数据手册"bursting rate up to 400MB/s"，
  本设计用下降沿发射 IOB 保证 T4=4.8ns hold，见 §5 时序）；采集块（640 行连续）期间
  平均产生 1296B / 4.58µs ≈ 283MB/s < 400MB/s，**无持续积压**，2~4 行弹性 FIFO 够用
  （实际配了 ~12.6 行包余量：2048×64b）。
* USB3 SuperSpeed Bulk：MPS=1024，bMaxBurst=15 → 理论 ~3.8GB/s；实测 FT601 平台普遍
  ≥ 300MB/s ≫ 119.4MB/s，**全帧率无丢帧**。每行 1296B 作为一个连续事务写入，
  FT601 侧自然形成 1024B 全包 + 272B 短包（短包兼作行边界提示，利于主机及时拿到行）。

### 延迟账本（源像素 → 主机收到该行，逐级）

| # | 环节 | 延迟 | 说明 |
|---|------|------|------|
| 1 | IT68051 内部（TMDS→TTL） | **~1 行（4.6µs）量级，未验证** | 取决于去抖/CDR/去交织；Programming Guide 未提供精确值，需实测 |
| 2 | IDDR + 解交织 + 裁剪 + YUV + 组包 | **≈ 0.06µs**（9 个 PCLK） | IDDR 0 + 字延迟 4/5 + YUV 流水 3 + 64b 配对，包头先行 2 拍 |
| 3 | 行 FIFO 攒行 | **0**（设计目标行原子推流） | 包头写入即刻可读；FIFO 只吸收突发（≤ 1 行 1.3KB） |
| 4 | CDC | ≤ 0.2µs | 格雷码 2FF，2~3 个 100MHz 周期 |
| 5 | FT601 排空 1296B | 3.24µs（324 字 × 10ns） | 与下一行产生（4.58µs）重叠，行尾最后 ~1µs 等待 |
| 6 | FT601 TX FIFO→USB 发包 | 0~34µs | 4KB 双缓冲：行事务结束触发短包提交；最坏等上一个 4KB 缓冲被 USB 排干（119MB/s 下 34µs） |
| 7 | USB 线缆 + 主机栈 | 10~100µs | 集线器/调度相关，超出固件控制 |
| 8 | 主机 URB→解析回调 | ≤ 1 个 URB 数据时间（~170µs@16 行/URB，可用 --urb-lines 调小） | linux_rx.py 停留时间为 0（多 URB 排队） |

**合计：典型 ~20-50µs 量级（设备侧 ≤ 8µs）。** 主导项是 IT68051 内部（#1，待实测）与
主机栈（#7）。若要压更低：`--urb-lines 4`（URB 粒度 4 行 ≈ 42µs@119MB/s）。

---

## 3. Gowin 软件手动步骤（首次建工程）

1. **建工程**：Gowin V1.9.8+ → New Project → Device: **GW5A-25**，封装 **UG324**，速度等级
   与实际芯片一致（C 版 GW5A-LV25UG324ES/C1 视芯片丝印）；Part Number 形如
   `GW5A-LV25UG324C1/I0`。
2. **加入设计**：`rtl/*.v` 全部加入（`it68051_capture_top.v` 设为 Top）。**不要**加入
   `init/it68051_init_table.txt`（仅是文档，ROM 已内嵌）。
3. **约束**：`Project → Configuration → FloorPlanner/Constraint` 加入 `constr/capture.cst`
   与 `constr/timing.sdc`（SDC 也可以放在 Implementation 的 "SDC File" 一栏）。
4. **IO 标准**：cst 中已逐脚 `IO_TYPE=LVCMOS33`。核对 Bank 供电：bank2/3/4/5/6/7 全部
   3.3V（VCCIO 由 FPGA_3V3/IOxx_3V3 网络），**bank0/1（1.5V DDR3 域）一个脚都没用**。
5. **综合选项（关键）**：Project Configuration → Synthesize → **Use IO Register
   (syn_useioff) = 1**（或在 User Rules 加 `-use_ioff true`）。顶层与 iddr_bus_in 已带
   `(* syn_useioff = 1 *)` 属性，但全局开关保证 FT601 输出与视频输入的寄存器落在 IOLOGIC。
6. **IDDR/PLL 设置**：v1 **不用 PLL**——RX_PCLK(M10=GCLKT_9B) 与 FT601_CLK(H4=GCLKT_15)
   都是直通全局时钟球，经 DCE 上全局网络（rtl 内已实例化，UG306 §3.1，CE 高电平有效接 1'b1）。IDDR 为显式原语，无属性需要设置。
   若实测捕获裕量不足（见 §7 检查项 2），再加 PLL（Gowin IP：Clock → PLL，输入 148.5M，
   输出 0° 148.5M；如需采样相移再讨论 90° 方案）。
7. **Place & Route**：Seed 默认；若时序不过优先检查 FT601 的 output min-delay（见 §7）。
8. **比特流**：生成 .fs；JTAG（A94/A96/A98/A100 为专用脚，板上 H2 排针）下载。

### 首次编译人工检查项（无法在本环境验证，务必逐条确认）

1. **FT601 输出 hold（T4=4.8ns）**：Timing Report → "output delay min" 路径
   （FT601_DATA_*/BE_*/WR_N/OE_N/RD_N，negedge 发射）。要求 setup（-max 1.0）为正即
   **tCO ≤ 4.0ns**；hold（-min 4.8）由半周期结构保证必过。若 tCO > 4ns：把
   `ft601_master.v`/`ft601_cmd_rx.v` 的输出寄存器从 IOB 挪回 fabric（去掉对应
   `syn_useioff` 影响），或加强驱动/降斜率后再查。
2. **IDDR 极性**：Q0=上升沿字、Q1=下降沿字是按 UG304 4.2.1 框图推定的。上电先用测试
   图形（如灰阶渐变）核对：若出现"半行错位/蓝绿互换"，把 `iddr_bus_in.v` 的 Q0/Q1 对调。
3. **IDDR 时钟**：确认 IDDR 的 CLK 已接到 RX_PCLK 的 DCE 输出且 IOLOGIC 时钟路由成功
   （FPGA Editor / Device 窗口查看 IOLOGIC 时钟来源）。
4. **BSRAM 推断**：综合日志确认 `async_fifo_dc` 的 mem 与 init ROM 被推断为 BSRAM/EBR
   （FIFO 8 块 / ROM 1 块；总 BSRAM 应 < 10 块，预算 30）。
5. **I2C 实测**：逻辑分析仪挂 K1/L1/L2：SCL=200kHz、地址 0x90 ACK。若 NACK（地址错），
   复核 PCADR(U5.23) 电平：R8(0Ω 上拉到 OVDD33) 为 **DNP 不焊**（用户已确认），R6(10k)
   下拉到 GND → 地址 0x90。若误焊 R8 则地址变 0x92。
6. **HPD 极性实测**：初始化完成后 B47 应为低电平、HDMI 源收到的 HPD（CN1.19）为高。
7. **图像方向**：若整图上下/左右镜像，调 y0/x0 或交换 UYVY 的 Cb/Cr（`rgb2yuv422.v`
   打包字）。
8. **色深检查**：IT68051 输出 10b（b2..b11），FPGA 取高 8 位（[11:4]→[9:2]）。若源输出
   12bpc（GCP 改变），IT68051 输出位宽仍为 30bit 总线、映射不变，但需确认 dither 后的
   低 2 位损失可接受。
9. **PC 是否枚举出 2560×1440@144（EDID 生效判据）**：接 GPU 后在显示设置/EDID 读出
   中确认首选模式为 2560×1440@144（制造商 "ZFW"、名称 "ZFW-CAP-2K"）。若没有 144 档：
   用 `edid-decode /sys/class/drm/card*/ HDMI-A-*/edid` 比对 `edid/edid_2k144.bin`
   （应逐字节一致）；不一致先查 I2C 0xA8 写事务与 bank0 0x4B=0xA9 使能（见 §4.5）。
10. **EDID 写入与回读**：逻辑分析仪确认 0xA8 的 254 个单字节写事务发生在初始化表
   之后、0xC5[4] 复位脉冲之前；0xA9 回读若全部 NACK 属"软跳过"（EN_EDID_RB 预期行为，
   见 §4.5），若回读值与 ROM 不一致则 status[11:8] 计数增长并可触发重初始化。

---

## 4. IT68051 初始化表（来源与处理）

**来源（权威）**：`ITE68051_MEGAWIN_EVB_1.57.20200928/src/iTE6805_INI_Table.h`
（v1.57, 2020-09-28）中 `iTE6805_INIT_HDMI_TABLE` 的**非注释行**，宏按 `config.h` 解析：
`eVidOutConfig=eTTL_SepSync_FullBusDDR_BYPASS_CSC`、`Reg_IOMode=0x40`、
`EQ_INIT_VALUE=0x9F`、`Clock_Delay=3`、`DownScale_Filter=0x01`、`ADDR_MHL|1=0xE1`、
`ADDR_CEC=0xC8`；`iTE68052`/`iTE6807`/`LVDSSSC` 块按条件编译剔除。

**生成**：`review/scripts/ia_extract_it68051_init.py`（python -I）→
`init/it68051_init_table.txt`（124 条表项 + 固定后序列）与 `rtl/it68051_init_rom.v`
（191 个操作字）。操作语义 = 固件 `hdmirxset()`（**读改写** reg=(reg&~mask)|val）与
`hdmirxwr()`（直写），与源码逐条对应。

表文件分节：
* SECTION A：EVB `INIT_HDMI_TABLE` 原文（124 条，含 bank 切换 `{0x0F,0x07,xx}`）。
* SECTION B：`iTE6805_Init_CAOF()`（AFE/PLL 上电、CAOF 触发与轮询，DELAY/POLL 化）。
* SECTION B2/B3：`Init_fsm` 中段（0x28=0x88、Video_Tristate ON：bank1 0xC5=0x38 对应
  单通道全总线 DDR 10bit）与 `OCLK_Cal` 的**静态近似**（0x91/0x92/3.0xFA）。
* SECTION D：`iTE6805_Set_TTL_Video_Path()`——bank1 0xC0[1]=1（**全总线 DDR**）、
  0xC1[1]=1（DDR）/[5]=0（分离同步）。
* SECTION E：`iTE6805_Port_Select_Body(PORT0)`（选主口 + TMDS 上电序列 + EDID/SCDC 复位脉冲）。
* SECTION F：`iTE6805_Enable_Video_Output()` 的 DVI 分支（0x6B 强制输入 RGB——与本板
  GPU 源一致；若将来接 YUV 源需按 AVI InfoFrame 运行时改写，v1 未做，见"未验证项"）。

**FPGA 回放流程**（init_ctrl）：SYSRSTN 低 10ms → 高 100ms → ROM 回放（WRITE=RMW，
mask=0xFF 时直写；POLL 超时 200ms 后继续）→ 等视频稳定（行活动 + 行周期/DE 宽度偶校验，
连续 100ms）→ HPD 拉低（对源宣告有效）。流丢失 50ms 或主机 SOFT_RESET → 释放 HPD →
完整重初始化。

### I2C 地址：0x90（写）/0x91（读），7 位 0x48 —— 与 EVB 一致

PCADR(U5.23) 在本板上为**低电平**：上拉电阻 R8(0Ω→OVDD33) **DNP 不焊**（用户确认的
装配决策；V1.4 网表中的 R8 只是占位焊盘），实际仅 R6(10k) 把 PCADR 下拉到 GND。
数据手册 Table 4-3：PCADR 低 → 设备地址 **0x90**（8 位写）/0x91（读），7 位 0x48，
与 EVB `config.h` 的 `ADDR_HDMI=0x90` 相同。`i2c_master_od`/`init_ctrl` 按 0x90/0x91
实现。注意：若板上误焊 R8，地址会变为 0x92/0x93——排查 NACK 时先确认 R8 未焊。

### 自制 EDID（v2，已实现）：edid/edid_2k144.bin + rtl/edid_init.v

**为什么必须自制**：EVB `EDID_SELECT_TABLE=8` 的 EDID 的 TMDS 上限是 300MHz 档
（HDMI VSDB Max_TMDS_Clock=0x3C，源码注释原话"0x3C for support 300MHz TMDS"），
且无 HDMI Forum VSDB → 源端永远不会发起 >340Mcsc 的 HDMI 2.0 信令，2K144 无从谈起。
本 EDID 为**全新内容**（不复用 EVB 表任何字节），由
`review/scripts/edid_gen_2k144.py` 生成（幂等可重跑），字节布局以
Linux `include/drm/drm_edid.h` / `drivers/gpu/drm/drm_edid.c` 与 VESA E-EDID 1.4 /
CTA-861 表（Wikipedia EDID 条目）交叉验证；独立校验器 `edid/verify_edid.py`
（解码侧第二实现，双通道互验）输出 `ALL CHECKS PASSED`。

#### 主时序：2560×1440@144（CVT-RB 几何 + 压缩 VBI）

| 量 | 值 | 推导 |
|----|----|------|
| HTotal | **2720** | CVT-RB 固定 HBlank=160（HFP48/HSW32/HBP80）；2560+160；**2720%4==0** ✓（IT68051 双像素 DDR 硬性要求） |
| VTotal | **1481** | VFP=3、VSYNC=4（CVT-RB 固定）+ VBP=34 → VBlank=41 |
| Pixel clock | **580.000 MHz** | 精确 144Hz 需 2720×1481×144=580.078MHz，取 CVT 0.25MHz 栅格 → 580.00；实际刷新 **143.981 Hz** |
| hsync 频率 | 213.235 kHz | 580e6/2720 |
| TMDS 字符率 | 580 Mcsc ≤ 594 | HDMI 2.0 档（>340 自动加扰） |
| IT68051 TTL PCLK | **145.0 MHz** | 双像素 DDR：580/4 ≤ 150 ✓ |

官方 VESA CVT-RB 的最小 VBI 是 460µs——144Hz 下会推出 VTotal=1621 → **635MHz > 594**，
超出 HDMI 2.0。因此 VBI 压缩到 41 行（HDMI 2.0 144Hz _sink 的通行做法），
其余 CVT-RB 几何不变。这是**有记录的偏离**，不是笔误（edid_2k144_annotated.txt §1）。

辅助时序：DTD2=1920×1080@60（VIC16 正式时序 148.5MHz/2200/1125）、标准时序
1920×1080@60 与 1440×900@60、established 800×600@60 + 640×480@60（HTotal 均 %4==0，
刻意**不**广告 720p60：CEA 1650%4≠0 会导致"有流但 FPGA 不拉 HPD"）；
CTA 扩展 DTD 槽放 1440×900@60（DMT 0x2F，106.5MHz/1904）。

#### CTA-861 扩展（字节级布局以内核源码为准）

`02 03 18 30 | 41 90 | 68 03 0C 00 10 00 00 44 20 | 67 D8 5D C4 01 78 80 00 | 00 | DTD | 00填充 | 校验和`
* byte3=0x30：YCbCr444(bit5)+YCbCr422(bit4) 能力位（RGB444 由扩展存在隐含；
  bit6 基础音频=0——本机只采视频）；DTD 起始偏移 24（4 对齐，且 > HDMI VSDB 偏移 6，
  保证 EVB 解析器 `iTE6805_EDID_Find_Phyaddress` 能走到 VSDB）。
* VDB：SVD 0x90 = VIC16(1080p60)+native。
* **HDMI VSDB**（len 8）：OUI 000C03、物理地址 **1.0.0.0**（PORT0 根设备，与 EVB
  `rxphyadr[PORT0]={0x10,0x00}` 一致）、Max_TMDS_Clock=**0x44=340MHz**（字节上限档）、
  byte7 bit5 HDMI_Video_Present=1。不声明深色（8bpc sink）。
* **HF-VSDB（HDMI Forum VSDB，紧跟 HDMI VSDB 之后——HDMI 2.0 排序要求）**：
  OUI C45DD8、version 1、**Max_TMDS_Character_Rate=0x78=600MHz**、
  **byte6=0x80 → SCDC_Present=1**（RR_Capable=0 不轮询、Max_FRL=0 纯 HDMI 2.0）。
  内核 `drm_parse_hdmi_forum_scds()` 的注释即判定依据："All HDMI 2.0 monitors must
  support scrambling at rates > 340 MHz...（HF-VSDB 存在 + Max_TMDS_Char_Rate>340 +
  SCDC 支持三者齐备）"——没有这一块源端不会上 594Mcsc 档信号。
* Range Limits（基块 0x6C）：v 48–150Hz、h 30–220kHz、**max pixel clock 600MHz**、
  flags=0x01 "range limits only"（范围内任意时序皆可接受，防止驱动按 GTF/CVT 公式
  校验拒掉 580MHz 的 DTD）。

#### IT68051 EDID SRAM 写入机制（依据 EVB 源码，仅学机制、不取内容）

* **容量**：每端口 ≥256B——`Customer_EDID[EDID_COUNT][256]`
  （iTE6805_EDID_Table.h:38）+ `iTE6805_EDID_UpdateRAM()` 写 offset 0..126 与
  128..254（iTE6805_EDID.c:133-154）→ 256B 方案成立（128B 精简方案不需要）。
* **访问通道**：EDID SRAM 是主寄存器同一条 I2C 总线上的**第二从机**
  `ADDR_EDID=0xA8`（写）/0xA9（读）（config.h:371）；使用前须 bank0 寄存器
  `0x4B=0xA9`（[7:1] 从机地址、[0] 使能，iTE6805_EDID.c:33）。主寄存器仍走
  0x90/0x91（本板 PCADR 低，见上节）。
* **EVB 写入顺序**（= 我们脚本的顺序）：HPD 拉低（EDID.c:74-79）→ 0x4B 使能（:33）→
  按 127B/块逐字节写 SRAM（UpdateRAM :82-89，**偏移 127/255 不写**——块校验和由
  硬件从寄存器取）→ bank0/4 的 0xC9 写 block0 校验和（:83-86）→ 0xC6=VSDB 物理地址
  偏移、0xC7/0xC8=各端口 CEC 物理地址覆写、0xCA=各端口 block1 校验和
  （bank0=PORT0、bank4=PORT1，:115-124；端口 1 的校验和按 EVB DRV.c:111-112 做
  `+0x10+0x00−0x20−0x00` 修正）→ 0xC5[0]=0 使能内部 EDID（:36-38）→ 每端口
  0xC5[4] 复位脉冲 + 1ms（:41-50）→ chgbank(0)。HPD 由 SYS 状态机在信号稳定后才拉高
  （iTE6805_SYS.c:3034/3148/3321）——**EDID 先于 HPD** 是 EVB 惯例，本板 HPD 由 FPGA
  引脚经 Q3 控制，M_ONLINE 之前恒为高阻（源看到 HPD 低），天然满足该时序。
* 本板写入值：0xC9=0xE5、0xC6=0x89、bank0 0xCA=0xBC、bank4 0xCA=0xAC（生成器输出，
  与 bin 校验和一致；PORT0 时 EDID 内容与寄存器覆写值相同，源读到的即写入的）。

#### FPGA 集成（init_ctrl.v 补丁 + edid_init.v）

* `edid_init.v`（自动生成）：256B EDID ROM + 528 字操作脚本 ROM。操作字编码与
  `it68051_init_rom.v` 相同（[31:28] op、[27:16] {bank,reg}、[15:8] mask、[7:0] val），
  新增 op5=EDWR（写 EDID SRAM 一字节：START/0xA8/索引/数据/STOP）与
  op6=EDRD（写指针+从 0xA9 读回比较）。
* `init_ctrl.v`：初始化表 OP_END 后进入 **EDID 阶段**（复用 M_RUN 字节步进机，
  opword 源切换为脚本 ROM）→ 脚本 END → M_TBLDONE（init_done 置位）→ 等视频稳定 →
  HPD。**先后依据**：EVB `iTE6805_Init_fsm()` 先回放 `iTE6805_INIT_HDMI_TABLE`
  （iTE6805_DRV.c:2528）后调 `iTE6805_EDID_Init()`（:2562）——表先行、EDID 随后。
  流丢失/主机 SOFT_RESET → 完整重初始化**含 EDID 重写**（源在 HPD 低→高之间必须看到
  新 EDID）。参数：`EN_EDID=0` 恢复 v1 行为（不写 EDID）；`EN_EDID_RB=0` 跳过回读。
* 回读（EN_EDID_RB=1，默认开）：EDID RAM 可读性**没有 EVB 源码证据**（EVB 从不读回），
  因此 0xA9 地址 NACK 按软处理——直接跳到脚本 END，不算失败；回读不一致计入
  status[11:8]（累计 8 次会像 I2C NACK 一样触发重初始化）。写入阶段 0xA8 NACK 仍算
  硬错误。
* **顺带修复 v1 缺陷**：DELAY 操作此前会误发一个 I2C START 并在 ~10µs 后即完成
  （ms 计时永远输给 i2c_done）→ 现在 issuer 对 OP_DELAY 不发总线事务，真实延时。
  影响：EDID 的 0xC5[4] 1ms 复位脉冲与 CAOF 相关延时首次按 EVB 时序生效（总启动时间
  略变长，不影响正确性）。

#### 首次点屏验证 EDID 生效

1. 逻辑分析仪看 K1/L1/L2：表回放后应看到对 **0xA8** 的 254 个单字节写事务 +
   0xA9 的回读，然后才是视频稳定判据。
2. PC（Windows 显示设置 / Linux `xrandr --prop` / `edid-decode /sys/class/drm/*/edid`）
   应枚举出 **2560×1440@144**（首选），以及 1920×1080@60、1440×900@60、800×600@60、
   640×480@60；制造商 "ZFW"，名称 "ZFW-CAP-2K"。
3. 源设为 2560×1440@144 后：STATUS 包 status[4](init_done)=1、status[3](hpd_active)=1，
   帧率 ≈144fps（`sw/linux_rx.py` 帧号差）。

### 未验证项（无法从文本源确认，需实测/查 Programming Guide）

1. **IT68051 内部管线延迟**（延迟账本 #1）：数据手册未给，EVB 源码只有轮询节奏。
2. **YUV422 直出的 QD 位映射**：数据手册只说"Refer to Video Data Bus Mappings"
   （在 Programming Guide，纯文本不可得）。EVB 默认 `BYPASS_CSC`=RGB 直出，位映射已由
   **V1.4 网表网络名证实**：QD[9:0]=A_B2..11（像素A 蓝 b2-11）、QD[19:10]=A_G*、
   QD[29:20]=A_R*、QD[39:30]=B_B*、QD[49:40]=B_G*、QD[59:50]=B_R*。因此 **v1 不使用
   422 直出**：IT68051 按 EVB 默认输出 RGB，YUV422 由 FPGA 转换（BT.601/709 参数可配，
   2~3 拍流水，不增加帧延迟）。若将来要 422 直出（省 24 个乘法器），必须人工核对
   Programming Guide 的 422 映射寄存器（bank0 0x6B[3:2]=4'b01 + CSC RGB2YUV 表）。
3. **OCLK_Cal 动态测量**：固件用内部振荡计频算 1µs 定时（0x91/0x92）与 SCDC RR 保持
   （bank3 0xFA）；FPGA 用静态近似 0x91=0x14/0x92=0x80/0xFA=0xCD（RCLK≈20.5MHz）。
   只影响音频/CEC/SCDC-RR 时基，**不影响视频**；视频通路若异常可回读实测后改 ROM。
4. **CAOF 轮询重试分支**：固件失败时复位重试（waitcnt/failcnt 循环）；FPGA 版 POLL
   超时 200ms 后**继续**（不重试）。CAOF 失败 = 无视频，靠"流不稳定→不拉 HPD"兜底。
5. **运行时 INT# 处理**：EVB 主循环轮询 INT# 做格式切换/EQ/CSC 矩阵选择/重发 HPD；
   v1 不轮询（INT# 只做状态记录）。数据手册明言"硬件可以输出某种可显示的视频"，
   对 DVI/RGB 源成立；HDMI-YUV 源的 CSC 矩阵选择缺失 → 图像可能偏色（不花屏）。
6. **EDID（v2 已实现，见"自制 EDID"节）**：EVB `EDID_SELECT_TABLE=8` 的 EDID 上限
   300MHz 档的问题已通过自制 `edid/edid_2k144.bin` 解决（HF-VSDB max_TMDS=0x78、
   SCDC_Present=1、2560×1440@144 主 DTD，经 0xA8 从机 + bank0/4 0xC6..0xCA 寄存器
   写入）。残余待实测项：① 0xA9 读回是否 NACK（软处理不影响功能，只影响回读校验
   是否生效）；② IT68051 对 hsync≈213kHz（144Hz 高行频）的 CDR/行频上限——数据手册
   本地文本只有音频频率参数，视频行频范围需人工核对 Programming Guide。
7. **HDCP**：IT68051 内置 HDCP 引擎自动应答 DDC；无源码级开关残留。若实测加解密后
   图像异常，检查 bank0 0xD9/0x2C/0x2D（表内已含 EVB 值）。

---

## 5. FT601 接口时序设计（为什么下降沿发射）

FT601 245 同步 FIFO、100MHz：`T3 主机建立 1ns / T4 主机保持 4.8ns`（Table 4.2）。
T4 的含义：FT601 在 CLK **上升沿**采样 WR_N/DATA/BE，之后旧数据须保持 4.8ns——
若 FPGA 在同一上升沿换数据（IOB tCO 仅 2.5~4ns），必然违例。FTDI 官方参考
（AN_421 FIFO Bus Master）用 `set_output_delay -min 4.8` 让工具保证 tCO≥4.8ns。

本方案的确定性做法：**FSM 决策在上升沿，引脚寄存器用下降沿（IOB）发射**：
* 建立：数据在 negedge(N)+tCO 到达，采样沿 posedge(N+1) 前 5ns−tCO ≥ 1ns → **tCO ≤ 4ns**
  （检查项 1 用 STA 核实）；
* 保持：下一次发射在整整一个周期之后（C+5ns+），恒满足 C+4.8ns；
* 峰值仍为 1 字/时钟 = 400MB/s（FSM 双槽 skid 缓冲吸收 FIFO 的 1 拍读延迟，TXE_N 低时
  无气泡）。

TXE_N/RXF_N/DATA（从机输出）按 T1=3.0/T2=3.5ns 约束，上升沿 IOB 采样（sdc 已写）。
BE[3:0] 为**高有效**字节使能（"the bus master asserts the signal for the valid bytes"），
本设计恒 4'hF（1296B = 324 整字）。

---

## 6. 已知假设清单

1. 源模式 HTotal%4==0 且 HActive%4==0（IT68051 双像素 DDR 的硬性要求；2K144 CVT-RB
   2720 ✓，1080p 2200 ✓，720p CEA 1650 ✗ 不支持——FPGA 会因行周期/DE 宽度检查失败而
   不拉 HPD，表现为"无流"而非花屏）。
2. 裁剪窗 x0、y0 均为偶数（RTL 强制 x0%4==0；SET_CROP 主机侧也做 %4）。
3. FT601 出厂配置：245 同步 FIFO 模式（GPIO0/1 上电采到 00）、CLK=100MHz、32bit、
   IN MPS=1024 / burst 15（用 FT_Prog 核对 EEPROM；板上 GPIO0/1 的上电电平由
   FT601 内部上拉决定，若为多通道模式需改写）。
4. 2K144 像素率：自制 EDID 精确给出 **580MHz**（VTotal 1481，刷新 143.98Hz），
   IT68051 TTL PCLK=145MHz（原冻结估算的 148.5MHz/594M 是上限情形，采集逻辑同样
   覆盖）；其它分辨率下裁剪/采集逻辑自适配，但行周期上限按 FIFO 预算校核过
   4.58µs 最坏情形（580MHz/HTotal2720 = 4.69µs 行周期，FIFO 预算仍成立）。
5. 视频与 USB 两域的复位由各自上电计数器给出；PCLK 缺失时视频域保持复位（无害）。
6. `EN_CMD_PATH=1` 时主机向 Bulk OUT 写命令：若 FPGA 命令 FIFO 未被读走（命令速率
   极低，实际不可能填满 4KB），USB 写会阻塞——这是行为特征不是缺陷。
7. linux_rx.py 的 VID/PID 默认 FTDI 0403:601F，按实际 FT601 描述符调整 `--vid/--pid`。
8. 主机侧延迟统计的 med/p95 是**到达间隔**（流水线抖动指标）；真正的端到端延迟需要
   源侧打时间戳（如源画面嵌秒/帧号用 OCR/帧差）——见 §2 延迟账本。

---

## 7. 自检

`review/scripts/`：
* `ia_extract_it68051_init.py` —— 从 EVB 源码生成初始化表 + ROM（可重跑，幂等）。
* `ia_gen_cst.py` —— 由 CSV 生成 capture.cst，并与 `V1.4.tel` 逐脚交叉核对
  （U1 每个信号网络 vs 端口表、球脚唯一性、bank0/1 禁用、JTAG 排除）。
* `ia_check_fw.py` —— RTL 顶层端口 ↔ CST 端口 1:1 校验、结构 lint
  （module/case/begin 配对、实例化连线声明检查）、ROM ↔ 表条数一致性。
  当前输出：`ports: CST=114 RTL=114 ... ALL CHECKS PASSED`（EDID 补丁后复跑通过，
  端口数不变）。
* `edid_gen_2k144.py` —— 生成 `edid/edid_2k144.bin` + 逐字节注释（幂等，重跑逐位一致）。
* `edid_gen_rtl.py` —— 由 .bin 生成 `rtl/edid_init.v`（EDID ROM + 528 字写入脚本，
  幂等；保证 RTL 与 .bin 同源）。

`fpga_fw/edid/`：
* `verify_edid.py`（python -I）—— **独立第二实现**的校验器（与生成器互验）：双块
  checksum、基块/CTA 结构 walk（established 位图、标准时序、4 个描述符槽、CTA 数据块
  逐个解析）、主时序第一性重算（2560×1440、CVT-RB 几何常数、HTotal%4、0.25MHz 栅格、
  ≤594MHz、TTL PCLK≤150MHz、±hsync 极性）、VIC16 精确比对、Range Limits 覆盖性、
  HDMI VSDB/HF-VSDB 字段（OUI/物理地址/340MHz/600MHz/SCDC_Present/排序）。
  当前输出：`RESULT: ALL CHECKS PASSED`；生成器 hex 打印与校验器对 .bin 的 dump
  逐行 diff 相同（双渠道一致）。

（本环境无 Gowin EDA / iverilog，可综合性依据：Verilog-2001 保守子集、同步复位、
无锁存器风格的 always 组合块均带 default/else、显式原语仅 IDDR 与 DCE。）
