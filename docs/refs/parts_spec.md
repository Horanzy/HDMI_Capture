# 小元件规格核查 (parts_spec)

提取日期: 2026-11-15。来源以立创商城 (lcsc.com / szlcsc) 产品页 HTML 与其托管的原厂数据手册 PDF 为准；
每条参数均标注来源。无法从一手来源确认的条目标 CONFIDENCE:low/medium。

---

## 1. PBY201209T-600Y-N 磁珠 (0805) —— 审查重点

| 参数 | 值 | 来源 |
|---|---|---|
| 品牌 | Chilisin (奇力新) | LCSC C142054 产品页 |
| 封装 | 0805 (2012) | 同上 |
| 100MHz 阻抗 | 60 Ω @ 100 MHz (1 line) | LCSC 属性 "Impedance @ Frequency = 60Ω@100MHz" |
| DCR | 30 mΩ | LCSC 属性 "DC Resistance(DCR) = 30mΩ" |
| 额定电流 | **3 A** (Current Rating) | LCSC 属性 "Current Rating = 3A" |
| 工作温度 | -55 ~ +125 ℃ | LCSC 属性 |
| 精度 | ±25% (阻抗) | WebSearch 摘要 (LCSC 页), CONFIDENCE:medium |

- 来源: https://www.lcsc.com/product-detail/ferrite-beads_chilisin-elec-pby201209t-600y-n_C142054.html
- 审查提示: 3A 为 LCSC 标注的额定电流，30mΩ DCR 在 IT68051 各电源轨串联时压降可忽略；单珠在 >1A 轨上的温升与
  磁珠饱和（阻抗随偏置电流下降）需对比 IT68051 各轨实际电流评估。FTDI DS §6.1/6.2 (p23) 也明确要求 USB 电源串
  磁珠时 "value of the Ferrite Bead depends on the total current drawn by the application"。

## 2. SY8089AAAC (Silergy, SOT-23-5 同步 buck)

| 参数 | 值 | 来源 |
|---|---|---|
| 输入电压 | 2.7 – 5.5 V | AN_SY8089 Rev0.9A (LCSC 托管) 电气表 "Input Voltage Range VIN 2.7 5.5 V"; LCSC C78988 属性 |
| 最大输出电流 | 2 A 连续 / 3 A 峰值 | AN_SY8089 标题 "5.5V, 2A continuous, 3A peak, 1MHz Synchronous Step Down Regulator" |
| FB 基准 VREF | **0.588 / 0.6 / 0.612 V** (min/typ/max) | AN_SY8089 "Feedback Reference Voltage VREF 0.588 0.6 0.612 V" |
| Vout 公式 | Vout = 0.6 × (1 + R1/R2) | AN_SY8089 FB 引脚说明 |
| 开关频率 | 1 MHz | AN_SY8089 标题; LCSC 属性 "Frequency - Switching = 1MHz" |
| 电流限 | 3.5 A (PFET ILIM) | AN_SY8089 电气表 |
| 内阻 | PFET 110 mΩ / NFET 80 mΩ | AN_SY8089 电气表 |
| 静态电流 | 55 µA | AN_SY8089; LCSC 属性 |

- 来源: https://www.lcsc.com/product-detail/DC-DC-Converters_Silergy-Corp-SY8089AAAC_C78988.html
  DS: https://datasheet.lcsc.com/datasheet/pdf/400ca014d7a690f5be0127468f87fcc6.pdf
- **分压验算 (FB=0.6V typ, 用 R1=反馈到VOUT的上端电阻, R2=反馈到GND的下端电阻):**
  - 1.0V 轨: 0.6 × (1 + 100k/150k) = 0.6 × 1.6667 = **1.000 V** ✔ (考虑 VREF 0.588–0.612V → 0.980–1.020V)
  - 3.3V 轨: 0.6 × (1 + 200k/44.2k) = 0.6 × 5.5249 = **3.315 V** ✔ (≈3.31V; VREF 偏差 → 3.250–3.384V)
  - 两组分压与 FB=0.6V 自洽，验算通过。

## 3. ESD3V3U4ULC-P (2.5x1.0) (TECH PUBLIC 台舟电子, DFN2510-10, LCSC C2923367) —— 引脚图为审查关键

| 参数 | 值 | 来源 |
|---|---|---|
| 品牌/封装 | TECH PUBLIC, DFN2510-10 (2.5×1.0×0.5 mm) | LCSC C2923367 属性 + 原厂 DS p1 |
| 类型 | 4 通道单向 (Uni-directional) TVS/ESD 阵列 | 原厂 DS p1 "4-Line Uni-directional TVS Diode" |
| VRWM | 3.3 V | DS p2 电气表; LCSC 属性 |
| VBR | 3.6 min / 4 typ / 5.5 max V @ IT=1mA | DS p2 电气表 |
| VC | 15 V max @ IPP=4A (8×20µs), any I/O to GND | DS p2 电气表 |
| IPP / PPP | 4 A @ 8/20µs / 70 W | DS p2; LCSC 属性 |
| IR | 0.06 µA typ @ VRWM=3.3V | DS p2 |
| **结电容 CJ** | **0.3 typ / 0.4 max pF** (VR=0V, 1MHz, I/O-to-I/O)；**0.8 max pF (any I/O to GND)** | DS p2 电气表 |
| ESD 能力 | IEC61000-4-2 空气 ±25kV / 接触 ±20kV | DS p1/p2 |
| 应用 | HDMI/DVI/UDI/MDDI/PCIe/SATA | DS p1 Applications |

**引脚定义 (原厂 DS p1 "Dimensions and Pin Configuration", 已放大目视核对):**

| 脚 | 信号 | 脚 | 信号 |
|---|---|---|---|
| 1 | I/O_1 | 6 | NC |
| 2 | I/O_2 | 7 | NC |
| 3 | GND | 8 | GND |
| 4 | I/O_3 | 9 | NC |
| 5 | I/O_4 | 10 | NC |

- 内部结构: 每个 I/O 一对二极管夹在两条 GND 母线之间（上母线=脚8，下母线=脚3），中央一只 TVS 稳压管跨接两母线；
  脚 6/7/9/10 为虚线画法 = **内部无连接 (NC)**。
- **审查结论（回答"每通道两脚 1&10, 2&9, 4&7, 5&6, 3&8=GND"的假设）: 与器件实际 pinout 不符。**
  实际每通道只有 **一个** I/O 脚（1/2/4/5），**没有** "1&10、2&9、4&7、5&6" 的通道配对；仅 "3&8=GND" 一半正确。
  若底板把 1&10（等）短接在同一 HDMI TMDS 网络上做对地钳位：由于 6/7/9/10 为 NC，短接本身不会造成短路，钳位功能
  通过 1/2/4/5 脚仍然成立（前提是 3、8 已接地）→ **功能可用但引脚模型错误**，属 footpin 映射/原理图符号错误，需改
  库并复核封装焊盘是否有多余连线。若设计本意是 "信号从脚1进脚10出" 的 flow-through 走法，则电路不成立。
  (CONFIDENCE: high —— 以 C2923367 原厂 DS 为准；同名的 Infineon 原版 ESD3V3U4ULC 为 PG-TSLP-9-1 封装，
  国内克隆版为 10 脚 DFN2510-10，两者脚定义是否一致未逐脚核对，CONFIDENCE:low)
- 来源: https://www.lcsc.com/product-detail/C2923367.html
  DS: https://datasheet.lcsc.com/datasheet/pdf/4a5f0dea8ecbe7f8336155490fc61ac0.pdf (扫描件, 已渲染目视)

## 4. H3V3SD3B (珠海鸿嘉诚 Hongjiacheng, SOD-323, LCSC C20615802)

| 参数 | 值 | 来源 |
|---|---|---|
| 类型 | 双向 (Bi-directional) 3.3V 小电容 ESD | 原厂 DS "H3V3SD3B Bi-directional 3.3V Small Capacitance ESD Rev 2.1" |
| VRWM | 3.3 V max | DS 电气表; LCSC 属性 |
| VBR | 8 V (LCSC: Voltage-Breakdown = 8V) | LCSC 属性; DS 电气表 |
| VC | 19 V @ IPP | LCSC 属性 "Clamping Voltage = 19V" |
| IPP / PPP | 20 A @ 8/20µs / 380 W | DS; LCSC 属性 |
| IR | 500 nA | LCSC 属性 |
| **结电容** | **1.0 pF typ** (CJ @ VR=0V, f=1MHz) | DS "Small capacitance 1.0pF(Typ.)" |
| ESD | ±30 kV 空气/接触 (IEC61000-4-2) | DS Features |

- 来源: https://www.lcsc.com/product-detail/C20615802.html
  DS: https://datasheet.lcsc.com/datasheet/pdf/4c65a703ca4944011b77e05bb654b49d.pdf

## 5. SMF5.0CA (Slkor, SOD-123FL, LCSC C920175)

| 参数 | 值 | 来源 |
|---|---|---|
| VRWM | 5 V | LCSC 属性 |
| VBR | 6.4 V min | LCSC 属性 |
| VC | 9.2 V @ IPP | LCSC 属性 |
| IPP / PPP | 21.7 A / 200 W @ 10/1000 µs | LCSC 属性 |
| IR | 400 µA | LCSC 属性 |
| 极性/封装 | 双向, SOD-123FL, -55~+150 ℃ | LCSC 属性 |
| 结电容 | 未标注 (SMF 系列数据手册通常不给 Cj) | CONFIDENCE:low —— 需要时按通用 SMF5.0CA (Vishay/Bourns) 约几十~上百 pF 估算 |

- 来源: https://www.lcsc.com/product-detail/C920175.html
  (同类: ZHIDE C2683628, 标注 "5VWM 10VC"。通用 SMF5.0CA: VRWM 5V / VBR 6.4-7.0V / VC ≤9.2V@21.5A / 200W)

## 6. BAT54C (High Diode, SOT-23 共阴双肖特基, LCSC C466637)

| 参数 | 值 | 来源 |
|---|---|---|
| 结构 | 1 Pair Common Cathode (共阴) | LCSC 属性 |
| VR | 30 V | LCSC 属性 |
| IF(AV) | 200 mA | LCSC 属性 |
| VF | **1 V max @ 100 mA** | LCSC 属性 "Voltage - Forward(Vf@If) = 1V@100mA" |
| IFSM | 600 mA 非重复 | LCSC 属性 |
| IR | 2 µA @ 25 V | LCSC 属性 |
| 典型 VF | ~0.3–0.5 V @ 小电流 (通用品级) | CONFIDENCE:medium (通识值; LCSC 页未列 typ) |

- 来源: https://www.lcsc.com/product-detail/Schottky-Barrier-Diodes-SBD_High-Diode-BAT54C_C466637.html
  注意: 通用 BAT54C (Infineon/onsemi) VF 规格为 ≤1V@100mA、~0.32V@1mA typ，与上表一致。

## 7. 2N7002 / BSS138 (HPD/DDC 电平转换快速确认)

**2N7002** (LCSC C350313, Slkor):
- VDS 60 V, ID 115 mA, VGS ±20 V, SOT-23 — LCSC 属性
- VGS(th): **1.0 – 2.5 V** (ID=250µA, VGS=VDS) — 原厂 DS (LCSC 托管) 特性表
- VDS(on): ≤0.375 V @ ID=50mA, VGS=5V; ≤3.75 V @ ID=500mA, VGS=10V (≈Rds 7.5Ω@10V)
- DS: https://datasheet.lcsc.com/datasheet/pdf/e2feb323efa20250cd3bbf6f7282def3.pdf
- 注意: 各家 2N7002 参数差异大 (onsemi 2N7002: RDS(on) ≤5Ω @ VGS=4.5V)。CONFIDENCE:medium (以所购厂家 DS 为准)

**BSS138** (LCSC C52895, onsemi):
- VDS 50 V, ID 220 mA, VGS ±20 V, SOT-23 — LCSC 属性
- VGS(th): **0.8 / 1.3 / 1.5 V** (min/typ/max, VDS=VGS, ID=1mA) — onsemi DS
- RDS(on): **3.5 Ω max @ VGS=10V**；**6.0 Ω max @ VGS=4.5V**；VGS=10V, 125℃ → 5.8Ω — onsemi DS
- DS: https://datasheet.lcsc.com/datasheet/pdf/1b0ec857472f4f81b41634099e0bf4b5.pdf

**HPD/DDC 电平转换可用性结论:** DDC 为 I²C (100k 上拉, <3mA)、HPD 为 100k 级上拉信号线, 电流均在 mA 级。
- 3.3V 栅极驱动: BSS138 完全胜任 (Vgs(th)≤1.5V, 4.5V 时 Rds≤6Ω, 3.3V 时导通更深); **可用**。
- 2N7002: Vgs(th) 最大 2.5V, 3.3V 栅压下处于弱导通区 (阈值刚过), 小电流下可工作但余量小, 对 DDC 5V 上拉的
  bidirectional level-shift (经典 BSS138 结构) 建议 BSS138; 若板上 2N7002 用于同样拓扑, 需实测 Vgs(th) 分档。
  (CONFIDENCE: medium)

## 8. PNR3015 系列电感 (APV 爱普微, 磁胶屏蔽合金绕线)

来源: APV 官网 PNR 系列表 https://www.apvind.com/pnrseries/page_6.html (HTML 表格, 2026-11-15 抓取)

| 型号 | L | 容差 | DCR (Ω) | Heat/Irms (A) | Isat (A) |
|---|---|---|---|---|---|
| PNR3015-2R2M | 2.2 µH | ±20% | 0.115 | 2 / 2.3 | **4 / 4.5** |
| PNR3015-3R3M | 3.3 µH | ±20% | 0.175 | 2 / 2.5 | 3.4 / 4 |
| PNR3015-6R8M | 6.8 µH | ±20% | 0.29 | 1.5 / 2 | 2 / 2.5 |
| PNR3015-100M | 10 µH | ±20% | 0.46 | 1.5 / 2 | 1.5 / 2 |

- 表头为 "DC Resistance | Heat Rating Current (Irms) | Saturation Current (Isat)"，每列带 Typ./Max. 两个值；
  5 个数值与 2R2M 行 (0.115 | 2 | 2.3 | 4 | 4.5) 对应关系 CONFIDENCE:medium（typ/max 划分为解读, 原表未逐格核对）。
  第三方 JLCPCB 对同系列 PNR3015-100M 标 "rated 2A / sat 2A" 与上表 Max. 列一致，交叉印证。
- **封装兼容性:** PNR3015 系列统一尺寸 3.0×3.0×1.5 mm（"3015" = 3.0 长×3.0 宽×1.5 高; APV 系列描述
  "PNR3012 = 3×3×1.2mm" 同理），6.8µH 与 2.2µH 同属 PNR3015 → **同一 footprint, 兼容** (footprint 名
  PNR3015-6R8M 放 2.2µH 物料无尺寸冲突)。
- 审查提示: SY8089 3.3V/1A 级轨用 2.2µH/Isat 4A 余量充足; 但 footprin 名义值 6.8µH 与 BOM 2.2µH 不一致属
  BOM/footprint 命名管理问题。

## 9. 27MHz 3225 无源晶体 (CL=9–12pF) 通用规格

- 实例 1 (szlcsc 在售): **YXC X322527MSB4SI**, SMD3225-4P, 3.2×2.5×0.7mm, 27MHz, CL=20pF,
  常温频差 ±10ppm, 稳定度 ±20ppm, **ESR 50Ω**, -40~+85℃。
  来源: https://item.szlcsc.com/9528.html (页面属性表)
- 3225 系列 27MHz 常见 CL 档: 8 / 9 / 12 / 20 pF；ESR 50–150Ω max；频差 ±10/±20ppm。
  来源: WebSearch 摘要 (SMD-3225 规格页, 3X 系列), CONFIDENCE:medium
- **18pF 外部负载电容核对:** CL = C1·C2/(C1+C2) + Cstray。取 C1=C2=18pF → 9pF + Cstray(≈2.5–3pF) ≈ **11.5–12pF**,
  与 CL=12pF 晶体匹配 ✔; 若 BOM 晶体为 CL=9pF, 则 18pF 电容偏大 (负载 ≈11.5pF, 频率偏低), 应改 12pF 电容。
  (晶诺威科技应用参考: CL=12pF → 外接 15–18pF; CL=20pF → 27–33pF; 来源同上, CONFIDENCE:medium)
- 注: FT601Q 的晶体要求与本文无关 —— FTDI DS §4.5 (p18) 要求 **30MHz ±20ppm / 18pF / 50Ω** 晶体接 XI/XO;
  27MHz/3225 为 IT68051 侧时钟。

---

### 附: 本目录产出的证据文件
- review/ft601/DS_FT600Q-FT601Q.pdf (LCSC CDN 下载, v1.05, FT_001118/FTDI#424, 33页)
- review/ft601/ft601_pinout.csv (76脚+EP, 含方向/上下拉/章节引用)
- review/ft601/page7.png (DS Figure 3.2 QFN76 Pin Out 截图), page22/24.png (§6/§7 参考设计截图)
- review/parts/sy8089_ds.pdf, esd3v3u4ulc_ds.pdf (+p1/p2 渲染图, esd_pinout_zoom.png), h3v3sd3b_ds.pdf,
  bss138_ds.pdf, 2n7002_ds.pdf (均为 LCSC datasheet.lcsc.com 托管原件)
