# docs/refs — 开发参考资料索引

## 引脚与映射（开发必查，文本可git追踪）
| 文件 | 内容 |
|---|---|
| `final_bank_table_full.csv` | **引脚总表（V1.4网表）**：118行 = 底板网络名 ↔ IT68051脚号 ↔ 核心板连接器pad ↔ SchLib名 ↔ bank ↔ FPGA球脚 ↔ IO全名(含GCLK/双功能标记)。分配新引脚/写约束/查球脚都以它为准 |
| `schlib_pins.csv` | ACG525核心板连接器200 pad权威映射（SchLib二进制解析，pad→信号名），含40个空闲pad |
| `pad_ball.json` | SchLib名→FPGA球脚映射（经UG985交叉验证，133对零错配） |
| `ug985_ug324.json` | GW5A-25 UG324每个IO：球脚/bank/全名（含GCLKT_x、SSPI_x等双功能标记） |
| `ft601_pinout.csv` | FT601Q全部76脚：名称/方向(245与Multi-Channel两种模式)/上下拉/数据手册出处 |
| `硬件审查报告.md` | 整板原理图审查报告（V1.4）：IT68051/FT601/HDMI/电源逐项结论、HPD电路验证、与官方EVB差异表 |
| `parts_spec.md` | 关键元件实测规格（磁珠/buck/ESD/TVS/晶振，含立创链接） |

## 数据手册（docs/refs/datasheets/，PDF较大已gitignore）
| 文件 | 用途 |
|---|---|
| `IT68051_DataSheet_V0.94.pdf` | 主芯片：引脚表Table 4-1~4-5、DDR双像素模式(§150MHz/300M、HTotal%4)、电流§7.6 |
| `IT68051_Programming_Guide_20171201.pdf` | **寄存器级权威文档**：初始化表来源、CSC/422输出配置、bank3 0xFA等 |
| `DS_FT600Q-FT601Q.pdf` | FT601Q：引脚表、245同步时序表4.2(T1-T4)、§4.1模式strap、§4.5晶体要求 |
| `DS1103-*.pdf` | GW5A-25数据手册：资源表1-1、电气特性、FMAXIDDR=400Mbps、绝对最大值 |
| `UG304-*.pdf` | GPIO/IOLOGIC：IDDR/ODDR原语、DDR推断规则、SSPI双功能脚 |
| `UG306-*.pdf` | 时钟：DCE原语(CE高有效)、GCLK/HCLK架构、时钟 entering 规则 |
| `UG714-*.pdf` | 配置手册：MSPI自启动 vs SSPI从机配置、Dual-Purpose Pin设置后果 |
| `UG985-*.pdf` | Pinout手册：UG324每球bank/双功能（八球复用N6/P7/N5/R3/T3/R5/U5/V5的依据） |
| `UG987-*.pdf` | 原理图指导：电源上电顺序、去耦、磁珠合并规则 |
| `HDMI2.0_HW_Guidelines_V12.pdf` | ITE官方：TMDS布线、ESD选型(ESD3V3U4ULC)、DDC低容要求 |

## EVB源码（docs/refs/evb_src/）
ITE官方EVB固件源码，是 `rtl/it68051_init_rom.v` 初始化序列和寄存器处理的原始依据。
寄存器级问题先查这里：CSC/422输出、SCDC、EDID SRAM机制(0xA8/0xA9+bank0 0x4B)的实现参考。
注意：EVB的EDID表内容**不可照抄**（Max TMDS 300MHz，不支持2K144），机制可学、内容自造。
