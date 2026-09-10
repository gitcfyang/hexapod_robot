# 硬件文件

六足机器人的 PCB 设计与机械结构。硬件接口说明见 [../STATUS.md](../STATUS.md) 硬件章节。

## PCB (v2, 2026-09)

- [pcb/source/](pcb/source/) — 立创EDA 专业版工程（原理图 + PCB）
- [pcb/schematic.pdf](pcb/schematic.pdf) — 原理图 PDF（网页可直接预览）
- [pcb/board/gerber/](pcb/board/gerber/) — 打板文件（zip 打包，直接上传嘉立创）
- [pcb/board/assembly/](pcb/board/assembly/) — 贴装 BOM + 坐标文件（SMT 贴片用）

设计要点：2S 18650 直供舵机轨（2× AO4407A 门控，上电默认断电）；VBAT 入口
SMBJ18A + 每颗 MOSFET 两端 SMBJ12A；I2C 总线 TVS + BNO055 stub 串阻 + VDD 限流；
电池分压 330k+47k 入 GP28 ADC2。保护设计背景见 STATUS.md「电路保护建议」。

## 物料清单 (BOM)

- [bom/hexapod_bom.md](bom/hexapod_bom.md) — 全项目物料（含购买链接，网页可点击）
- [bom/hexapod_bom.csv](bom/hexapod_bom.csv) — 同名数据（链接为 URL 文本，供程序处理）

## 机械结构

- [mechanical/hexapod.step](mechanical/hexapod.step) — 整机架装配体（18× 双轴舵机版）
- 低成本 3D 打印替代机架：见根目录 README「机架」

## 待补

- 板子 3D 渲染图 / 实物照片（pcb/images/）
- 3D 打印件 STL
- 接线图 / 装配说明（docs/）

## 许可

固件为 GPLv3（根目录 LICENSE）。PCB 与机械文件建议另行采用
**CERN-OHL-S 2.0** / **CC-BY-SA 4.0**（硬件设计不适用 GPL）。
