# 硬件文件

六足机器人的 PCB 与机械结构文件。固件 (pico/) 与硬件配套关系见根目录 README
与 STATUS 的硬件章节。

> 文件尚未入库时对应目录由 `.gitkeep` 占位。
> EDA 工具: **立创EDA** (导出流程见文末)。

## 目录约定

| 路径 | 内容 | 要求 |
|---|---|---|
| `pcb/source/` | 立创EDA 工程 (另存到本地: 文件→另存工程到本地) | 原样放入, 保持工程原名; 改版后同步替换 |
| `pcb/schematic.pdf` | 原理图 PDF | 每次改版从 EDA 重新导出, 与源工程同版 |
| `pcb/board/gerber/` | 打板文件 | 导出为 zip 压缩包, 命名 `hexapod_pcb_v2_2026-08.zip` 形式 (版本号+日期) |
| `pcb/board/assembly/` | EDA 导出的贴装 BOM + 坐标文件 (CPL) | JLC 贴片用, 与打板文件同版导出; 命名 `bom_assembly_日期.csv` / `pickandplace_日期.csv` |
| `pcb/images/` | 板子 3D 渲染图 + 实物照片 | PNG/JPG, 原理图预览替代品 |
| `bom/hexapod_bom.csv` | **全项目物料清单** — 不只 PCB: PCB 元件 + 舵机×18 + PCA9685×2 + 电池 + 3D 打印件 + 紧固件 + 线材 + 成品模块等全部物料 | 一个表看全整个机器人所有物料; csv 便于 git diff, 用"类别"列分组 (电子/舵机/机械/耗材) |
| `mechanical/step/` | 可编辑 CAD 源文件 | Fusion/FreeCAD/SW 导出 STEP, 命名含部件名+版本 (如 `coxa_v3.step`) |
| `mechanical/stl/` | 3D 打印件 | 与 step 同名对应 (如 `coxa_v3.stl`); 腿节 Coxa 45mm / Femur 75mm / Tibia 120mm、底板等 |
| `docs/` | 接线图、装配说明、器件位置图 | 图片或 md |

## 规则

1. **源文件与导出件都要入库** — 只有 Gerber 别人改不了设计, 只有源文件别人得装 EDA 才能看
2. **二进制文件直接提交** — 单个 <2MB 直接进 git; 超过则启用 Git LFS
3. **大版本改动同步打包** — 源工程、schematic.pdf、gerber、全项目 BOM 必须同版本提交
4. **根目录 `nuke_universal.uf2` 不入库** (已在 .gitignore, 为 Pico 恢复工具)

## 立创EDA 导出流程

> 菜单路径在不同版本略有差异, 下列入口均可达 (专业版)。

| 导出物 | 存放位置 | 操作 |
|---|---|---|
| 工程源文件 | `pcb/source/` | 文件 → 另存工程到本地 (含原理图+PCB 图纸) |
| 原理图 PDF | `pcb/schematic.pdf` | 文件 → 导出 → 原理图 PDF |
| 打板文件 (Gerber) | `pcb/board/gerber/` | 文件 → 导出 → PCB制板文件 (Gerber) (或 制造 → PCB制板文件), 先过 DRC, 导出为 zip |
| 贴装 BOM | `pcb/board/assembly/` | 文件 → 导出 → 物料清单 BOM (XLSX/CSV, 建议含立创编号列) |
| 贴装坐标 (CPL) | `pcb/board/assembly/` | 导出 → 坐标文件 |
| 全项目 BOM | `bom/hexapod_bom.csv` | 手工汇总: 从贴装 BOM 拷贝 PCB 元件 + 补充舵机/电池/打印件/紧固件等全部物料 |
| 3D 模型 (可选) | `mechanical/step/` | 导出 → 3D 文件 (STEP) |

快捷下单: 文件 → 导出 → PCB下单 (上传 Gerber 直接下单), 或 元件购买 (上传 BOM 去立创商城配货)。
SMT 贴片在嘉立创下单系统上传 BOM + 坐标文件。

## 许可证

固件为 GPLv3。**硬件文件不适用 GPL** — 建议:

- PCB 设计: **CERN-OHL-S 2.0** (硬件开放许可, 允许商用但需注明来源)
- 机械结构: **CC-BY-SA 4.0**

决定后在 `hardware/` 下放置对应许可证全文, 并在本文件注明分块授权。
