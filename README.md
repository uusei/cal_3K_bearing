# cal_3K_bearing

3K 减速器行星轮轴承寿命计算  
**Bearing Life Calculation for Planet Wheel Bearings in a 3K Reducer**

---

## 功能简介 Overview

`cal_3K_bearing.py` 根据 **ISO 281** 标准，计算 NGW 型 3K 行星减速器中
行星轮销轴（行星轮轴承）的基本额定寿命 L10。

`cal_3K_bearing.py` calculates the basic rated bearing life **L10** for the
planet-wheel pin bearings of an NGW-type 3K planetary reducer, following the
**ISO 281** standard.

仓库还包含 `3k_bearing_calculator.xlsx`，用于按题述推导直接在 Excel 中录入参数、
自动重算双联行星轮轴承载荷、寿命与静安全系数。

The repository also includes `3k_bearing_calculator.xlsx`, an Excel calculator
for the paired-planet bearing derivation so users can update inputs and
recalculate load, life, and static safety directly in a worksheet.

---

## 计算原理 Theory

| 步骤 Step | 公式 Formula |
|-----------|-------------|
| 行星架转速 Carrier speed | `n_H = n_input · z_sun / (z_sun + z_K1)` |
| 行星轮自转转速 Planet spin | `n_spin = (n_input − n_H) · z_sun / z_planet` |
| 啮合切向力 Tangential mesh force | `Ft = T_input / (r_sun · n_planets)` |
| 轴承径向合力 Bearing radial load | `Fr = 2 · Ft` (切向分量叠加，径向分量抵消) |
| 当量动载荷 Equivalent dynamic load | `P = X · Fr + Y · Fa` |
| 基本额定寿命 Basic rated life | `L10 = (C/P)^p` (球轴承 p=3；滚子轴承 p=10/3) |
| 小时寿命 Life in hours | `L10h = L10 × 10^6 / (60 · n_spin)` |

---

## 快速开始 Quick Start

```bash
python cal_3K_bearing.py
```

内置示例参数将自动执行并打印完整报告。  
The built-in example runs automatically and prints a full report.

### Excel 模板 Excel workbook

- 打开 `3k_bearing_calculator.xlsx`
- 修改蓝色输入单元格
- 工作表会自动重算
- 表中同时给出矢量合成载荷和保守同向叠加载荷，寿命计算默认采用矢量合成值

---

## 作为模块使用 Use as a Module

```python
from cal_3K_bearing import calculate, print_report

params = {
    # 减速器参数 Reducer parameters
    'n_input'     : 1500,    # 输入转速 [rpm]
    'T_input'     : 50000,   # 输入转矩 [N·mm]
    'z_sun'       : 20,      # 太阳轮齿数
    'z_ring_k1'   : 60,      # K1 内齿圈齿数（固定）
    'n_planets'   : 3,       # 行星轮个数
    'm'           : 2,       # 模数 [mm]
    # 轴承参数 Bearing parameters
    'C'           : 25000,   # 基本额定动载荷 [N]
    # 可选 Optional (defaults shown)
    'alpha'       : 20,      # 压力角 [degrees]
    'bearing_type': 'ball',  # 'ball' or 'roller'
    'X'           : 1.0,     # 径向载荷系数
    'Y'           : 0.0,     # 轴向载荷系数
    'Fa'          : 0.0,     # 轴向载荷 [N]
    'fd'          : 1.25,    # 动载系数
}

results = calculate(params)
print_report(params, results)

print(f"L10h = {results['L10h']:.1f} h")
```

---

## 参数说明 Parameters

### 必填 Required

| 参数 | 说明 | 单位 |
|------|------|------|
| `n_input` | 输入（太阳轮）转速 Input speed | rpm |
| `T_input` | 输入转矩 Input torque | N·mm |
| `z_sun` | 太阳轮齿数 Sun gear tooth count | — |
| `z_ring_k1` | K1 内齿圈齿数（固定圈）Fixed ring gear tooth count | — |
| `n_planets` | 行星轮个数 Number of planet gears | — |
| `m` | 模数 Module | mm |
| `C` | 轴承基本额定动载荷 Basic dynamic load rating | N |

### 可选 Optional

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `alpha` | `20` | 压力角 Pressure angle [degrees] |
| `bearing_type` | `'ball'` | `'ball'` or `'roller'` |
| `X` | `1.0` | 径向载荷系数 Radial load factor |
| `Y` | `0.0` | 轴向载荷系数 Axial load factor |
| `Fa` | `0.0` | 轴向载荷 Axial load [N] |
| `fd` | `1.0` | 动载系数 Dynamic load factor |

---

## 运行测试 Run Tests

```bash
python -m unittest test_cal_3K_bearing -v
```

共 33 个单元测试，覆盖每个函数及端到端数值校验。  
33 unit tests covering every function and an end-to-end numerical check.

---

## 依赖 Dependencies

仅使用 Python 标准库 (`math`, `unittest`)，无需额外安装。  
Standard library only — no extra dependencies.
