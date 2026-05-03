"""
3K 减速器行星轮轴承寿命计算
Bearing Life Calculation for Planet Wheels in a 3K Reducer

理论基础 / Theoretical basis
─────────────────────────────────────────────────────────────
1. 运动学 Kinematics
   采用威利斯方程 (Willis equation) 推导行星轮自转转速和行星架转速。
   对于 NGW 型 3K 减速器（K1 固定，太阳轮输入，K2 输出）：
       n_H = n_sun · z_sun / (z_sun + z_K1)

2. 齿轮啮合力 Gear mesh forces
   太阳轮-行星轮啮合切向力（均布到各行星轮）：
       Ft = T_input / (r_sun · n_planets)

3. 行星轮轴承载荷 Planet bearing load
   行星轮同时与太阳轮（外啮合）和内齿圈（内啮合）接触。
   经向量分析，两侧啮合力的径向分量相互抵消，切向分量叠加：
       Fr_bearing = 2 · Ft

4. 当量动载荷 Equivalent dynamic load (ISO 281)
       P = X · Fr + Y · Fa

5. 基本额定寿命 Basic rated life (ISO 281)
       L10  = (C / P)^p            [× 10⁶ 转 / million revolutions]
       L10h = L10 · 10⁶ / (60 · n) [小时 / hours]
   球轴承 ball bearing: p = 3
   滚子轴承 roller bearing: p = 10/3
"""

import math

# ── 常量 Constants ──────────────────────────────────────────────────────────────
_MILLION = 1.0e6
_MIN_PER_HOUR = 60.0


# ── 运动学 Kinematics ───────────────────────────────────────────────────────────

def planet_speed(n_input: float, z_sun: int, z_ring: int):
    """计算行星轮自转转速和行星架转速（K1 固定）。

    Planet self-rotation speed and carrier speed (K1 fixed).

    威利斯方程 Willis equation (K1 fixed, n_ring = 0):
        (n_sun - n_H) / (0 - n_H) = -z_ring / z_sun
        → n_H = n_sun · z_sun / (z_sun + z_ring)

    行星轮自转 Planet spin (relative to carrier):
        n_spin = (n_sun - n_H) · z_sun / z_planet
        z_planet = (z_ring - z_sun) / 2

    Parameters
    ----------
    n_input : float
        输入（太阳轮）转速 [rpm]  Input (sun gear) speed.
    z_sun : int
        太阳轮齿数  Sun gear tooth count.
    z_ring : int
        K1 内齿圈齿数（固定）  K1 ring gear tooth count (fixed).

    Returns
    -------
    n_spin : float
        行星轮相对行星架的自转转速 [rpm]  Planet self-rotation speed.
    n_carrier : float
        行星架转速 [rpm]  Carrier speed.

    Raises
    ------
    ValueError
        若齿轮齿数不满足装配条件  If gear tooth counts violate assembly conditions.
    """
    if (z_ring - z_sun) % 2 != 0:
        raise ValueError(
            f"(z_ring - z_sun) 必须为偶数以满足装配条件: "
            f"z_ring={z_ring}, z_sun={z_sun}"
        )
    if z_ring <= z_sun:
        raise ValueError(
            f"z_ring 必须大于 z_sun: z_ring={z_ring}, z_sun={z_sun}"
        )

    n_carrier = n_input * z_sun / (z_sun + z_ring)
    z_planet = (z_ring - z_sun) / 2.0
    n_spin = (n_input - n_carrier) * z_sun / z_planet
    return n_spin, n_carrier


# ── 齿轮啮合力 Gear mesh forces ─────────────────────────────────────────────────

def tangential_force(T_input: float, r_sun: float, n_planets: int) -> float:
    """计算单个行星轮上太阳轮侧的啮合切向力。

    Tangential mesh force from sun gear acting on a single planet.

        Ft = T_input / (r_sun · n_planets)

    Parameters
    ----------
    T_input : float
        输入转矩 [N·mm]  Input torque.
    r_sun : float
        太阳轮分度圆半径 [mm]  Sun gear pitch radius.
    n_planets : int
        行星轮个数  Number of planet gears.

    Returns
    -------
    Ft : float
        切向力 [N]  Tangential force.
    """
    if r_sun <= 0:
        raise ValueError(f"太阳轮半径必须大于 0: r_sun={r_sun}")
    if n_planets <= 0:
        raise ValueError(f"行星轮个数必须大于 0: n_planets={n_planets}")
    return T_input / (r_sun * n_planets)


# ── 轴承载荷 Bearing load ───────────────────────────────────────────────────────

def planet_bearing_radial_load(Ft: float) -> float:
    """计算行星轮销轴（轴承）所受径向合力。

    Radial load on the planet pin / bearing.

    对于 NGW 型行星传动：
    • 行星轮与太阳轮的外啮合法向力径向分量：向外
    • 行星轮与内齿圈的内啮合法向力径向分量：向内（两者相消）
    • 两侧切向力方向相同，叠加后：Fr_bearing = 2 · Ft

    For NGW planetary: radial components of sun-mesh and ring-mesh forces
    cancel; tangential components add → Fr_bearing = 2 · Ft.

    Parameters
    ----------
    Ft : float
        啮合切向力 [N]  Mesh tangential force.

    Returns
    -------
    Fr : float
        轴承径向合力 [N]  Bearing radial resultant force.
    """
    return 2.0 * Ft


# ── 当量动载荷 Equivalent dynamic load ─────────────────────────────────────────

def equivalent_load(Fr: float, Fa: float = 0.0,
                    X: float = 1.0, Y: float = 0.0) -> float:
    """计算当量动载荷 P（ISO 281）。

    Equivalent dynamic bearing load per ISO 281.

        P = X · Fr + Y · Fa

    Parameters
    ----------
    Fr : float
        径向载荷 [N]  Radial load.
    Fa : float, optional
        轴向载荷 [N]，默认 0  Axial load (default 0).
    X : float, optional
        径向载荷系数，默认 1.0  Radial load factor (default 1.0).
    Y : float, optional
        轴向载荷系数，默认 0.0  Axial load factor (default 0.0).

    Returns
    -------
    P : float
        当量动载荷 [N]  Equivalent dynamic load.
    """
    return X * Fr + Y * Fa


# ── 基本额定寿命 Basic rated life ───────────────────────────────────────────────

def bearing_life_L10(C: float, P: float, n: float,
                     bearing_type: str = 'ball'):
    """计算轴承基本额定寿命 L10（ISO 281）。

    Basic rated bearing life L10 per ISO 281.

        L10  = (C / P)^p                  [× 10⁶ r]
        L10h = L10 · 10⁶ / (60 · n)      [h]

    寿命指数 Life exponent:
        球轴承 ball bearing:    p = 3
        滚子轴承 roller bearing: p = 10/3

    Parameters
    ----------
    C : float
        基本额定动载荷 [N]  Basic dynamic load rating.
    P : float
        当量动载荷 [N]  Equivalent dynamic load.
    n : float
        轴承转速 [rpm]  Bearing speed (planet self-rotation speed).
    bearing_type : str, optional
        'ball'（球轴承）或 'roller'（滚子轴承），默认 'ball'.

    Returns
    -------
    L10h : float
        基本额定寿命 [h]  Basic rated life in hours.
    L10 : float
        基本额定寿命 [× 10⁶ r]  Basic rated life in million revolutions.

    Raises
    ------
    ValueError
        若 P ≤ 0 或 n ≤ 0.
    """
    if P <= 0:
        raise ValueError(f"当量动载荷 P 必须大于 0，当前 P={P}")
    if n <= 0:
        raise ValueError(f"轴承转速 n 必须大于 0，当前 n={n}")

    p = 3.0 if bearing_type == 'ball' else 10.0 / 3.0
    L10 = (C / P) ** p
    L10h = L10 * _MILLION / (_MIN_PER_HOUR * n)
    return L10h, L10


# ── 主计算接口 Main calculation interface ───────────────────────────────────────

def calculate(params: dict) -> dict:
    """3K 减速器行星轮轴承寿命综合计算。

    Full bearing life calculation for 3K reducer planet wheel bearings.

    Parameters
    ----------
    params : dict
        必填 Required:
            n_input   (float) – 输入转速 [rpm]
            T_input   (float) – 输入转矩 [N·mm]
            z_sun     (int)   – 太阳轮齿数
            z_ring_k1 (int)   – K1 内齿圈齿数（固定圈）
            n_planets (int)   – 行星轮个数
            m         (float) – 模数 [mm]
            C         (float) – 轴承基本额定动载荷 [N]

        可选 Optional（括号内为默认值）:
            alpha        (float, 20)    – 压力角 [°]
            bearing_type (str, 'ball')  – 'ball' 或 'roller'
            X            (float, 1.0)   – 径向载荷系数
            Y            (float, 0.0)   – 轴向载荷系数
            Fa           (float, 0.0)   – 轴向载荷 [N]
            fd           (float, 1.0)   – 动载系数（设计载荷 = fd · Fr）

    Returns
    -------
    results : dict
        几何参数 Geometry:
            z_planet      – 行星轮齿数
            r_sun_mm      – 太阳轮分度圆半径 [mm]
            r_planet_mm   – 行星轮分度圆半径 [mm]
            r_ring_mm     – K1 内齿圈分度圆半径 [mm]
            gear_ratio    – 传动比 i = (z_sun + z_ring) / z_sun

        转速 Speeds:
            n_carrier_rpm      – 行星架转速 [rpm]
            n_planet_spin_rpm  – 行星轮自转转速 [rpm]

        啮合力 Mesh forces:
            Ft_N      – 切向力 [N]
            Fr_mesh_N – 径向力 [N]  (= Ft · tan α)
            Fn_N      – 法向力 [N]  (= Ft / cos α)

        轴承载荷 Bearing loads:
            Fr_bearing_N – 轴承径向合力 [N]  (= 2 · Ft)
            Fr_design_N  – 设计径向力 [N]   (= fd · Fr_bearing)
            P_N          – 当量动载荷 [N]

        寿命 Life:
            L10_Mrev – 基本额定寿命 [× 10⁶ r]
            L10h     – 基本额定寿命 [h]
    """
    # 参数提取 Extract parameters
    n_input      = params['n_input']
    T_input      = params['T_input']
    z_sun        = params['z_sun']
    z_ring_k1    = params['z_ring_k1']
    n_planets    = params['n_planets']
    m            = params['m']
    alpha        = params.get('alpha', 20.0)
    C            = params['C']
    bearing_type = params.get('bearing_type', 'ball')
    X            = params.get('X', 1.0)
    Y            = params.get('Y', 0.0)
    Fa           = params.get('Fa', 0.0)
    fd           = params.get('fd', 1.0)

    # 几何参数 Geometry
    z_planet = (z_ring_k1 - z_sun) / 2.0
    r_sun    = m * z_sun     / 2.0
    r_planet = m * z_planet  / 2.0
    r_ring   = m * z_ring_k1 / 2.0

    # 传动比 Gear ratio (K1 fixed)
    gear_ratio = (z_sun + z_ring_k1) / z_sun

    # 转速 Speeds
    n_spin, n_carrier = planet_speed(n_input, z_sun, z_ring_k1)

    # 啮合力 Mesh forces
    alpha_rad = math.radians(alpha)
    Ft        = tangential_force(T_input, r_sun, n_planets)
    Fr_mesh   = Ft * math.tan(alpha_rad)
    Fn        = Ft / math.cos(alpha_rad)

    # 轴承径向力 Bearing radial force
    Fr_bearing = planet_bearing_radial_load(Ft)
    Fr_design  = Fr_bearing * fd

    # 当量动载荷 Equivalent load
    P = equivalent_load(Fr_design, Fa, X, Y)

    # 轴承寿命 Bearing life
    L10h, L10 = bearing_life_L10(C, P, n_spin, bearing_type)

    return {
        # Geometry
        'z_planet'     : z_planet,
        'r_sun_mm'     : r_sun,
        'r_planet_mm'  : r_planet,
        'r_ring_mm'    : r_ring,
        'gear_ratio'   : gear_ratio,
        # Speeds
        'n_carrier_rpm'     : n_carrier,
        'n_planet_spin_rpm' : n_spin,
        # Mesh forces
        'Ft_N'     : Ft,
        'Fr_mesh_N': Fr_mesh,
        'Fn_N'     : Fn,
        # Bearing loads
        'Fr_bearing_N': Fr_bearing,
        'Fr_design_N' : Fr_design,
        'P_N'         : P,
        # Life
        'L10_Mrev': L10,
        'L10h'    : L10h,
    }


# ── 格式化报告 Formatted report ─────────────────────────────────────────────────

def print_report(params: dict, results: dict) -> None:
    """打印格式化计算报告。

    Print a formatted calculation report to stdout.

    Parameters
    ----------
    params : dict
        传入 `calculate()` 的参数字典  Input parameters dict.
    results : dict
        `calculate()` 返回的结果字典  Results dict from `calculate()`.
    """
    sep = '=' * 64
    print(sep)
    print('  3K 减速器行星轮轴承寿命计算报告')
    print('  Bearing Life Report — 3K Reducer Planet Wheel')
    print(sep)

    print('\n┌─ 输入参数 Input Parameters '
          '──────────────────────────────────')
    print(f'│  输入转速        n_input      ='
          f' {params["n_input"]:>10.1f}  rpm')
    print(f'│  输入转矩        T_input      ='
          f' {params["T_input"]:>10.1f}  N·mm')
    print(f'│  太阳轮齿数      z_sun        ='
          f' {params["z_sun"]:>10d}')
    print(f'│  K1 内齿圈齿数   z_ring_k1    ='
          f' {params["z_ring_k1"]:>10d}')
    print(f'│  行星轮个数      n_planets    ='
          f' {params["n_planets"]:>10d}')
    print(f'│  模数            m            ='
          f' {params["m"]:>10.2f}  mm')
    print(f'│  压力角          alpha        ='
          f' {params.get("alpha", 20):>10.1f}  °')
    print(f'│  轴承类型                     ='
          f' {params.get("bearing_type", "ball"):>10s}')
    print(f'│  额定动载荷      C            ='
          f' {params["C"]:>10.1f}  N')
    print(f'│  动载系数        fd           ='
          f' {params.get("fd", 1.0):>10.3f}')

    print('\n├─ 几何与运动参数 Geometry & Kinematics '
          '──────────────────────')
    print(f'│  行星轮齿数      z_planet     ='
          f' {results["z_planet"]:>10.0f}')
    print(f'│  传动比          i            ='
          f' {results["gear_ratio"]:>10.4f}')
    print(f'│  行星架转速      n_H          ='
          f' {results["n_carrier_rpm"]:>10.2f}  rpm')
    print(f'│  行星轮自转转速  n_planet     ='
          f' {results["n_planet_spin_rpm"]:>10.2f}  rpm')

    print('\n├─ 齿轮啮合力 Gear Mesh Forces '
          '────────────────────────────────')
    print(f'│  切向力          Ft           ='
          f' {results["Ft_N"]:>10.2f}  N')
    print(f'│  径向力          Fr_mesh      ='
          f' {results["Fr_mesh_N"]:>10.2f}  N')
    print(f'│  法向力          Fn           ='
          f' {results["Fn_N"]:>10.2f}  N')

    print('\n├─ 轴承载荷 Bearing Loads '
          '──────────────────────────────────────')
    print(f'│  轴承径向合力    Fr_bearing   ='
          f' {results["Fr_bearing_N"]:>10.2f}  N')
    print(f'│  设计径向力      Fr_design    ='
          f' {results["Fr_design_N"]:>10.2f}  N')
    print(f'│  当量动载荷      P            ='
          f' {results["P_N"]:>10.2f}  N')

    print('\n└─ 轴承寿命 Bearing Life '
          '───────────────────────────────────────')
    print(f'   基本额定寿命    L10          ='
          f' {results["L10_Mrev"]:>10.4f}  × 10⁶ r')
    print(f'   基本额定寿命    L10h         ='
          f' {results["L10h"]:>10.2f}  h')
    print(sep)


# ── 示例入口 Example entry point ────────────────────────────────────────────────

if __name__ == '__main__':
    example_params = {
        # 减速器参数 Reducer parameters
        'n_input'     : 1500,    # 输入转速 [rpm]
        'T_input'     : 50000,   # 输入转矩 [N·mm]  (= 50 N·m)
        'z_sun'       : 20,      # 太阳轮齿数
        'z_ring_k1'   : 60,      # K1 内齿圈齿数（固定）
        'n_planets'   : 3,       # 行星轮个数
        'm'           : 2,       # 模数 [mm]
        'alpha'       : 20,      # 压力角 [°]
        # 轴承参数 Bearing parameters
        'bearing_type': 'ball',  # 轴承类型：ball / roller
        'C'           : 25000,   # 基本额定动载荷 [N]  (= 25 kN)
        'X'           : 1.0,     # 径向载荷系数
        'Y'           : 0.0,     # 轴向载荷系数
        'Fa'          : 0.0,     # 轴向载荷 [N]
        'fd'          : 1.25,    # 动载系数
    }

    results = calculate(example_params)
    print_report(example_params, results)
