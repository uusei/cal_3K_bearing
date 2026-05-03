"""
单元测试 — 3K 减速器行星轮轴承寿命计算
Unit tests for cal_3K_bearing
"""

import math
import unittest

from cal_3K_bearing import (
    bearing_life_L10,
    calculate,
    equivalent_load,
    planet_bearing_radial_load,
    planet_speed,
    tangential_force,
)


class TestPlanetSpeed(unittest.TestCase):
    """planet_speed() — 行星轮转速计算"""

    def test_carrier_speed(self):
        """行星架转速 n_H = n_input · z_sun / (z_sun + z_ring)"""
        n_spin, n_carrier = planet_speed(1500, 20, 60)
        expected_carrier = 1500 * 20 / (20 + 60)   # 375 rpm
        self.assertAlmostEqual(n_carrier, expected_carrier, places=6)

    def test_planet_spin_speed(self):
        """行星轮自转 n_spin = (n_input - n_H) · z_sun / z_planet"""
        n_spin, n_carrier = planet_speed(1500, 20, 60)
        # z_planet = (60-20)/2 = 20
        # n_spin = (1500 - 375) * 20 / 20 = 1125 rpm
        self.assertAlmostEqual(n_spin, 1125.0, places=6)

    def test_carrier_zero_when_ring_infinite(self):
        """当 z_ring >> z_sun 时，行星架趋近于 0（极端比值）"""
        _, n_carrier = planet_speed(1000, 10, 990)
        self.assertAlmostEqual(n_carrier, 1000 * 10 / 1000, places=6)

    def test_invalid_odd_difference(self):
        """(z_ring - z_sun) 为奇数时应抛出 ValueError"""
        with self.assertRaises(ValueError):
            planet_speed(1000, 20, 61)   # 61-20 = 41 (奇数)

    def test_invalid_ring_smaller_than_sun(self):
        """z_ring ≤ z_sun 时应抛出 ValueError"""
        with self.assertRaises(ValueError):
            planet_speed(1000, 60, 20)


class TestTangentialForce(unittest.TestCase):
    """tangential_force() — 啮合切向力计算"""

    def test_basic(self):
        """Ft = T_input / (r_sun · n_planets)"""
        r_sun = 2 * 20 / 2.0   # m=2, z=20 → r=20 mm
        Ft = tangential_force(50000, r_sun, 3)
        self.assertAlmostEqual(Ft, 50000 / (20 * 3), places=6)

    def test_invalid_r_sun(self):
        with self.assertRaises(ValueError):
            tangential_force(50000, 0, 3)

    def test_invalid_n_planets(self):
        with self.assertRaises(ValueError):
            tangential_force(50000, 20, 0)

    def test_single_planet(self):
        """单个行星轮承受全部载荷"""
        Ft = tangential_force(10000, 50, 1)
        self.assertAlmostEqual(Ft, 200.0, places=6)


class TestPlanetBearingRadialLoad(unittest.TestCase):
    """planet_bearing_radial_load() — 轴承径向合力"""

    def test_double_tangential(self):
        """Fr_bearing = 2 · Ft"""
        self.assertAlmostEqual(planet_bearing_radial_load(500), 1000.0)

    def test_zero(self):
        self.assertAlmostEqual(planet_bearing_radial_load(0), 0.0)


class TestEquivalentLoad(unittest.TestCase):
    """equivalent_load() — 当量动载荷"""

    def test_radial_only(self):
        """P = X · Fr  (Fa = 0, Y = 0)"""
        P = equivalent_load(1000, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(P, 1000.0)

    def test_combined(self):
        """P = X · Fr + Y · Fa"""
        P = equivalent_load(1000, 500, X=1.0, Y=0.5)
        self.assertAlmostEqual(P, 1250.0)

    def test_default_args(self):
        """默认参数：P = Fr"""
        self.assertAlmostEqual(equivalent_load(800), 800.0)


class TestBearingLifeL10(unittest.TestCase):
    """bearing_life_L10() — L10 寿命计算"""

    def test_ball_bearing_L10(self):
        """L10 = (C/P)^3  (ball bearing)"""
        C, P, n = 25000, 2083.33, 1125
        L10h, L10 = bearing_life_L10(C, P, n, 'ball')
        expected_L10 = (C / P) ** 3
        self.assertAlmostEqual(L10, expected_L10, places=3)

    def test_ball_bearing_L10h(self):
        """L10h = L10 · 10⁶ / (60 · n)"""
        C, P, n = 25000, 2083.33, 1125
        L10h, L10 = bearing_life_L10(C, P, n, 'ball')
        expected_L10h = L10 * 1e6 / (60 * n)
        self.assertAlmostEqual(L10h, expected_L10h, places=3)

    def test_roller_bearing_exponent(self):
        """滚子轴承 p = 10/3"""
        C, P, n = 30000, 3000, 500
        L10h_roller, L10_roller = bearing_life_L10(C, P, n, 'roller')
        L10h_ball,   L10_ball   = bearing_life_L10(C, P, n, 'ball')
        # p_roller = 10/3 > 3 = p_ball  →  L10_roller > L10_ball for C > P
        self.assertGreater(L10_roller, L10_ball)

    def test_invalid_P(self):
        with self.assertRaises(ValueError):
            bearing_life_L10(25000, 0, 1125)

    def test_invalid_n(self):
        with self.assertRaises(ValueError):
            bearing_life_L10(25000, 2000, 0)

    def test_c_equals_p_gives_one(self):
        """当 C = P 时，L10 = 1 (× 10⁶ r)"""
        _, L10 = bearing_life_L10(5000, 5000, 100)
        self.assertAlmostEqual(L10, 1.0)


class TestCalculate(unittest.TestCase):
    """calculate() — 综合计算接口"""

    def setUp(self):
        self.params = {
            'n_input'     : 1500,
            'T_input'     : 50000,
            'z_sun'       : 20,
            'z_ring_k1'   : 60,
            'n_planets'   : 3,
            'm'           : 2,
            'alpha'       : 20,
            'bearing_type': 'ball',
            'C'           : 25000,
            'X'           : 1.0,
            'Y'           : 0.0,
            'Fa'          : 0.0,
            'fd'          : 1.25,
        }

    def test_geometry(self):
        """几何参数计算正确"""
        r = calculate(self.params)
        self.assertAlmostEqual(r['z_planet'], 20.0)
        self.assertAlmostEqual(r['r_sun_mm'], 20.0)
        self.assertAlmostEqual(r['r_planet_mm'], 20.0)
        self.assertAlmostEqual(r['r_ring_mm'], 60.0)
        self.assertAlmostEqual(r['gear_ratio'], 4.0)

    def test_speeds(self):
        """转速计算正确"""
        r = calculate(self.params)
        self.assertAlmostEqual(r['n_carrier_rpm'], 375.0)
        self.assertAlmostEqual(r['n_planet_spin_rpm'], 1125.0)

    def test_tangential_force(self):
        """切向力 Ft = 50000 / (20 · 3) = 833.33 N"""
        r = calculate(self.params)
        self.assertAlmostEqual(r['Ft_N'], 50000 / (20 * 3), places=2)

    def test_bearing_radial_load(self):
        """轴承径向合力 = 2 · Ft"""
        r = calculate(self.params)
        self.assertAlmostEqual(r['Fr_bearing_N'], 2 * r['Ft_N'], places=6)

    def test_design_load_with_fd(self):
        """设计载荷 = fd · Fr_bearing"""
        r = calculate(self.params)
        self.assertAlmostEqual(r['Fr_design_N'],
                               r['Fr_bearing_N'] * self.params['fd'],
                               places=6)

    def test_equivalent_load(self):
        """当量载荷 P = X · Fr_design（无轴向载荷时）"""
        r = calculate(self.params)
        self.assertAlmostEqual(r['P_N'], r['Fr_design_N'], places=6)

    def test_L10_positive(self):
        """寿命结果为正数"""
        r = calculate(self.params)
        self.assertGreater(r['L10_Mrev'], 0)
        self.assertGreater(r['L10h'], 0)

    def test_L10h_formula(self):
        """L10h = L10 · 10⁶ / (60 · n_planet)"""
        r = calculate(self.params)
        expected = r['L10_Mrev'] * 1e6 / (60 * r['n_planet_spin_rpm'])
        self.assertAlmostEqual(r['L10h'], expected, places=4)

    def test_defaults_applied(self):
        """缺省可选参数（fd=1, alpha=20, bearing_type='ball'）"""
        minimal = {
            'n_input'  : 1000,
            'T_input'  : 30000,
            'z_sun'    : 18,
            'z_ring_k1': 54,
            'n_planets': 3,
            'm'        : 2,
            'C'        : 20000,
        }
        r = calculate(minimal)
        self.assertGreater(r['L10h'], 0)

    def test_roller_bearing_longer_life(self):
        """在相同条件下，滚子轴承额定寿命更长（p=10/3 > 3）"""
        r_ball   = calculate({**self.params, 'bearing_type': 'ball'})
        r_roller = calculate({**self.params, 'bearing_type': 'roller'})
        self.assertGreater(r_roller['L10h'], r_ball['L10h'])

    def test_higher_C_longer_life(self):
        """更高额定载荷 C 对应更长寿命"""
        r_lo = calculate({**self.params, 'C': 20000})
        r_hi = calculate({**self.params, 'C': 40000})
        self.assertGreater(r_hi['L10h'], r_lo['L10h'])

    def test_higher_torque_shorter_life(self):
        """更大输入转矩对应更短寿命"""
        r_lo = calculate({**self.params, 'T_input': 30000})
        r_hi = calculate({**self.params, 'T_input': 80000})
        self.assertGreater(r_lo['L10h'], r_hi['L10h'])

    def test_known_values(self):
        """端到端数值校验（手工推导结果）

        z_planet = (60-20)/2 = 20
        r_sun    = 2·20/2   = 20 mm
        n_H      = 1500·20/80 = 375 rpm
        n_spin   = (1500-375)·20/20 = 1125 rpm
        Ft       = 50000/(20·3) ≈ 833.33 N
        Fr_bear  = 2·833.33 ≈ 1666.67 N
        Fr_des   = 1666.67·1.25 ≈ 2083.33 N
        P        = 2083.33 N
        L10      = (25000/2083.33)^3 ≈ 1728 × 10⁶ r
        L10h     = 1728·10⁶/(60·1125) ≈ 25600 h
        """
        r = calculate(self.params)
        self.assertAlmostEqual(r['n_carrier_rpm'],      375.0,    places=1)
        self.assertAlmostEqual(r['n_planet_spin_rpm'], 1125.0,    places=1)
        self.assertAlmostEqual(r['Ft_N'],               833.33,   places=1)
        self.assertAlmostEqual(r['Fr_bearing_N'],      1666.67,   places=1)
        self.assertAlmostEqual(r['Fr_design_N'],       2083.33,   places=1)
        self.assertAlmostEqual(r['P_N'],               2083.33,   places=1)
        self.assertAlmostEqual(r['L10_Mrev'],          1728.0,    places=0)
        self.assertAlmostEqual(r['L10h'],             25600.0,    places=0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
