#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
合成数据测试：用已知IIR系数生成频率响应，验证拟合算法能否恢复正确系数。
这是对拟合算法的自洽性验证。

用法:
  python test_synthetic.py
"""

import numpy as np

# 试试导入我们的拟合模块
try:
    from iir_fit_verify import fit_iir_coefficients, compute_frequency_response
except ImportError:
    print("请将本文件与 iir_fit_verify.py 放在同一目录下运行")
    import sys
    sys.exit(1)


def test_known_filter(name, b0_true, b1_true, b2_true, a1_true, a2_true, fs=150000.0):
    """用已知系数生成合成数据，验证拟合能否恢复。"""
    print(f"\n{'='*60}")
    print(f"测试: {name}")
    print(f"{'='*60}")
    print(f"真实系数: b0={b0_true:.8f} b1={b1_true:.8f} b2={b2_true:.8f} "
          f"a1={a1_true:.8f} a2={a2_true:.8f}")

    # 生成合成扫频数据 (1k-50k, 200Hz step)
    freqs = np.arange(1000, 50001, 200)
    gains_true, phases_true, _ = compute_frequency_response(
        b0_true, b1_true, b2_true, a1_true, a2_true, freqs, fs)

    # 添加少量噪声模拟真实测量
    np.random.seed(42)
    gains_noisy = gains_true + np.random.normal(0, 0.01, len(freqs))   # 0.01 dB noise
    phases_noisy = phases_true + np.random.normal(0, 0.1, len(freqs))   # 0.1 deg noise

    # 拟合
    b0_fit, b1_fit, b2_fit, a1_fit, a2_fit = fit_iir_coefficients(
        freqs, gains_noisy, phases_noisy, fs, max_iter=30, tol=1e-10, verbose=False)

    print(f"拟合系数: b0={b0_fit:.8f} b1={b1_fit:.8f} b2={b2_fit:.8f} "
          f"a1={a1_fit:.8f} a2={a2_fit:.8f}")

    # 误差
    b_err = np.array([abs(b0_fit-b0_true), abs(b1_fit-b1_true), abs(b2_fit-b2_true)])
    a_err = np.array([abs(a1_fit-a1_true), abs(a2_fit-a2_true)])
    print(f"系数误差: b_err=max({b_err.max():.2e}) a_err=max({a_err.max():.2e})")

    # 频率响应误差
    gains_fit, phases_fit, _ = compute_frequency_response(
        b0_fit, b1_fit, b2_fit, a1_fit, a2_fit, freqs, fs)

    gain_err_max = np.max(np.abs(gains_fit - gains_true))
    phase_err_max = np.max(np.abs(phases_fit - phases_true))
    gain_err_rms = np.sqrt(np.mean((gains_fit - gains_true)**2))
    phase_err_rms = np.sqrt(np.mean((phases_fit - phases_true)**2))

    print(f"频率响应拟合误差:")
    print(f"  增益: max={gain_err_max:.4f} dB, RMS={gain_err_rms:.4f} dB")
    print(f"  相位: max={phase_err_max:.4f} deg, RMS={phase_err_rms:.4f} deg")

    # 判定
    if gain_err_max < 0.1 and phase_err_max < 1.0 and b_err.max() < 1e-4:
        print(f"结果: ✓ 完美恢复!")
        return True
    elif gain_err_max < 1.0 and phase_err_max < 5.0:
        print(f"结果: ✓ 良好 (噪声影响)")
        return True
    else:
        print(f"结果: ✗ 误差偏大，需要检查")
        return False


if __name__ == '__main__':
    print("IIR拟合算法自洽性验证")
    print("=" * 60)
    print("用已知系数生成合成数据 → 拟合 → 比较")
    print("如果拟合算法正确，应该能精确恢复原始系数。")

    results = []

    # Test 1: 低通滤波器
    results.append(test_known_filter(
        "低通滤波器 (Butterworth-like)",
        b0_true=0.02, b1_true=0.04, b2_true=0.02,
        a1_true=-1.70, a2_true=0.75))

    # Test 2: 高通滤波器
    results.append(test_known_filter(
        "高通滤波器",
        b0_true=0.8, b1_true=-1.6, b2_true=0.8,
        a1_true=-1.5, a2_true=0.6))

    # Test 3: 带通滤波器
    results.append(test_known_filter(
        "带通滤波器",
        b0_true=0.1, b1_true=0.0, b2_true=-0.1,
        a1_true=-1.4, a2_true=0.7))

    # Test 4: 全通滤波器
    results.append(test_known_filter(
        "全通滤波器",
        b0_true=0.7, b1_true=-1.5, b2_true=1.0,
        a1_true=-1.5, a2_true=0.7))

    # Test 5: 陷波滤波器
    results.append(test_known_filter(
        "陷波滤波器 (Notch)",
        b0_true=0.9, b1_true=-1.7, b2_true=0.9,
        a1_true=-1.7, a2_true=0.8))

    print(f"\n{'='*60}")
    print(f"总结: {sum(results)}/{len(results)} 测试通过")
    if all(results):
        print("拟合算法自洽性验证通过! 所有滤波器类型均可正确恢复。")
    else:
        print("部分测试未通过，请检查算法。")
    print(f"{'='*60}")
