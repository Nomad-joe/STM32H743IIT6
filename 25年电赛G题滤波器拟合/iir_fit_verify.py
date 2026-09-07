#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IIR滤波器系数拟合与验证工具
=============================
功能:
  1. 从串口扫频数据txt中解析频率、幅度(dB)、相位(deg)
  2. 使用迭代重加权最小二乘法(SK方法)拟合IIR系数
  3. 从拟合系数数学推导幅度和相位响应 (1~50kHz, 步长200Hz)
  4. 与扫频数据进行对比，分析误差
  5. 生成对比图表
  6. 输出可用于C代码的系数

用法:
  python iir_fit_verify.py <扫频数据文件.txt> [采样频率Hz] [输出图片名]

  默认采样频率: 150000 Hz (与TIM3触发的ADC采样率一致)

扫频数据格式 (串口输出, 每行一个频率点):
  频率:1000.00 hz   幅度:-0.50 db   相位:-5.23 °
  频率:1200.00 hz   幅度:-0.80 db   相位:-8.12 °
  ...

或者CSV格式:
  1000.00,-0.50,-5.23
  1200.00,-0.80,-8.12

作者: Claude Code
日期: 2026-07-24
"""

import sys
import os
import re
import numpy as np
from io import StringIO

# 尝试导入绘图库
try:
    import matplotlib
    matplotlib.use('TkAgg')  # 或 'Agg' 用于无GUI环境
    import matplotlib.pyplot as plt
    HAS_PLT = True
    # 中文字体设置 (Windows)
    try:
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
    except:
        pass
except ImportError:
    HAS_PLT = False
    print("[WARNING] matplotlib not installed. Plotting disabled.")
    print("          Install with: pip install matplotlib")

# 尝试导入scipy (用于对比验证)
try:
    from scipy import signal as scipy_signal
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("[INFO] scipy not installed. Will use pure numpy implementation.")
    print("       Install with: pip install scipy")


# ============================================================
#  数据解析
# ============================================================

def parse_sweep_data(filepath):
    """
    解析扫频数据文件。
    支持格式:
      - 串口输出: "频率:XXX hz   幅度:XXX db   相位:XXX °"
      - CSV: freq,gain_db,phase_deg
    返回: (freqs, gains_db, phases_deg) 三个numpy数组
    """
    freqs = []
    gains = []
    phases = []

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    # 尝试中文格式: 频率:XXX hz   幅度:XXX db   相位:XXX °
    pattern_cn = r'频率[:：]\s*([\d.]+)\s*Hz.*?幅度[:：]\s*([\-\d.]+)\s*db.*?相位[:：]\s*([\-\d.]+)\s*[°度]'
    matches = re.findall(pattern_cn, content, re.IGNORECASE)

    if matches:
        print(f"[解析] 中文串口格式: 找到 {len(matches)} 个数据点")
        for m in matches:
            freqs.append(float(m[0]))
            gains.append(float(m[1]))
            phases.append(float(m[2]))
    else:
        # 尝试CSV格式: 每行 freq,gain,phase
        lines = content.strip().split('\n')
        csv_matches = 0
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('//'):
                continue
            parts = re.split(r'[,\t\s]+', line)
            if len(parts) >= 3:
                try:
                    freqs.append(float(parts[0]))
                    gains.append(float(parts[1]))
                    phases.append(float(parts[2]))
                    csv_matches += 1
                except ValueError:
                    continue
        if csv_matches > 0:
            print(f"[解析] CSV格式: 找到 {csv_matches} 个数据点")
        else:
            # 最后尝试: 逐行扫描数值
            num_pattern = r'([\d.]+)\s+[\s]*([\-\d.]+)\s+[\s]*([\-\d.]+)'
            for line in content.split('\n'):
                m = re.match(num_pattern, line.strip())
                if m:
                    freqs.append(float(m.group(1)))
                    gains.append(float(m.group(2)))
                    phases.append(float(m.group(3)))
            if freqs:
                print(f"[解析] 数值格式: 找到 {len(freqs)} 个数据点")

    if not freqs:
        print("[ERROR] 未能解析任何数据! 请检查文件格式.")
        print("支持格式: 频率:1000.00 hz   幅度:-0.50 db   相位:-5.23 °")
        print("       或 CSV: 1000.00,-0.50,-5.23")
        sys.exit(1)

    return np.array(freqs), np.array(gains), np.array(phases)


# ============================================================
#  IIR系数拟合 (迭代重加权最小二乘 SK方法)
# ============================================================

def fit_iir_coefficients(freqs, gains_db, phases_deg, fs, max_iter=20, tol=1e-8, verbose=True):
    """
    使用迭代重加权最小二乘法(SK方法)从频率响应数据拟合IIR系数。

    最小化: Σ |H_desired(ω_k) - B(ω_k)/A(ω_k)|²

    H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)

    参数:
      freqs: 频率数组 (Hz)
      gains_db: 增益数组 (dB)
      phases_deg: 相位数组 (度)
      fs: 采样频率 (Hz)
      max_iter: 最大迭代次数
      tol: 收敛容限
      verbose: 是否打印详细信息

    返回:
      (b0, b1, b2, a1, a2) 系数元组
    """
    N = len(freqs)
    omega = 2.0 * np.pi * freqs / fs  # 数字频率

    # 目标频率响应 H_desired
    mag = 10.0 ** (gains_db / 20.0)
    phase_rad = np.radians(phases_deg)
    H_real = mag * np.cos(phase_rad)
    H_imag = mag * np.sin(phase_rad)

    # ------------------------------
    # Step 1: 方程误差法 (Levy) 初始估计
    # ------------------------------
    if verbose:
        print("\n[Step 1] 方程误差法(Levy)初始估计...")

    # 构建方程矩阵 Phi * theta = Y
    # 每个频率点贡献两个方程（实部+虚部）
    eq_num = 2 * N
    Phi = np.zeros((eq_num, 5))
    Y = np.zeros(eq_num)

    cos_w = np.cos(omega)
    cos_2w = np.cos(2.0 * omega)
    sin_w = np.sin(omega)
    sin_2w = np.sin(2.0 * omega)

    # 实部方程 (行 2k)
    Phi[0::2, 0] = 1.0                                              # b0
    Phi[0::2, 1] = cos_w                                            # b1
    Phi[0::2, 2] = cos_2w                                           # b2
    Phi[0::2, 3] = -(H_real * cos_w + H_imag * sin_w)              # a1 (负号!)
    Phi[0::2, 4] = -(H_real * cos_2w + H_imag * sin_2w)            # a2 (负号!)
    Y[0::2] = H_real

    # 虚部方程 (行 2k+1)
    Phi[1::2, 0] = 0.0                                              # b0
    Phi[1::2, 1] = -sin_w                                           # b1 (负号!)
    Phi[1::2, 2] = -sin_2w                                          # b2 (负号!)
    Phi[1::2, 3] = H_imag * cos_w - H_real * sin_w                 # a1
    Phi[1::2, 4] = H_imag * cos_2w - H_real * sin_2w               # a2
    Y[1::2] = -H_imag

    # 最小二乘解: theta = (Phi^T * Phi)^(-1) * Phi^T * Y
    PhiT_Phi = Phi.T @ Phi
    PhiT_Y = Phi.T @ Y

    try:
        theta = np.linalg.solve(PhiT_Phi, PhiT_Y)
    except np.linalg.LinAlgError:
        print("[ERROR] 初始矩阵奇异! 尝试使用正则化...")
        theta = np.linalg.solve(PhiT_Phi + 1e-6 * np.eye(5), PhiT_Y)

    b0, b1, b2, a1, a2 = theta

    if verbose:
        print(f"  初始解: b0={b0:.8f} b1={b1:.8f} b2={b2:.8f} a1={a1:.8f} a2={a2:.8f}")

    # ------------------------------
    # Step 2: 迭代重加权 (SK方法)
    # ------------------------------
    if verbose:
        print(f"\n[Step 2] 迭代重加权最小二乘 (SK方法, max {max_iter} iter)...")

    prev_theta = theta.copy()

    for it in range(max_iter):
        # 计算权重: W_k = 1 / |A(ω_k)|²
        A_real = 1.0 + a1 * cos_w + a2 * cos_2w
        A_imag = a1 * sin_w + a2 * sin_2w
        A_mag_sq = A_real**2 + A_imag**2
        A_mag_sq = np.maximum(A_mag_sq, 1e-10)  # 防止除零
        weights = 1.0 / A_mag_sq

        sqrt_w = np.sqrt(weights)

        # 构建加权方程
        # 乘以 sqrt(w) 到每一行
        Phi_w = Phi.copy()
        Y_w = Y.copy()

        Phi_w[0::2] *= sqrt_w[:, np.newaxis]
        Phi_w[1::2] *= sqrt_w[:, np.newaxis]
        Y_w[0::2] *= sqrt_w
        Y_w[1::2] *= sqrt_w

        # 加权最小二乘
        PhiT_Phi_w = Phi_w.T @ Phi_w
        PhiT_Y_w = Phi_w.T @ Y_w

        try:
            theta = np.linalg.solve(PhiT_Phi_w, PhiT_Y_w)
        except np.linalg.LinAlgError:
            if verbose:
                print(f"  迭代{it+1}: 矩阵奇异, 使用正则化")
            theta = np.linalg.solve(PhiT_Phi_w + 1e-8 * np.eye(5), PhiT_Y_w)

        b0, b1, b2, a1, a2 = theta
        delta = np.sqrt(np.sum((theta - prev_theta) ** 2))

        if verbose and (it < 3 or (it + 1) % 5 == 0 or delta < tol):
            print(f"  Iter {it+1:2d}: b0={b0:.8f} b1={b1:.8f} b2={b2:.8f} "
                  f"a1={a1:.8f} a2={a2:.8f} |Δ|={delta:.2e}")

        if delta < tol:
            if verbose:
                print(f"  收敛于迭代 {it+1} (|Δ| < {tol:.1e})")
            break

        prev_theta = theta.copy()
    else:
        if verbose:
            print(f"  达到最大迭代次数 ({max_iter})")

    # ------------------------------
    # Step 3: 增益归一化 (使用第一个数据点)
    # ------------------------------
    omega_0 = omega[0]
    cos_0 = np.cos(omega_0)
    cos_20 = np.cos(2.0 * omega_0)
    sin_0 = np.sin(omega_0)
    sin_20 = np.sin(2.0 * omega_0)

    num_real = b0 + b1 * cos_0 + b2 * cos_20
    num_imag = -b1 * sin_0 - b2 * sin_20
    den_real = 1.0 + a1 * cos_0 + a2 * cos_20
    den_imag = -a1 * sin_0 - a2 * sin_20

    fit_mag = np.sqrt(num_real**2 + num_imag**2) / np.sqrt(den_real**2 + den_imag**2)
    meas_mag = mag[0]

    if fit_mag > 1e-10 and meas_mag > 0:
        norm_factor = meas_mag / fit_mag
        b0 *= norm_factor
        b1 *= norm_factor
        b2 *= norm_factor
        if verbose:
            print(f"\n[Step 3] 增益归一化: factor={norm_factor:.6f} "
                  f"(meas={meas_mag:.4f}, fit={fit_mag:.4f})")

    return b0, b1, b2, a1, a2


# ============================================================
#  频率响应计算 (从系数推导幅度和相位)
# ============================================================

def compute_frequency_response(b0, b1, b2, a1, a2, freqs, fs):
    """
    从IIR系数数学推导频率响应。

    H(e^(jω)) = (b0 + b1*e^(-jω) + b2*e^(-j2ω)) / (1 + a1*e^(-jω) + a2*e^(-j2ω))

    参数:
      b0,b1,b2,a1,a2: IIR系数
      freqs: 频率数组 (Hz)
      fs: 采样频率 (Hz)

    返回:
      gains_db: 幅度响应 (dB)
      phases_deg: 相位响应 (度)
      H: 复数频率响应
    """
    omega = 2.0 * np.pi * freqs / fs

    cos_w = np.cos(omega)
    cos_2w = np.cos(2.0 * omega)
    sin_w = np.sin(omega)
    sin_2w = np.sin(2.0 * omega)

    # B(e^(jω)) = b0 + b1*e^(-jω) + b2*e^(-j2ω)
    num_real = b0 + b1 * cos_w + b2 * cos_2w
    num_imag = -b1 * sin_w - b2 * sin_2w

    # A(e^(jω)) = 1 + a1*e^(-jω) + a2*e^(-j2ω)
    den_real = 1.0 + a1 * cos_w + a2 * cos_2w
    den_imag = -a1 * sin_w - a2 * sin_2w

    # H = num / den (复数除法)
    den_mag_sq = den_real**2 + den_imag**2

    H_real = (num_real * den_real + num_imag * den_imag) / den_mag_sq
    H_imag = (num_imag * den_real - num_real * den_imag) / den_mag_sq

    # 幅度 (dB) 和相位 (度)
    mag = np.sqrt(H_real**2 + H_imag**2)
    gains_db = 20.0 * np.log10(np.maximum(mag, 1e-15))
    phases_deg = np.degrees(np.arctan2(H_imag, H_real))

    return gains_db, phases_deg, H_real + 1j * H_imag


# ============================================================
#  误差分析
# ============================================================

def analyze_errors(meas_freqs, meas_gains, meas_phases,
                   fit_gains, fit_phases):
    """
    对比测量数据和拟合响应，计算误差统计。
    """
    gain_err = fit_gains - meas_gains
    phase_err = fit_phases - meas_phases

    # 相位误差归一化到 [-180, 180]
    phase_err = np.where(phase_err > 180, phase_err - 360, phase_err)
    phase_err = np.where(phase_err < -180, phase_err + 360, phase_err)

    results = {
        'gain_mae': np.mean(np.abs(gain_err)),
        'gain_rms': np.sqrt(np.mean(gain_err**2)),
        'gain_max': np.max(np.abs(gain_err)),
        'gain_max_freq': meas_freqs[np.argmax(np.abs(gain_err))],
        'phase_mae': np.mean(np.abs(phase_err)),
        'phase_rms': np.sqrt(np.mean(phase_err**2)),
        'phase_max': np.max(np.abs(phase_err)),
        'phase_max_freq': meas_freqs[np.argmax(np.abs(phase_err))],
        'gain_err': gain_err,
        'phase_err': phase_err,
    }

    return results


# ============================================================
#  稳定性检查
# ============================================================

def check_stability(a1, a2):
    """检查IIR滤波器稳定性。极点必须在单位圆内。"""
    # 极点: z² + a1*z + a2 = 0 的根
    discriminant = a1**2 - 4.0 * a2

    if discriminant >= 0:
        p1 = (-a1 + np.sqrt(discriminant)) / 2.0
        p2 = (-a1 - np.sqrt(discriminant)) / 2.0
        poles = [p1, p2]
    else:
        real_part = -a1 / 2.0
        imag_part = np.sqrt(-discriminant) / 2.0
        mag = np.sqrt(real_part**2 + imag_part**2)
        poles = [complex(real_part, imag_part), complex(real_part, -imag_part)]
        discriminant = -1  # 标记为复数

    max_mag = max(abs(p) for p in poles)
    stable = max_mag < 1.0

    return stable, poles, discriminant


# ============================================================
#  绘图
# ============================================================

def plot_results(meas_freqs, meas_gains, meas_phases,
                 fit_gains, fit_phases, errors, fs, output_path=None):
    """绘制幅频和相频对比图。"""
    if not HAS_PLT:
        print("[SKIP] matplotlib不可用，跳过绘图。")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

    # --- 幅度响应 ---
    ax1 = axes[0]
    ax1.plot(meas_freqs / 1000, meas_gains, 'b.-', label='Measured (Sweep)',
             markersize=3, linewidth=0.8, alpha=0.8)
    ax1.plot(meas_freqs / 1000, fit_gains, 'r-', label='Fitted (IIR Model)',
             linewidth=1.5)
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title(f'IIR Filter Frequency Response Fit (fs={fs/1000:.0f} kHz)')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)

    # --- 相位响应 ---
    ax2 = axes[1]
    ax2.plot(meas_freqs / 1000, meas_phases, 'b.-', label='Measured (Sweep)',
             markersize=3, linewidth=0.8, alpha=0.8)
    ax2.plot(meas_freqs / 1000, fit_phases, 'r-', label='Fitted (IIR Model)',
             linewidth=1.5)
    ax2.set_ylabel('Phase (deg)')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)

    # --- 误差 ---
    ax3 = axes[2]
    ax3.plot(meas_freqs / 1000, errors['gain_err'], 'b-',
             label=f'Gain Error (MAE={errors["gain_mae"]:.3f} dB, RMS={errors["gain_rms"]:.3f} dB)',
             linewidth=1.0)
    ax3.plot(meas_freqs / 1000, errors['phase_err'], 'r-',
             label=f'Phase Error (MAE={errors["phase_mae"]:.2f}°, RMS={errors["phase_rms"]:.2f}°)',
             linewidth=1.0)
    ax3.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax3.set_xlabel('Frequency (kHz)')
    ax3.set_ylabel('Error')
    ax3.legend(loc='best')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\n[绘图] 已保存到: {output_path}")

    try:
        plt.show()
    except:
        print("[绘图] 无法显示窗口，请使用输出图片文件查看。")


# ============================================================
#  主程序
# ============================================================

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("用法: python iir_fit_verify.py <数据文件.txt> [采样频率Hz] [输出图片]")
        print("示例: python iir_fit_verify.py sweep_data.txt 150000 fit_result.png")
        sys.exit(1)

    data_file = sys.argv[1]
    fs = float(sys.argv[2]) if len(sys.argv) > 2 else 150000.0
    output_img = sys.argv[3] if len(sys.argv) > 3 else None

    if not os.path.exists(data_file):
        print(f"[ERROR] 文件不存在: {data_file}")
        sys.exit(1)

    print("=" * 60)
    print("  IIR滤波器系数拟合与验证工具")
    print("=" * 60)
    print(f"数据文件: {data_file}")
    print(f"采样频率: {fs:.0f} Hz")

    # --------------------------------------------------
    # 1. 解析数据
    # --------------------------------------------------
    freqs, gains_db, phases_deg = parse_sweep_data(data_file)
    N = len(freqs)

    print(f"频率范围: {freqs[0]:.1f} ~ {freqs[-1]:.1f} Hz")
    print(f"频率步长: ~{freqs[1] - freqs[0]:.1f} Hz")
    print(f"数据点数: {N}")
    print(f"增益范围: {np.min(gains_db):.2f} ~ {np.max(gains_db):.2f} dB")
    print(f"相位范围: {np.min(phases_deg):.2f} ~ {np.max(phases_deg):.2f} deg")

    # --------------------------------------------------
    # 2. 拟合IIR系数
    # --------------------------------------------------
    b0, b1, b2, a1, a2 = fit_iir_coefficients(freqs, gains_db, phases_deg, fs)

    # --------------------------------------------------
    # 3. 计算全频段响应 (1~50kHz, step 200Hz)
    # --------------------------------------------------
    print(f"\n[Step 4] 计算全频段频率响应...")
    verify_freqs = np.arange(1000, 50001, 200)  # 1k ~ 50k, step 200Hz
    fit_gains, fit_phases, H = compute_frequency_response(b0, b1, b2, a1, a2, verify_freqs, fs)

    # 插值测量数据到验证频率点 (如果测量频率不完全匹配)
    meas_gains_interp = np.interp(verify_freqs, freqs, gains_db)
    meas_phases_interp = np.interp(verify_freqs, freqs, phases_deg)

    # --------------------------------------------------
    # 4. 误差分析
    # --------------------------------------------------
    print(f"\n[Step 5] 误差分析...")
    errors = analyze_errors(verify_freqs, meas_gains_interp, meas_phases_interp,
                            fit_gains, fit_phases)

    # --------------------------------------------------
    # 5. 稳定性检查
    # --------------------------------------------------
    stable, poles, disc = check_stability(a1, a2)
    print(f"\n===== 稳定性检查 =====")
    if disc >= 0:
        print(f"极点 (实数): {poles[0]:.6f}, {poles[1]:.6f}")
    else:
        p = poles[0]
        print(f"极点 (复数): {p.real:.6f} +/- j{abs(p.imag):.6f} (|p|={abs(p):.6f})")

    if stable:
        print(f"滤波器稳定 ✓ (|p_max| = {max(abs(p) for p in poles):.6f} < 1)")
    else:
        print(f"*** 警告: 滤波器不稳定! ✗ (|p_max| = {max(abs(p) for p in poles):.6f} >= 1) ***")

    # --------------------------------------------------
    # 6. 逐频率点对比表
    # --------------------------------------------------
    print(f"\n===== 频率响应对比 (每10个点) =====")
    print(f"{'Freq(Hz)':>8s} | {'Meas(dB)':>10s} {'Fit(dB)':>10s} {'Δ(dB)':>10s} | "
          f"{'Meas(°)':>10s} {'Fit(°)':>10s} {'Δ(°)':>10s}")
    print("-" * 82)

    for k in range(0, len(verify_freqs), 10):
        f = verify_freqs[k]
        mg = meas_gains_interp[k]
        fg = fit_gains[k]
        mp = meas_phases_interp[k]
        fp = fit_phases[k]
        ge = fg - mg
        pe = fp - mp
        # 相位误差归一化
        while pe > 180: pe -= 360
        while pe < -180: pe += 360
        print(f"{f:8.0f} | {mg:+10.4f} {fg:+10.4f} {ge:+10.4f} | "
              f"{mp:+10.3f} {fp:+10.3f} {pe:+10.3f}")

    # 也打印最后一个点
    k = len(verify_freqs) - 1
    if k % 10 != 0:
        f = verify_freqs[k]
        mg = meas_gains_interp[k]
        fg = fit_gains[k]
        mp = meas_phases_interp[k]
        fp = fit_phases[k]
        ge = fg - mg
        pe = fp - mp
        while pe > 180: pe -= 360
        while pe < -180: pe += 360
        print(f"{f:8.0f} | {mg:+10.4f} {fg:+10.4f} {ge:+10.4f} | "
              f"{mp:+10.3f} {fp:+10.3f} {pe:+10.3f}")

    # --------------------------------------------------
    # 7. 误差汇总
    # --------------------------------------------------
    print(f"\n===== 误差分析汇总 =====")
    print(f"增益误差:")
    print(f"  平均绝对误差 (MAE): {errors['gain_mae']:.4f} dB")
    print(f"  均方根误差 (RMS):   {errors['gain_rms']:.4f} dB")
    print(f"  最大绝对误差:       {errors['gain_max']:.4f} dB @ {errors['gain_max_freq']:.0f} Hz")
    print(f"相位误差:")
    print(f"  平均绝对误差 (MAE): {errors['phase_mae']:.4f} deg")
    print(f"  均方根误差 (RMS):   {errors['phase_rms']:.4f} deg")
    print(f"  最大绝对误差:       {errors['phase_max']:.4f} deg @ {errors['phase_max_freq']:.0f} Hz")

    # --------------------------------------------------
    # 8. 最终系数 (可直接复制到C代码)
    # --------------------------------------------------
    print(f"\n===== 最终IIR系数 (可直接复制到C工程) =====")
    print(f"float b0 = {b0:.10f}f;")
    print(f"float b1 = {b1:.10f}f;")
    print(f"float b2 = {b2:.10f}f;")
    print(f"float a1 = {a1:.10f}f;")
    print(f"float a2 = {a2:.10f}f;")
    print(f"")
    print(f"// H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)")
    print(f"// Sample rate: {fs:.0f} Hz")
    print(f"// Filter order: 2 (biquad)")
    print(f"// Fit range: {freqs[0]:.0f} ~ {freqs[-1]:.0f} Hz")

    # --------------------------------------------------
    # 9. 与scipy.invfreqz对比 (如果可用)
    # --------------------------------------------------
    if HAS_SCIPY:
        print(f"\n===== 与scipy.signal.invfreqz对比 =====")
        try:
            mag = 10.0 ** (gains_db / 20.0)
            phase_rad = np.radians(phases_deg)
            H_desired = mag * np.exp(1j * phase_rad)
            omega_scipy = 2.0 * np.pi * freqs / fs

            b_scipy, a_scipy = scipy_signal.invfreqz(H_desired, omega_scipy, 2, 2,
                                                       niter=20, tol=1e-8)
            # invfreqz 返回: b = [b0, b1, b2], a = [1, a1, a2]
            print(f"Scipy invfreqz:")
            print(f"  b = [{b_scipy[0]:.10f}f, {b_scipy[1]:.10f}f, {b_scipy[2]:.10f}f]")
            print(f"  a = [{a_scipy[0]:.10f}f, {a_scipy[1]:.10f}f, {a_scipy[2]:.10f}f]")
            print(f"本工具 (SK方法):")
            print(f"  b = [{b0:.10f}f, {b1:.10f}f, {b2:.10f}f]")
            print(f"  a = [1.0f, {a1:.10f}f, {a2:.10f}f]")

            # 对比
            b_diff = np.max(np.abs(np.array([b0, b1, b2]) - b_scipy))
            a_diff = np.max(np.abs(np.array([a1, a2]) - a_scipy[1:]))
            print(f"  系数最大偏差: b={b_diff:.2e}, a={a_diff:.2e}")

            if b_diff < 1e-4 and a_diff < 1e-4:
                print(f"  ✓ 两种方法结果一致")
            else:
                print(f"  (偏差可能在合理范围内，取决于具体数据和迭代路径)")
        except Exception as e:
            print(f"Scipy对比失败: {e}")

    # --------------------------------------------------
    # 10. 绘图
    # --------------------------------------------------
    plot_results(verify_freqs, meas_gains_interp, meas_phases_interp,
                 fit_gains, fit_phases, errors, fs, output_img)

    print(f"\n{'='*60}")
    print(f"处理完成!")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
