/**
 ****************************************************************************************************
 * @file        iir.c
 * @brief       IIR滤波器实现（双线性变换 + 直接拟合 + 迭代重加权最小二乘）
 * @attention
 *   方法1：给定SecondOrderTF传递函数 + 采样频率，通过双线性变换得到IIR系数
 *   方法2：给定扫频得到BodeData + 采样频率，直接用迭代重加权最小二乘拟合IIR系数
 *
 *   迭代重加权最小二乘 (SK / Sanathanan-Koerner 方法):
 *     初始化: 用方程误差法(Levy)得到初始系数估计
 *     迭代:  W_k = 1/|A(ω_k)|², 加权最小二乘 → 收敛到输出误差最小化
 *     H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 ****************************************************************************************************
 */

#include "./BSP/IIR/my_iir.h"
#include "./SYSTEM/usart/usart.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

// 局部函数声明
static int mat_inv5(double* mat, double* inv);
static void mat_mul(double* A, double* B, double* C, int rowA, int colA, int colB);
static void build_equation_row(double f, double H_real, double H_imag, double fs,
                                double* phi_row_real, double* phi_row_imag,
                                double* y_real, double* y_imag);
static double compute_A_mag_sq(double omega, double a1, double a2);

/**
 * @brief 矩阵乘法
 */
static void mat_mul(double* A, double* B, double* C, int rowA, int colA, int colB)
{
    for (int i = 0; i < rowA; i++)
    {
        for (int j = 0; j < colB; j++)
        {
            C[i * colB + j] = 0.0;
            for (int k = 0; k < colA; k++)
            {
                C[i * colB + j] += A[i * colA + k] * B[k * colB + j];
            }
        }
    }
}

/**
 * @brief 5x5矩阵求逆（高斯-约当消元法）
 */
static int mat_inv5(double* mat, double* inv)
{
    double aug[5][10] = { 0 };
    const int n = 5;
    int i, j, k;

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++) aug[i][j] = mat[i * n + j];
        aug[i][i + n] = 1.0;
    }

    for (i = 0; i < n; i++)
    {
        int pivot = i;
        for (k = i; k < n; k++)
            if (fabs(aug[k][i]) > fabs(aug[pivot][i])) pivot = k;

        if (pivot != i)
        {
            for (j = i; j < 2 * n; j++)
            {
                double t = aug[i][j];
                aug[i][j] = aug[pivot][j];
                aug[pivot][j] = t;
            }
        }

        double div = aug[i][i];
        if (fabs(div) < 1e-12) return -1;

        for (j = i; j < 2 * n; j++) aug[i][j] /= div;
        for (k = 0; k < n; k++)
        {
            if (k != i && fabs(aug[k][i]) > 1e-12)
            {
                double fac = aug[k][i];
                for (j = i; j < 2 * n; j++)
                    aug[k][j] -= fac * aug[i][j];
            }
        }
    }

    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            inv[i * n + j] = aug[i][j + n];
    return 0;
}

/**
 * @brief 计算分母幅值平方 |A(ω)|²
 *        A(z) = 1 + a1*z^-1 + a2*z^-2
 *        |A(e^(jω))|² = (1 + a1*cosω + a2*cos2ω)² + (a1*sinω + a2*sin2ω)²
 */
static double compute_A_mag_sq(double omega, double a1, double a2)
{
    double cos_w = cos(omega);
    double sin_w = sin(omega);
    double cos_2w = cos(2.0 * omega);
    double sin_2w = sin(2.0 * omega);

    double real_part = 1.0 + a1 * cos_w + a2 * cos_2w;
    double imag_part = a1 * sin_w + a2 * sin_2w;

    return real_part * real_part + imag_part * imag_part;
}

/**
 * @brief 构建单行方程 (修正符号错误后的正确版本)
 *
 * 推导: H(z) = B(z)/A(z)
 *   B(z) = b0 + b1*z^-1 + b2*z^-2
 *   A(z) = 1 + a1*z^-1 + a2*z^-2
 *
 * 在 z = e^(jω) 处:
 *   B(e^(jω)) = b0 + b1*cos(ω) + b2*cos(2ω) - j*(b1*sin(ω) + b2*sin(2ω))
 *   A(e^(jω)) = 1 + a1*cos(ω) + a2*cos(2ω) - j*(a1*sin(ω) + a2*sin(2ω))
 *
 * 方程 H*A = B 的实部和虚部:
 *   实部: b0 + b1*cos(ω) + b2*cos(2ω) - a1*(Hr*cos(ω)+Hi*sin(ω)) - a2*(Hr*cos(2ω)+Hi*sin(2ω)) = Hr
 *   虚部: 0 - b1*sin(ω) - b2*sin(2ω) + a1*(Hi*cos(ω)-Hr*sin(ω)) + a2*(Hi*cos(2ω)-Hr*sin(2ω)) = -Hi
 */
static void build_equation_row(double f, double H_real, double H_imag, double fs,
                                double* phi_row_real, double* phi_row_imag,
                                double* y_real, double* y_imag)
{
    double omega = 2.0 * PI * f / fs;
    double cos_w = cos(omega);
    double cos_2w = cos(2.0 * omega);
    double sin_w = sin(omega);
    double sin_2w = sin(2.0 * omega);

    // ---- 实部方程 ----
    // [b0, b1, b2, a1, a2] * [1, cos_w, cos_2w, -(Hr*cos_w+Hi*sin_w), -(Hr*cos_2w+Hi*sin_2w)]^T = Hr
    phi_row_real[0] = 1.0;
    phi_row_real[1] = cos_w;
    phi_row_real[2] = cos_2w;
    phi_row_real[3] = -(H_real * cos_w + H_imag * sin_w);      // !! 修正: 取反
    phi_row_real[4] = -(H_real * cos_2w + H_imag * sin_2w);    // !! 修正: 取反
    *y_real = H_real;

    // ---- 虚部方程 ----
    // [b0, b1, b2, a1, a2] * [0, -sin_w, -sin_2w, (Hi*cos_w-Hr*sin_w), (Hi*cos_2w-Hr*sin_2w)]^T = -Hi
    phi_row_imag[0] = 0.0;
    phi_row_imag[1] = -sin_w;                                   // !! 修正: 取反
    phi_row_imag[2] = -sin_2w;                                  // !! 修正: 取反
    phi_row_imag[3] = H_imag * cos_w - H_real * sin_w;         // 正确
    phi_row_imag[4] = H_imag * cos_2w - H_real * sin_2w;       // 正确
    *y_imag = -H_imag;
}

/**
 * @brief 从扫频数据直接拟合IIR滤波器系数（迭代重加权最小二乘法）
 * @param data: 扫频数据数组（频率、增益dB、相位度）
 * @param cnt: 数据点数量
 * @param fs: ADC采样频率 (Hz)
 * @param filter: 输出的IIR滤波器系数
 * @return 0:成功, -1:失败
 *
 * @note  使用迭代重加权最小二乘 (SK方法) 最小化输出误差:
 *        min Σ |H_desired(ω_k) - B(ω_k)/A(ω_k)|²
 *
 *        算法流程:
 *        1. 方程误差法(Levy)得到初始估计
 *        2. 迭代: W_k = 1/|A_prev(ω_k)|², 加权最小二乘
 *        3. 收敛判断 |θ_new - θ_old| < tol
 */
int IIR_Design_Direct(BodeData* data, int cnt, float fs, IIR_Filter* filter)
{
    #define MAX_EQ_NUM 600
    #define MAX_ITER    20
    #define CONV_TOL    1e-6

    /* 全部使用静态数组，避免STM32上malloc死机 */
    static double Phi_static[MAX_EQ_NUM * 5];
    static double Y_static[MAX_EQ_NUM];
    static double PhiT_static[5 * MAX_EQ_NUM];
    static double weights[MAX_EQ_NUM / 2];
    static double Hr_buf[MAX_EQ_NUM / 2];      // 实部缓冲
    static double Hi_buf[MAX_EQ_NUM / 2];      // 虚部缓冲
    static double omega_buf[MAX_EQ_NUM / 2];   // 数字频率缓冲

    if (cnt < 5 || data == NULL || filter == NULL || fs <= 0)
        return -1;

    printf("\r\n===== IIR Filter Design (IRLS / SK Method) =====\r\n");
    printf("Data points: %d, Sample frequency: %.0f Hz\r\n", cnt, fs);

    const int eq_num = 2 * cnt;
    const int para_n = 5;
    const int n_freqs = cnt;

    if (eq_num > MAX_EQ_NUM) {
        printf("ERROR: Too many equations! Max=%d, need=%d\r\n", MAX_EQ_NUM, eq_num);
        return -1;
    }

    // ============================================================
    //  Step 1: 提取频域数据 (H_desired at each frequency)
    // ============================================================
    printf("Step 1: Extracting frequency response data...\r\n");

    double* Hr = Hr_buf;
    double* Hi = Hi_buf;
    double* omega_arr = omega_buf;

    for (int k = 0; k < cnt; k++)
    {
        double f = data[k].freq;
        double g_db = data[k].gain_db;
        double p_deg = data[k].phase_deg;

        double mag = pow(10.0, g_db / 20.0);
        double phase_rad = p_deg * PI / 180.0;

        Hr[k] = mag * cos(phase_rad);
        Hi[k] = mag * sin(phase_rad);
        omega_arr[k] = 2.0 * PI * f / fs;
    }
    printf("Step 1 done (Hr/Hi/omega arrays ready)\r\n");

    // ============================================================
    //  Step 2: 方程误差法 (Levy) 初始估计 (所有权重=1)
    // ============================================================
    printf("Step 2: Equation-error initial estimate (Levy method)...\r\n");

    memset(Phi_static, 0, eq_num * para_n * sizeof(double));
    memset(Y_static, 0, eq_num * sizeof(double));
    memset(weights, 0, n_freqs * sizeof(double));

    for (int k = 0; k < cnt; k++)
    {
        weights[k] = 1.0;  // 初始权重 = 1

        double phi_real[5], phi_imag[5];
        double yr, yi;

        build_equation_row(data[k].freq, Hr[k], Hi[k], fs,
                           phi_real, phi_imag, &yr, &yi);

        // 实部方程 (行 2k)
        int r1 = 2 * k;
        for (int j = 0; j < 5; j++) Phi_static[r1 * 5 + j] = phi_real[j];
        Y_static[r1] = yr;

        // 虚部方程 (行 2k+1)
        int r2 = 2 * k + 1;
        for (int j = 0; j < 5; j++) Phi_static[r2 * 5 + j] = phi_imag[j];
        Y_static[r2] = yi;
    }

    // PhiT = Phi^T
    for (int i = 0; i < para_n; i++)
        for (int j = 0; j < eq_num; j++)
            PhiT_static[i * eq_num + j] = Phi_static[j * para_n + i];

    double PhiT_Phi[25] = { 0.0 };
    mat_mul(PhiT_static, Phi_static, PhiT_Phi, para_n, eq_num, para_n);

    // Light Tikhonov regularization for numerical stability only
    // (weight clamping handles Q control, not regularization)
    double lambda_init = 1e-3;
    PhiT_Phi[0] += lambda_init;
    PhiT_Phi[6] += lambda_init;
    PhiT_Phi[12] += lambda_init;
    PhiT_Phi[18] += lambda_init;
    PhiT_Phi[24] += lambda_init;

    double inv_PhiT_Phi[25] = { 0.0 };
    if (mat_inv5(PhiT_Phi, inv_PhiT_Phi) != 0) {
        printf("ERROR: Matrix inversion failed in initial step!\r\n");
        return -1;
    }

    double PhiT_Y[5] = { 0.0 };
    mat_mul(PhiT_static, Y_static, PhiT_Y, para_n, eq_num, 1);

    double theta[5] = { 0.0 };
    mat_mul(inv_PhiT_Phi, PhiT_Y, theta, para_n, para_n, 1);

    // 提取初始系数
    double b0 = theta[0];
    double b1 = theta[1];
    double b2 = theta[2];
    double a1 = theta[3];
    double a2 = theta[4];

    printf("  Initial: b0=%.6f b1=%.6f b2=%.6f a1=%.6f a2=%.6f\r\n", b0, b1, b2, a1, a2);

    // ============================================================
    //  Step 3: 迭代重加权最小二乘 (SK method)
    // ============================================================
    printf("Step 3: Iterative reweighted LS (SK method)...\r\n");

    double prev_theta[5];
    int converged = 0;

    for (int iter = 0; iter < MAX_ITER; iter++)
    {
        // 保存上一次的系数
        prev_theta[0] = b0; prev_theta[1] = b1; prev_theta[2] = b2;
        prev_theta[3] = a1;  prev_theta[4] = a2;

        // 更新权重: W_k = 1 / |A_prev(ω_k)|²
        // 但需要防止除零，加一个小常数
        for (int k = 0; k < cnt; k++)
        {
            double A_mag_sq = compute_A_mag_sq(omega_arr[k], a1, a2);
            if (A_mag_sq < 1e-8) A_mag_sq = 1e-8;
            weights[k] = 1.0 / A_mag_sq;
            // Clamp weights: prevent resonance from dominating.
            // Natural resonance weight ~4e5; clamp at 100 forces broader fit.
            if (weights[k] > 100.0) weights[k] = 100.0;
        }

        // 构建加权方程: sqrt(W) * Phi * θ = sqrt(W) * Y
        memset(Phi_static, 0, eq_num * para_n * sizeof(double));
        memset(Y_static, 0, eq_num * sizeof(double));

        for (int k = 0; k < cnt; k++)
        {
            double sqrt_w = sqrt(weights[k]);

            double phi_real[5], phi_imag[5];
            double yr, yi;

            build_equation_row(data[k].freq, Hr[k], Hi[k], fs,
                               phi_real, phi_imag, &yr, &yi);

            // 加权实部方程
            int r1 = 2 * k;
            for (int j = 0; j < 5; j++) Phi_static[r1 * 5 + j] = sqrt_w * phi_real[j];
            Y_static[r1] = sqrt_w * yr;

            // 加权虚部方程
            int r2 = 2 * k + 1;
            for (int j = 0; j < 5; j++) Phi_static[r2 * 5 + j] = sqrt_w * phi_imag[j];
            Y_static[r2] = sqrt_w * yi;
        }

        // PhiT = Phi^T
        for (int i = 0; i < para_n; i++)
            for (int j = 0; j < eq_num; j++)
                PhiT_static[i * eq_num + j] = Phi_static[j * para_n + i];

        // PhiT_Phi = PhiT * Phi
        memset(PhiT_Phi, 0, 25 * sizeof(double));
        mat_mul(PhiT_static, Phi_static, PhiT_Phi, para_n, eq_num, para_n);

        // Minimal regularization in IRLS: weight clamping already limits Q
        PhiT_Phi[0] += 1e-6;
        PhiT_Phi[6] += 1e-6;
        PhiT_Phi[12] += 1e-6;
        PhiT_Phi[18] += 1e-6;
        PhiT_Phi[24] += 1e-6;

        // inv(PhiT_Phi)
        memset(inv_PhiT_Phi, 0, 25 * sizeof(double));
        if (mat_inv5(PhiT_Phi, inv_PhiT_Phi) != 0) {
            printf("  WARNING: Matrix singular at iter %d, using previous result\r\n", iter);
            b0 = prev_theta[0]; b1 = prev_theta[1]; b2 = prev_theta[2];
            a1 = prev_theta[3]; a2 = prev_theta[4];
            break;
        }

        // PhiT_Y = PhiT * Y
        memset(PhiT_Y, 0, 5 * sizeof(double));
        mat_mul(PhiT_static, Y_static, PhiT_Y, para_n, eq_num, 1);

        // theta = inv(PhiT_Phi) * PhiT_Y
        memset(theta, 0, 5 * sizeof(double));
        mat_mul(inv_PhiT_Phi, PhiT_Y, theta, para_n, para_n, 1);

        b0 = theta[0];
        b1 = theta[1];
        b2 = theta[2];
        a1 = theta[3];
        a2 = theta[4];

        // 检查收敛
        double delta = 0.0;
        for (int j = 0; j < 5; j++)
        {
            double d = theta[j] - prev_theta[j];
            delta += d * d;
        }
        delta = sqrt(delta);

        if (iter < 3 || (iter + 1) % 5 == 0 || delta < CONV_TOL) {
            printf("  Iter %2d: b0=%.6f b1=%.6f b2=%.6f a1=%.6f a2=%.6f (|Δ|=%.2e)\r\n",
                   iter + 1, b0, b1, b2, a1, a2, delta);
        }

        if (delta < CONV_TOL) {
            printf("  Converged at iteration %d (|Δ| < %.1e)\r\n", iter + 1, CONV_TOL);
            converged = 1;
            break;
        }
    }

    if (!converged) {
        printf("  Reached max iterations (%d)\r\n", MAX_ITER);
    }

    // ============================================================
    //  Step 4: 系数赋给滤波器结构
    // ============================================================
    filter->b0 = (float)b0;
    filter->b1 = (float)b1;
    filter->b2 = (float)b2;
    filter->a1 = (float)a1;
    filter->a2 = (float)a2;

    // ============================================================
    //  Step 5: DC增益归一化 (如果测量数据包含DC附近数据)
    // ============================================================
    // 注意: 只有当数据从很低频率开始时才做DC归一化
    // 扫频从1kHz开始，所以这里使用第一个数据点而不是DC
    float ref_gain_measured = pow(10.0, data[0].gain_db / 20.0);
    double omega_ref = 2.0 * PI * data[0].freq / fs;
    double cos_w_ref = cos(omega_ref);
    double cos_2w_ref = cos(2.0 * omega_ref);
    double sin_w_ref = sin(omega_ref);
    double sin_2w_ref = sin(2.0 * omega_ref);

    double num_real_ref = b0 + b1 * cos_w_ref + b2 * cos_2w_ref;
    double num_imag_ref = -b1 * sin_w_ref - b2 * sin_2w_ref;
    double den_real_ref = 1.0 + a1 * cos_w_ref + a2 * cos_2w_ref;
    double den_imag_ref = -a1 * sin_w_ref - a2 * sin_2w_ref;
    double ref_gain_current = sqrt(num_real_ref*num_real_ref + num_imag_ref*num_imag_ref) /
                               sqrt(den_real_ref*den_real_ref + den_imag_ref*den_imag_ref);

    printf("\r\nStep 4: Gain normalization...\r\n");
    printf("  Measured gain at %.0f Hz: %.4f (%.2f dB)\r\n",
           data[0].freq, ref_gain_measured, data[0].gain_db);
    printf("  Fitted  gain at %.0f Hz: %.4f (%.2f dB)\r\n",
           data[0].freq, ref_gain_current, 20.0 * log10(ref_gain_current));

    if (fabs(ref_gain_current) > 0.0001 && ref_gain_measured > 0) {
        float norm_factor = ref_gain_measured / ref_gain_current;
        filter->b0 *= norm_factor;
        filter->b1 *= norm_factor;
        filter->b2 *= norm_factor;
        printf("  Normalization factor: %.4f\r\n", norm_factor);
        b0 = filter->b0; b1 = filter->b1; b2 = filter->b2;  // 更新局部变量
    }

    IIR_Reset(filter);

    // ============================================================
    //  Step 6: 稳定性检查
    // ============================================================
    printf("\r\n===== Stability Check =====\r\n");
    double discriminant = (double)filter->a1 * filter->a1 - 4.0 * filter->a2;
    if (discriminant >= 0.0) {
        double p1 = (-(double)filter->a1 + sqrt(discriminant)) / 2.0;
        double p2 = (-(double)filter->a1 - sqrt(discriminant)) / 2.0;
        printf("Poles: %.6f, %.6f\r\n", p1, p2);
        if (fabs(p1) >= 1.0 || fabs(p2) >= 1.0) {
            printf("*** WARNING: UNSTABLE FILTER! ***\r\n");
        } else {
            printf("Stable (|p| < 1)\r\n");
        }
    } else {
        double real_part = -(double)filter->a1 / 2.0;
        double imag_part = sqrt(-discriminant) / 2.0;
        double mag = sqrt(real_part*real_part + imag_part*imag_part);
        printf("Poles: %.6f +/- j%.6f (mag=%.6f)\r\n", real_part, imag_part, mag);
        if (mag >= 1.0) {
            printf("*** WARNING: UNSTABLE FILTER! ***\r\n");
        } else {
            printf("Stable (|p| < 1)\r\n");
        }
    }

    // ============================================================
    //  Step 7: 全频段频率响应验证及误差分析
    // ============================================================
    printf("\r\n===== Full Frequency Response Verification =====\r\n");
    printf("Freq(Hz)   | MeasGain(dB)  FitGain(dB)  ΔGain(dB) | MeasPh(deg)  FitPh(deg)  ΔPh(deg)\r\n");
    printf("-----------+------------------------------------------------------+---------------------------\r\n");

    double sum_gain_err = 0.0, sum_phase_err = 0.0;
    double max_gain_err = 0.0, max_phase_err = 0.0;
    int max_gain_idx = 0, max_phase_idx = 0;
    double sum_gain_err_sq = 0.0, sum_phase_err_sq = 0.0;

    for (int k = 0; k < cnt; k++)
    {
        double omega = 2.0 * PI * data[k].freq / fs;
        double cos_w = cos(omega);
        double cos_2w = cos(2.0 * omega);
        double sin_w = sin(omega);
        double sin_2w = sin(2.0 * omega);

        // 计算 H(e^(jω)) = B(e^(jω)) / A(e^(jω))
        double num_real = b0 + b1 * cos_w + b2 * cos_2w;
        double num_imag = -b1 * sin_w - b2 * sin_2w;
        double den_real = 1.0 + a1 * cos_w + a2 * cos_2w;
        double den_imag = -a1 * sin_w - a2 * sin_2w;

        // 复数除法
        double den_mag_sq = den_real * den_real + den_imag * den_imag;
        double H_fit_real = (num_real * den_real + num_imag * den_imag) / den_mag_sq;
        double H_fit_imag = (num_imag * den_real - num_real * den_imag) / den_mag_sq;

        // 拟合的幅值和相位
        double fit_mag = sqrt(H_fit_real * H_fit_real + H_fit_imag * H_fit_imag);
        double fit_gain_db = 20.0 * log10(fit_mag > 1e-15 ? fit_mag : 1e-15);
        double fit_phase_deg = atan2(H_fit_imag, H_fit_real) * 180.0 / PI;

        // 测量的幅值和相位
        double meas_gain_db = data[k].gain_db;
        double meas_phase_deg = data[k].phase_deg;

        // 误差
        double gain_err = fit_gain_db - meas_gain_db;
        double phase_err = fit_phase_deg - meas_phase_deg;

        // 相位误差归一化到 [-180, 180]
        while (phase_err > 180.0) phase_err -= 360.0;
        while (phase_err < -180.0) phase_err += 360.0;

        sum_gain_err += fabs(gain_err);
        sum_phase_err += fabs(phase_err);
        sum_gain_err_sq += gain_err * gain_err;
        sum_phase_err_sq += phase_err * phase_err;

        if (fabs(gain_err) > max_gain_err) {
            max_gain_err = fabs(gain_err);
            max_gain_idx = k;
        }
        if (fabs(phase_err) > max_phase_err) {
            max_phase_err = fabs(phase_err);
            max_phase_idx = k;
        }

        // 打印每一个点的对比 (为节省串口输出，每5个点打一次详细信息)
        if (k % 5 == 0 || k == cnt - 1 || fabs(gain_err) > 1.0 || fabs(phase_err) > 5.0) {
            printf("%8.0f Hz | %+8.3f dB  %+8.3f dB  %+8.3f dB | %+8.2f deg %+8.2f deg %+8.2f deg\r\n",
                   data[k].freq,
                   meas_gain_db, fit_gain_db, gain_err,
                   meas_phase_deg, fit_phase_deg, phase_err);
        }
    }

    // ============================================================
    //  Step 8: 误差统计汇总
    // ============================================================
    printf("\r\n===== Error Analysis Summary =====\r\n");
    printf("Gain Error:\r\n");
    printf("  Mean Absolute Error: %.4f dB\r\n", sum_gain_err / cnt);
    printf("  RMS Error:           %.4f dB\r\n", sqrt(sum_gain_err_sq / cnt));
    printf("  Max Absolute Error:  %.4f dB at %.0f Hz\r\n", max_gain_err, data[max_gain_idx].freq);
    printf("Phase Error:\r\n");
    printf("  Mean Absolute Error: %.4f deg\r\n", sum_phase_err / cnt);
    printf("  RMS Error:           %.4f deg\r\n", sqrt(sum_phase_err_sq / cnt));
    printf("  Max Absolute Error:  %.4f deg at %.0f Hz\r\n", max_phase_err, data[max_phase_idx].freq);

    // 打印最终系数（保持原有格式，可直接复制到其他工程使用）
    printf("\r\n===== Final IIR Coefficients (copy-ready) =====\r\n");
    printf("float b0 = %.10ff;\r\n", filter->b0);
    printf("float b1 = %.10ff;\r\n", filter->b1);
    printf("float b2 = %.10ff;\r\n", filter->b2);
    printf("float a1 = %.10ff;\r\n", filter->a1);
    printf("float a2 = %.10ff;\r\n", filter->a2);
    printf("\r\n// H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)\r\n");
    printf("// Sample rate: %.0f Hz\r\n", fs);
    printf("===== IIR Design Complete =====\r\n\r\n");

    return 0;
}

/**
 * @brief 四阶IIR设计：顺序级联两个biquad
 * @param data: 扫频数据
 * @param cnt: 数据点数
 * @param fs: 采样频率
 * @param sos: 输出的级联biquad系数
 * @return 0:成功, -1:失败
 *
 * 算法: 
 *   1. 拟合第一个biquad捕捉主要频响特征
 *   2. 计算残差 H_residual = H_measured / H_bq1
 *   3. 拟合第二个biquad修正残差
 *   4. 级联 = 四阶IIR
 */
/* ===== General NxN matrix inversion (Gauss-Jordan) ===== */
static int mat_inv_n(double* mat, double* inv, int n)
{
    #define N_MAX 12
    double aug[N_MAX][2*N_MAX];
    int i, j, k;

    if (n > N_MAX) return -1;
    memset(aug, 0, sizeof(aug));

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) aug[i][j] = mat[i*n + j];
        aug[i][i + n] = 1.0;
    }

    for (i = 0; i < n; i++) {
        int pivot = i;
        for (k = i; k < n; k++)
            if (fabs(aug[k][i]) > fabs(aug[pivot][i])) pivot = k;

        if (fabs(aug[pivot][i]) < 1e-14) return -1;

        if (pivot != i)
            for (j = i; j < 2*n; j++) {
                double t = aug[i][j]; aug[i][j] = aug[pivot][j]; aug[pivot][j] = t;
            }

        double div = aug[i][i];
        for (j = i; j < 2*n; j++) aug[i][j] /= div;

        for (k = 0; k < n; k++) {
            if (k == i) continue;
            double fac = aug[k][i];
            if (fabs(fac) < 1e-14) continue;
            for (j = i; j < 2*n; j++) aug[k][j] -= fac * aug[i][j];
        }
    }

    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            inv[i*n + j] = aug[i][j + n];
    return 0;
}

/* ===== Build equation row for order N ===== */
static void build_eq_row_n(double f, double Hr, double Hi, double fs, int order,
                            double* phi_r, double* phi_i, double* yr, double* yi)
{
    double omega = 2.0 * PI * f / fs;
    int nb = order + 1;  // b0..b_order
    int na = order;      // a1..a_order
    int n = nb + na;     // total params

    for (int m = 0; m < n; m++) { phi_r[m] = 0.0; phi_i[m] = 0.0; }

    // Real: b0 + Σb_m*cos(mω) - Σa_m*(Hr*cos(mω)+Hi*sin(mω)) = Hr
    phi_r[0] = 1.0;  // b0
    for (int m = 1; m <= order; m++) {
        double cm = cos(m * omega), sm = sin(m * omega);
        phi_r[m] = cm;                          // b_m
        phi_r[nb + m - 1] = -(Hr * cm + Hi * sm);  // a_m
    }
    *yr = Hr;

    // Imag: -Σb_m*sin(mω) + Σa_m*(Hi*cos(mω)-Hr*sin(mω)) = -Hi
    for (int m = 1; m <= order; m++) {
        double cm = cos(m * omega), sm = sin(m * omega);
        phi_i[m] = -sm;                         // b_m
        phi_i[nb + m - 1] = Hi * cm - Hr * sm;  // a_m
    }
    *yi = -Hi;
}

/**
 * @brief 直接拟合N阶IIR (基于IRLS/SK方法)
 * @return 0成功
 */
static int iir_fit_order(BodeData* data, int cnt, float fs, int order,
                          double* b_coeff, double* a_coeff, int verbose)
{
    int nb = order + 1;
    int na = order;
    int para_n = nb + na;
    int eq_num = 2 * cnt;

    #define FIT_MAX_PTS  300
    #define FIT_MAX_EQ   (2*FIT_MAX_PTS)
    #define FIT_MAX_PAR  12

    static double Phi_s[FIT_MAX_EQ * FIT_MAX_PAR];
    static double Y_s[FIT_MAX_EQ];
    static double PhiT_s[FIT_MAX_PAR * FIT_MAX_EQ];
    static double weights_s[FIT_MAX_PTS];
    static double Hr_s[FIT_MAX_PTS], Hi_s[FIT_MAX_PTS], omg_s[FIT_MAX_PTS];

    if (cnt > FIT_MAX_PTS || para_n > FIT_MAX_PAR) return -1;

    /* Step 1: Extract frequency response */
    for (int k = 0; k < cnt; k++) {
        double mag = pow(10.0, data[k].gain_db / 20.0);
        double ph = data[k].phase_deg * PI / 180.0;
        Hr_s[k] = mag * cos(ph);
        Hi_s[k] = mag * sin(ph);
        omg_s[k] = 2.0 * PI * data[k].freq / fs;
    }

    /* Step 2: Initial estimate (Levy, weights=1) */
    memset(Phi_s, 0, eq_num * para_n * sizeof(double));
    memset(Y_s, 0, eq_num * sizeof(double));

    for (int k = 0; k < cnt; k++) {
        double pr[FIT_MAX_PAR], pi[FIT_MAX_PAR], yr, yi;
        build_eq_row_n(data[k].freq, Hr_s[k], Hi_s[k], fs, order, pr, pi, &yr, &yi);
        int r1 = 2*k, r2 = 2*k+1;
        for (int j = 0; j < para_n; j++) { Phi_s[r1*para_n + j] = pr[j]; Phi_s[r2*para_n + j] = pi[j]; }
        Y_s[r1] = yr; Y_s[r2] = yi;
    }

    for (int i = 0; i < para_n; i++)
        for (int j = 0; j < eq_num; j++)
            PhiT_s[i*eq_num + j] = Phi_s[j*para_n + i];

    double PTP[FIT_MAX_PAR * FIT_MAX_PAR];
    mat_mul(PhiT_s, Phi_s, PTP, para_n, eq_num, para_n);
    // Light regularization
    for (int i = 0; i < para_n; i++) PTP[i*para_n + i] += 1e-3;

    double inv_PTP[FIT_MAX_PAR * FIT_MAX_PAR];
    if (mat_inv_n(PTP, inv_PTP, para_n) != 0) return -1;

    double PTY[FIT_MAX_PAR];
    mat_mul(PhiT_s, Y_s, PTY, para_n, eq_num, 1);

    double theta[FIT_MAX_PAR];
    mat_mul(inv_PTP, PTY, theta, para_n, para_n, 1);

    for (int j = 0; j < nb; j++) b_coeff[j] = theta[j];
    for (int j = 0; j < na; j++) a_coeff[j] = theta[nb + j];

    if (verbose) {
        printf("  Initial: b=[%.4f %.4f %.4f %.4f %.4f] a=[%.4f %.4f %.4f %.4f]\r\n",
               b_coeff[0],b_coeff[1],b_coeff[2],b_coeff[3],b_coeff[4],
               a_coeff[0],a_coeff[1],a_coeff[2],a_coeff[3]);
    }

    /* Step 3: IRLS iterations */
    double prev_theta[FIT_MAX_PAR];
    for (int iter = 0; iter < 20; iter++) {
        memcpy(prev_theta, theta, para_n * sizeof(double));

        // Update weights
        for (int k = 0; k < cnt; k++) {
            double A_mag_sq = 1.0;
            double cw = cos(omg_s[k]), sw = sin(omg_s[k]);
            double Are = 1.0, Aim = 0.0;
            for (int m = 1; m <= order; m++) {
                double cm = cos(m*omg_s[k]), sm = sin(m*omg_s[k]);
                Are += a_coeff[m-1] * cm;
                Aim -= a_coeff[m-1] * sm;
            }
            A_mag_sq = Are*Are + Aim*Aim;
            if (A_mag_sq < 1e-10) A_mag_sq = 1e-10;
            weights_s[k] = 1.0 / A_mag_sq;
            if (weights_s[k] > 100.0) weights_s[k] = 100.0;
        }

        // Build weighted equations
        memset(Phi_s, 0, eq_num * para_n * sizeof(double));
        memset(Y_s, 0, eq_num * sizeof(double));
        for (int k = 0; k < cnt; k++) {
            double sw = sqrt(weights_s[k]);
            double pr[FIT_MAX_PAR], pi[FIT_MAX_PAR], yr, yi;
            build_eq_row_n(data[k].freq, Hr_s[k], Hi_s[k], fs, order, pr, pi, &yr, &yi);
            int r1 = 2*k, r2 = 2*k+1;
            for (int j = 0; j < para_n; j++) {
                Phi_s[r1*para_n + j] = sw * pr[j];
                Phi_s[r2*para_n + j] = sw * pi[j];
            }
            Y_s[r1] = sw * yr; Y_s[r2] = sw * yi;
        }

        for (int i = 0; i < para_n; i++)
            for (int j = 0; j < eq_num; j++)
                PhiT_s[i*eq_num + j] = Phi_s[j*para_n + i];

        mat_mul(PhiT_s, Phi_s, PTP, para_n, eq_num, para_n);
        for (int i = 0; i < para_n; i++) PTP[i*para_n + i] += 1e-6;

        if (mat_inv_n(PTP, inv_PTP, para_n) != 0) break;

        mat_mul(PhiT_s, Y_s, PTY, para_n, eq_num, 1);
        mat_mul(inv_PTP, PTY, theta, para_n, para_n, 1);

        for (int j = 0; j < nb; j++) b_coeff[j] = theta[j];
        for (int j = 0; j < na; j++) a_coeff[j] = theta[nb + j];

        double delta = 0;
        for (int j = 0; j < para_n; j++) { double d = theta[j]-prev_theta[j]; delta += d*d; }
        delta = sqrt(delta);

        if (verbose && (iter < 3 || delta < 1e-6)) {
            printf("  Iter %d: |Δ|=%.2e\r\n", iter+1, delta);
        }
        if (delta < 1e-6) break;
    }

    return 0;
}

/**
 * @brief 四阶IIR设计：直接拟合4阶传递函数
 */
int IIR_Design_Direct_4th(BodeData* data, int cnt, float fs, IIR_SOS* sos)
{
    if (cnt < 9 || data == NULL || sos == NULL || fs <= 0)
        return -1;

    printf("\r\n===== 4th-Order IIR Design (Direct Fit) =====\r\n");
    printf("Data points: %d, Sample rate: %.0f Hz\r\n", cnt, fs);

    double b[5], a[4];
    if (iir_fit_order(data, cnt, fs, 4, b, a, 1) != 0) {
        printf("ERROR: 4th-order fitting failed!\r\n");
        return -1;
    }

    printf("\r\n===== 4th-Order IIR Coefficients =====\r\n");
    printf("// Direct Form I (4th order)\r\n");
    printf("float b0 = %.10ff;\r\n", (float)b[0]);
    printf("float b1 = %.10ff;\r\n", (float)b[1]);
    printf("float b2 = %.10ff;\r\n", (float)b[2]);
    printf("float b3 = %.10ff;\r\n", (float)b[3]);
    printf("float b4 = %.10ff;\r\n", (float)b[4]);
    printf("float a1 = %.10ff;\r\n", (float)a[0]);
    printf("float a2 = %.10ff;\r\n", (float)a[1]);
    printf("float a3 = %.10ff;\r\n", (float)a[2]);
    printf("float a4 = %.10ff;\r\n", (float)a[3]);
    printf("// y[n] = b0*x[n] + ... + b4*x[n-4] - a1*y[n-1] - ... - a4*y[n-4]\r\n");

    /* Verification */
    printf("\r\n===== 4th-Order Verification =====\r\n");
    double sum_ge=0, sum_pe=0, sum_g2=0, sum_p2=0, max_ge=0, max_pe=0;

    for (int k = 0; k < cnt; k++) {
        double omega = 2.0 * PI * data[k].freq / fs;

        // B(e^(jω))
        double Br = b[0], Bi = 0;
        for (int m = 1; m <= 4; m++) {
            Br += b[m] * cos(m * omega);
            Bi -= b[m] * sin(m * omega);
        }
        // A(e^(jω))
        double Ar = 1.0, Ai = 0;
        for (int m = 1; m <= 4; m++) {
            Ar += a[m-1] * cos(m * omega);
            Ai -= a[m-1] * sin(m * omega);
        }
        double den = Ar*Ar + Ai*Ai;
        double Hr = (Br*Ar + Bi*Ai) / den;
        double Hi = (Bi*Ar - Br*Ai) / den;
        double Hmag = sqrt(Hr*Hr + Hi*Hi);
        double Hph = atan2(Hi, Hr) * 180.0 / PI;

        double fg = 20.0 * log10(Hmag > 1e-15 ? Hmag : 1e-15);
        double ge = fg - data[k].gain_db;
        double pe = Hph - data[k].phase_deg;
        while (pe > 180) pe -= 360;
        while (pe < -180) pe += 360;

        sum_ge += fabs(ge); sum_g2 += ge*ge;
        sum_pe += fabs(pe); sum_p2 += pe*pe;
        if (fabs(ge) > max_ge) max_ge = fabs(ge);
        if (fabs(pe) > max_pe) max_pe = fabs(pe);

        if (k % 5 == 0 || k == cnt-1 || fabs(ge) > 0.5) {
            printf("%8.0f | %+7.3f  %+7.3f  %+7.3f | %+7.2f %+7.2f %+7.2f\r\n",
                   data[k].freq, data[k].gain_db, fg, ge,
                   data[k].phase_deg, Hph, pe);
        }
    }

    printf("\r\n===== 4th-Order Error Summary =====\r\n");
    printf("Gain  MAE: %.4f dB  RMS: %.4f dB  Max: %.4f dB\r\n", sum_ge/cnt, sqrt(sum_g2/cnt), max_ge);
    printf("Phase MAE: %.4f deg  RMS: %.4f deg  Max: %.4f deg\r\n", sum_pe/cnt, sqrt(sum_p2/cnt), max_pe);

    // Store as SOS: copy first 2nd-order to stage1, rest to stage2 (approximate factorization)
    // For now, user should use Direct Form I implementation
    sos->stage1.b0 = (float)b[0]; sos->stage1.b1 = (float)b[1]; sos->stage1.b2 = (float)b[2];
    sos->stage1.a1 = (float)a[0]; sos->stage1.a2 = (float)a[1];
    sos->stage2.b0 = (float)b[3]; sos->stage2.b1 = (float)b[4]; sos->stage2.b2 = 0;
    sos->stage2.a1 = (float)a[2]; sos->stage2.a2 = (float)a[3];
    IIR_SOS_Reset(sos);

    printf("===== 4th-Order Design Complete =====\r\n\r\n");
    return 0;
}

/**
 * @brief 四阶Direct Form I滤波处理
 *        y[n] = b0*x[n] + b1*x[n-1] + ... + b4*x[n-4]
 *               - a1*y[n-1] - a2*y[n-2] - a3*y[n-3] - a4*y[n-4]
 */
void IIR_Filter_Process_4th(uint16_t *src, uint16_t *dst, uint32_t len, IIR_Filter4* f4)
{
    for(uint32_t i = 0; i < len; i++)
    {
        float input_voltage = (float)src[i] * 3.3f / 4095.0f;
        float x = (input_voltage / 3.3f) * 5.0f - 2.5f;

        float y = f4->b[0] * x
                + f4->b[1] * f4->x[0]
                + f4->b[2] * f4->x[1]
                + f4->b[3] * f4->x[2]
                + f4->b[4] * f4->x[3]
                - f4->a[0] * f4->y[0]
                - f4->a[1] * f4->y[1]
                - f4->a[2] * f4->y[2]
                - f4->a[3] * f4->y[3];

        // Shift delay lines
        f4->x[3] = f4->x[2];
        f4->x[2] = f4->x[1];
        f4->x[1] = f4->x[0];
        f4->x[0] = x;
        f4->y[3] = f4->y[2];
        f4->y[2] = f4->y[1];
        f4->y[1] = f4->y[0];
        f4->y[0] = y;

        if (y > 2.5f) y = 2.5f;
        if (y < -2.5f) y = -2.5f;

        dst[i] = (uint16_t)(((y + 2.5f) / 5.0f) * 16383.0f + 0.5f);
    }
}

/**
 * @brief 重置四阶IIR状态
 */
void IIR_SOS_Reset(IIR_SOS* sos)
{
    IIR_Reset(&sos->stage1);
    IIR_Reset(&sos->stage2);
}

/**
 * @brief IIR滤波器处理函数
 * @param src: 输入ADC原始数据 (12位: 0~4095)
 * @param dst: 输出DAC数据 (14位: 0~16383)
 * @param len: 数据长度
 * @param filter: IIR滤波器结构体指针
 *
 * @note  信号映射:
 *        ADC 12位 (0~4095) -> 电压 0~3.3V
 *        电压 0~3.3V -> DAC电压 -2.5V~+2.5V
 *        DAC电压 -> DAC 14位 (0~16383)
 */
void IIR_Filter_Process(uint16_t *src, uint16_t *dst, uint32_t len, IIR_Filter* filter)
{
    for(uint32_t i = 0; i < len; i++)
    {
        // ADC 12位数据 -> 电压 0~3.3V
        float input_voltage = (float)src[i] * 3.3f / 4095.0f;

        // 映射到DAC输入范围 -2.5V ~ +2.5V
        float x = (input_voltage / 3.3f) * 5.0f - 2.5f;

        // IIR差分方程:
        // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        float y = filter->b0 * x +
                  filter->b1 * filter->x1 +
                  filter->b2 * filter->x2 -
                  filter->a1 * filter->y1 -
                  filter->a2 * filter->y2;

        // 更新延迟状态
        filter->x2 = filter->x1;
        filter->x1 = x;
        filter->y2 = filter->y1;
        filter->y1 = y;

        // 限幅保护，防止溢出
        if (y > 2.5f) y = 2.5f;
        if (y < -2.5f) y = -2.5f;

        // 滤波器输出电压 -> DAC 14位数据 (0~16383)
        dst[i] = (uint16_t)(((y + 2.5f) / 5.0f) * 16383.0f + 0.5f);
    }
}

/**
 * @brief 重置IIR滤波器状态
 * @param filter: IIR滤波器结构体指针
 */
void IIR_Reset(IIR_Filter* filter)
{
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

/**
 * @brief 从模拟传递函数通过双线性变换设计IIR (保留原有接口)
 * @param tf: 二阶传递函数参数
 * @param sample_freq: 采样频率
 * @param filter: 输出IIR系数
 * @return 0:成功
 */
int IIR_Design_From_TF(SecondOrderTF* tf, float sample_freq, IIR_Filter* filter)
{
    if (tf == NULL || filter == NULL || sample_freq <= 0)
        return -1;

    double T = 1.0 / sample_freq;
    double T2 = T * T;

    double B0 = tf->B0;
    double B1 = tf->B1;
    double B2 = tf->B2;
    double A1 = tf->A1;
    double A2 = tf->A2;

    // 双线性变换: s = 2/T * (1 - z^-1)/(1 + z^-1)
    // 预畸变（如果需要的话在此添加）

    double K = 2.0 / T;
    double K2 = K * K;

    // H(z) = H(s) | s = K*(1-z^-1)/(1+z^-1)
    // 代入整理得:
    double den = A2 * K2 + A1 * K + 1.0;

    filter->b0 = (float)((B2 * K2 + B1 * K + B0) / den);
    filter->b1 = (float)((2.0 * B0 - 2.0 * B2 * K2) / den);
    filter->b2 = (float)((B2 * K2 - B1 * K + B0) / den);
    filter->a1 = (float)((2.0 - 2.0 * A2 * K2) / den);
    filter->a2 = (float)((A2 * K2 - A1 * K + 1.0) / den);

    IIR_Reset(filter);
    return 0;
}
