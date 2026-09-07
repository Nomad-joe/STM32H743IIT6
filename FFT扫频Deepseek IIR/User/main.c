/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       DSP FFT 实验
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/SDRAM/sdram.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/timer.h"
#include "./BSP/ADC/my_adc.h"
#include "./BSP/FFT/my_fft.h"
#include "./CMSIS/DSP/Include/arm_math.h"
#include "./BSP/AD9834/my_ad9834.h"
#include "./BSP/IIR/my_iir.h"


#define FFT_LEN  4096 
uint16_t DMA_Buffer[FFT_LEN] = {0};
uint16_t DMA_Buffer_2[FFT_LEN] = {0};
float ADC_float [FFT_LEN] = {0};
float ADC_float_2 [FFT_LEN] = {0};
float FFT_input[2*FFT_LEN] = {0};
float FFT_input_2[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};//存放频谱
float FFT_mag_2[FFT_LEN] ={0};
uint32_t Index = 0; //mag数组中最大值的下标
float Finally_Index ;
float Finally_Freq;  //准确的频率
float Finally_Amp;   //准确的幅值
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
float FFT_Amp_2,FFT_Amp;
uint8_t DMA1_flag =0;
uint8_t DMA2_flag =0;




uint16_t point =0;
#define num_points  29 //扫频点数
#define MAX_EQ      (2 * num_points)
 	
float Start_F	= 100.0;
float End_F =3000.0;

typedef struct {
    float freq;      // 频率 (Hz)
    float gain_db;   // 增益 (dB)
    float phase_deg; // 相位差 (度)
} BodeData;
BodeData data[num_points];

int calcSecondOrderTF(BodeData* data, int cnt, SecondOrderTF* tf);
static int mat_inv5(double* mat, double* inv);
static void mat_mul(double* A, double* B, double* C, int rowA, int colA, int colB);
void Scan_Freq(float Start_F, float End_F,BodeData* data);//扫频
void Design_Filter(void);//根据扫频得到的数据求IIR系数
int main(void)
{  
 //   sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
    MX_USART2_UART_Init(115200);
    
	MX_TIM3_Init();//初始化adc所需时钟
  AD9834_Init();//初始化DDS
  PeriphCommonClock_Config();
	MX_ADC1_Init();//PA0  dds
	MX_ADC2_Init();//PA7  filter
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
  delay_ms(20);
  HAL_TIM_Base_Start(&htim3);
	AD9834_AmpSet(65);
	AD9834_WaveSeting(100,0,1,0);

  Design_Filter();
		
		
    while (1)
    {
		


	 

   }

}
void Scan_Freq(float Start_F, float End_F,BodeData* data)  //输入参数：开始频率，终止频率，存放增益数组，存放相位差数组   
{

  for(uint16_t DDS_freq=Start_F;DDS_freq<=End_F;DDS_freq+=100){
      AD9834_WaveSeting(DDS_freq,0,1,0);delay_ms(5);//给出激励
		  //data[point++].freq=DDS_freq;
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC1开始采集，DMA开启搬运
			HAL_ADC_Start_DMA(&hadc2,(uint32_t*)DMA_Buffer_2,FFT_LEN);//ADC2开始采集，DMA开启搬运

	    while(DMA1_flag !=1 || DMA2_flag!=1){}
			DMA1_flag =0;	DMA2_flag=0;// 采样完成
				
			for(uint16_t i=0;i<FFT_LEN;i++){  //时域加窗 + 转变为复信号
			  ADC_float[i] = (float)DMA_Buffer[i]*3.3f/65536;
				ADC_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  
				FFT_input[2*i] = ADC_float[i];
			  FFT_input[2*i+1] = 0;
				
			  ADC_float_2[i] = (float)DMA_Buffer_2[i]*3.3f/65536;
				ADC_float_2[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  
				FFT_input_2[2*i] = ADC_float_2[i];
			  FFT_input_2[2*i+1] = 0;
			}	
		
		arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
		FFT_mag[0]=0;FFT_mag[1]=0;FFT_mag[2]=0;
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input_2, 0, 1);
    arm_cmplx_mag_f32(FFT_input_2, FFT_mag_2, FFT_LEN);
		
	 Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
	 data[point].freq = Finally_Index*3.662109375;
		
			
   Finally_Amp = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1])/0.62f*1.05;
   FFT_Amp_2   = sqrt(FFT_mag_2[Index-1] * FFT_mag_2[Index-1] + FFT_mag_2[Index] * FFT_mag_2[Index] + FFT_mag_2[Index+1] * FFT_mag_2[Index+1])/0.62f/1.03f*1.75f;
   //   printf("放大倍数:%.2f\r\n",1.8*FFT_Amp_2/Finally_Amp);
		data[point].gain_db = 20.0 * log10(FFT_Amp_2/Finally_Amp); 
			
// 使用幅值平方作为统一权重
float w_prev = FFT_mag[Index-1] * FFT_mag[Index-1];
float w_curr = FFT_mag[Index] * FFT_mag[Index];
float w_next = FFT_mag[Index+1] * FFT_mag[Index+1];
float w_sum = w_prev + w_curr + w_next;

// ADC1的复数加权平均
float real_1 = (FFT_input[2*(Index-1)] * w_prev + 
                FFT_input[2*Index] * w_curr + 
                FFT_input[2*(Index+1)] * w_next) / w_sum;
float imag_1 = (FFT_input[2*(Index-1)+1] * w_prev + 
                FFT_input[2*Index+1] * w_curr + 
                FFT_input[2*(Index+1)+1] * w_next) / w_sum;

// ADC2的复数加权平均
float w2_prev = FFT_mag_2[Index-1] * FFT_mag_2[Index-1];
float w2_curr = FFT_mag_2[Index] * FFT_mag_2[Index];
float w2_next = FFT_mag_2[Index+1] * FFT_mag_2[Index+1];
float w2_sum = w2_prev + w2_curr + w2_next;

float real_2 = (FFT_input_2[2*(Index-1)] * w2_prev + 
                FFT_input_2[2*Index] * w2_curr + 
                FFT_input_2[2*(Index+1)] * w2_next) / w2_sum;
float imag_2 = (FFT_input_2[2*(Index-1)+1] * w2_prev + 
                FFT_input_2[2*Index+1] * w2_curr + 
                FFT_input_2[2*(Index+1)+1] * w2_next) / w2_sum;

// 计算相位
float phase_1 = atan2f(imag_1, real_1) * 180.0f / PI;
float phase_2 = atan2f(imag_2, real_2) * 180.0f / PI;




float Pre_Phase = phase_2 - phase_1;
while(Pre_Phase >= 180) Pre_Phase -= 360;
while(Pre_Phase <= -180) Pre_Phase += 360;

       data[point++].phase_deg = Pre_Phase-180.0f;	
      
			printf("测量频点:%.2f hz   ",data[point-1].freq);
			printf("增益:%.2f db   ",data[point-1].gain_db);
			printf("相位差:%.2f 度\r\n",data[point-1].phase_deg);
		}			
				
}
	
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)       DMA1_flag =1;
		else if (hadc->Instance == ADC2) 	DMA2_flag =1;
}
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

// 拟合函数：输入全部扫频数据，输出归一化二阶传递函数参数
int calcSecondOrderTF(BodeData* data, int cnt, SecondOrderTF* tf)
{
    if (cnt < 5 || cnt > num_points || tf == NULL || data == NULL)
        return -1;

    const int eq_num = 2 * cnt;
    const int para_n = 5;
    double Phi[MAX_EQ * 5] = { 0.0 };
    double Y[MAX_EQ] = { 0.0 };

    for (int k = 0; k < cnt; k++)
    {
        double f = data[k].freq;
        double g_db = data[k].gain_db;
        double p_deg = data[k].phase_deg;  // 已经是滤波器真实相位（已减180°）

        double mag = pow(10.0, g_db / 20.0);
        double phase = p_deg * PI / 180.0;
        double omega = 2.0 * PI * f;
        double w2 = omega * omega;

        double x = mag * cos(phase);
        double y = mag * sin(phase);

        // 实部方程
        int r1 = 2 * k;
        Phi[r1 * 5 + 0] = -w2 * x;
        Phi[r1 * 5 + 1] = omega;
        Phi[r1 * 5 + 2] = 1.0;
        Phi[r1 * 5 + 3] = -omega * y;
        Phi[r1 * 5 + 4] = x;
        Y[r1] = w2 * x;

        // 虚部方程
        int r2 = 2 * k + 1;
        Phi[r2 * 5 + 0] = w2 * y;
        Phi[r2 * 5 + 1] = -omega;
        Phi[r2 * 5 + 2] = 0.0;
        Phi[r2 * 5 + 3] = omega * x;
        Phi[r2 * 5 + 4] = y;
        Y[r2] = w2 * y;
    }

    double PhiT[5 * MAX_EQ] = { 0.0 };
    for (int i = 0; i < para_n; i++)
        for (int j = 0; j < eq_num; j++)
            PhiT[i * eq_num + j] = Phi[j * para_n + i];

    double PhiT_Phi[25] = { 0.0 };
    mat_mul(PhiT, Phi, PhiT_Phi, 5, eq_num, 5);

    double inv_PhiT_Phi[25] = { 0.0 };
    if (mat_inv5(PhiT_Phi, inv_PhiT_Phi) != 0)
        return -1;

    double PhiT_Y[5] = { 0.0 };
    mat_mul(PhiT, Y, PhiT_Y, 5, eq_num, 1);

    double theta[5] = { 0.0 };
    mat_mul(inv_PhiT_Phi, PhiT_Y, theta, 5, 5, 1);

    tf->B0 = theta[0];
    tf->B1 = theta[1];
    tf->B2 = theta[2];
    tf->A1 = theta[3];
    tf->A2 = theta[4];

    // ===== 关键：确保滤波器直流增益为正 =====
    // 对于正常滤波器（非反相），低频增益应该为正
    double dc_gain = tf->B2 / tf->A2;
    if (dc_gain < 0) {
        // 如果直流增益为负，说明数据中还有残余的180°问题
        // 整体反转分子
        tf->B0 = -tf->B0;
        tf->B1 = -tf->B1;
        tf->B2 = -tf->B2;
        // 重新计算直流增益
        dc_gain = tf->B2 / tf->A2;
    }
    
    // 验证直流增益是否合理（可选）
    printf("Fit check: DC gain = %.4f (%.2f dB)\r\n", dc_gain, 20.0 * log10(dc_gain));

    return 0;
}
void Design_Filter(void)
{
	    
    point = 0;// 重置扫频点计数器
	
    // 1.扫频  
    Scan_Freq(Start_F,End_F,data);
		// 2. 第二步：调用拟合函数
    SecondOrderTF tf_param;
    int ret = calcSecondOrderTF(data, num_points, &tf_param);
    // 3. 输出拟合结果
if (ret == 0)
{
    printf("\r\n===== Raw Parameters =====\r\n");
    printf("B0 = %f\r\n", (float)tf_param.B0);
    printf("B1 = %f\r\n", (float)tf_param.B1);
    printf("B2 = %f\r\n", (float)tf_param.B2);
    printf("A1 = %f\r\n", (float)tf_param.A1);
    printf("A2 = %f\r\n", (float)tf_param.A2);
    
    printf("\r\n===== Analysis =====\r\n");
    printf("Transfer function:\r\n");
    printf("G(s) = (%.4f s^2 + %.4f s + %.4f) / (s^2 + %.4f s + %.4f)\r\n",
           tf_param.B0, tf_param.B1, tf_param.B2, tf_param.A1, tf_param.A2);
 
    float dc_gain = tf_param.B2 / tf_param.A2;
    printf("DC gain = %.4f (%.2f dB)\r\n", dc_gain, 20.0f * log10(fabs(dc_gain)));
    
    if (tf_param.A2 > 0) {
        float w0 = sqrt(tf_param.A2);
        float fn = w0 / (2.0f * PI);
        float zeta = tf_param.A1 / (2.0f * w0);
        printf("Natural freq fn = %.2f Hz\r\n", fn);
        printf("Damping zeta = %.4f\r\n", zeta);
        
        if (fabs(tf_param.B0) < 0.01 && fabs(tf_param.B1) < 0.01) {
            printf("Type: Low-pass filter\r\n");
        } else if (fabs(tf_param.B2) < 0.01) {
            printf("Type: High-pass or Band-pass filter\r\n");
        } else if (tf_param.B0 > 0.01) {
            printf("Type: Notch or complex filter\r\n");
        } else {
            printf("未识别\r\n");
        }
    }
    
    // 验证拟合质量
    printf("\r\n===== Verification =====\r\n");
    float test_freqs[] = {100.0f, 500.0f, 1000.0f, 2000.0f, 3000.0f};
    for (int i = 0; i < 5; i++) {
        float w = 2.0f * PI * test_freqs[i];
        float w2 = w * w;
        
        float num_real = -tf_param.B0 * w2 + tf_param.B2;
        float num_imag = tf_param.B1 * w;
        float den_real = -w2 + tf_param.A2;
        float den_imag = tf_param.A1 * w;
        
        float mag = sqrt(num_real*num_real + num_imag*num_imag) / 
                    sqrt(den_real*den_real + den_imag*den_imag);
        float phase = atan2f(num_imag, num_real) - atan2f(den_imag, den_real);
        phase = phase * 180.0f / PI;
        
        while (phase > 180.0f) phase -= 360.0f;
        while (phase < -180.0f) phase += 360.0f;
        
        printf("f=%.0f Hz: Gain=%.2f dB, Phase=%.2f deg\r\n",
               test_freqs[i], 20.0f*log10(mag), phase);
    }
    
    // ===== 新增：设计IIR滤波器并获取系数 =====
    printf("\r\n===== 设计IIR数字滤波器 =====\r\n");
    
    IIR_Filter iir_coeff;
    // 第二个参数是ADC采样频率，请根据实际修改！
    if (IIR_Design_From_TF(&tf_param, 2000000.0f, &iir_coeff) == 0)
    {
        printf("\r\n===== IIR滤波器系数（复制到滤波工程使用） =====\r\n");
        printf("float b0 = %.10ff;\r\n", iir_coeff.b0);
        printf("float b1 = %.10ff;\r\n", iir_coeff.b1);
        printf("float b2 = %.10ff;\r\n", iir_coeff.b2);
        printf("float a1 = %.10ff;\r\n", iir_coeff.a1);
        printf("float a2 = %.10ff;\r\n", iir_coeff.a2);
        printf("\r\n===== 系数获取完成 =====\r\n");
    }
    else
    {
        printf("IIR滤波器设计失败！\r\n");
    }
}
   else
   {
    printf("\r\n拟合失败！\r\n");
   }
}