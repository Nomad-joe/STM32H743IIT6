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
#include "./BSP/DMA/my_dma.h"	//Çı¶¯²¢ĞĞdac
#include "math.h"
#include "./BSP/AD9834/my_ad9834.h"//dds¸ø¼¤Àø
#include "./BSP/IIR/my_iir.h"

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
#define FFT_LEN  4096


#define  Freq_Start    1000
#define  Freq_End      50000
#define  Freq_Step     200
#define num_points    ((Freq_End - Freq_Start) / Freq_Step + 1)
#define MAX_EQ      (2 * num_points)
uint16_t ADC1_Buffer[FFT_LEN];
uint16_t ADC2_Buffer[FFT_LEN];
float ADC1_float [FFT_LEN] = {0};
float ADC2_float [FFT_LEN] = {0};
float FFT_input[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};
float FFT2_input[2*FFT_LEN] = {0};
float FFT2_mag[FFT_LEN] ={0};
float FFT_Amp;

// Flat-top window (ISO 18431-1 SFT3F) for accurate amplitude measurement
float flat_top_win[FFT_LEN];
void FlatTopWin_Init(void) {
    for (int i = 0; i < FFT_LEN; i++) {
        double theta = 2.0 * PI * i / (FFT_LEN - 1);
        flat_top_win[i] = (float)(1.0 - 1.932678*cos(theta) + 1.287403*cos(2*theta)
                                  - 0.387006*cos(3*theta) + 0.032220*cos(4*theta));
    }
}

uint32_t Index = 0; //magÊı×éÖĞ×î´óÖµµÄÏÂ±ê

BodeData data[num_points];
typedef struct {
    float b0, b1, b2;
    float a1, a2;
} IIR_Coeff_Direct;

static   double Phi[MAX_EQ * 5] = { 0.0 };
static   double Y[MAX_EQ] = { 0.0 };
static   double PhiT[5 * MAX_EQ] = { 0.0 };

float AD9834_Freq,AD9834_Amp,Filter_Amp;
uint16_t point =0;
uint8_t ADC1_Flag=0;//adc²ÉÑùÍê³ÉÖĞ¶Ï±êÖ¾Î»
uint8_t ADC2_Flag=0;
void RX_Change(void);
void Rx_Delete(void);
double gain(double f);

static int mat_inv5(double* mat, double* inv);
static void mat_mul(double* A, double* B, double* C, int rowA, int colA, int colB);
void Scan_Freq(void);//FFTÉ¨ÆµµÃµ½ÔöÒæºÍÏàÎ»
void Design_Filter(void);//¸ù¾İÉ¨ÆµµÃµ½µÄÊı¾İÇóIIRÏµÊı
int calcSecondOrderTF(BodeData* data, int cnt, SecondOrderTF* tf);
int main(void)
{  
    sys_cache_enable();                     /* Ê¹ÄÜL1-Cache */
    HAL_Init();                             /* ³õÊ¼»¯HAL¿â */
    sys_stm32_clock_init(192, 5, 2, 4);     /* ÉèÖÃÊ±ÖÓ, 480Mhz */
    delay_init(480);                        /* ÑÓÊ±³õÊ¼»¯ */
    usart_init(115200);                     /* ³õÊ¼»¯USART */ 
    led_init();                             /* ³õÊ¼»¯LED */
    mpu_memory_protection();                /* ±£»¤Ïà¹Ø´æ´¢ÇøÓò */
    sdram_init();                           /* ³õÊ¼»¯SDRAM */
    key_init();                             /* ³õÊ¼»¯°´¼ü */  
	AD9834_Init();
  MX_TIM3_Init();       //Çı¶¯adc1 2 ×öÉ¨Æµ 150k
	MX_TIM4_Init();       //Çı¶¯adc3  4M

	MX_GPIO_Init();       //³õÊ¼»¯dac904ËùĞèËùÓĞµÄPC¿Ú
  MX_TIM12_Init();      //2M pwm Çı¶¯dac

	MX_DMA_Init();        //dacÊı¾İ°áÔËdma Ñ­»·Ä£Ê½
	
  PeriphCommonClock_Config();
	MX_ADC1_Init();//PA0  dds
	MX_ADC2_Init();//PA7  filter
  MX_ADC3_Init();//PF10
  
  //HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//PH9 ×öclk
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  delay_ms(20);
//	HAL_TIM_Base_Start(&htim3);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_Buffer, DMA_Len);
//  HAL_TIM_Base_Start(&htim3);

//   	 if(adc1_dma_halfcomplete_flag==1){
//		adc1_dma_halfcomplete_flag=0;
//		IIR_Filter_Process(DMA_Buffer, Filtered_Buffer, DMA_Len/2); 
//			 
//		if(flag==0){ 
//			flag =1;
//			HAL_DMA_Start(&hdma_dma_generator0, 
//              (uint32_t)Filtered_Buffer, 
//              (uint32_t)&GPIOC->ODR, 
//              DMA_Len); 
//    	if(adc1_dma_complete_flag==1){
//	    adc1_dma_complete_flag=0;
//        IIR_Filter_Process(&DMA_Buffer[DMA_Len/2], &Filtered_Buffer[DMA_Len/2], DMA_Len/2);

		AD9834_WaveSeting(1000,0,1,0);AD9834_AmpSet(76);//1V VPP
		Design_Filter();
		while (1)
    {  
			RX_Change();
			delay_ms(20);
	  }

}


//×¢ÒâµÄÊÇ£º°üÍ·ÊÇd£¬°üÎ²±ØĞëÊÇf£¬·ñÔò»á½øÈëËÀÑ­»·

void RX_Change(void)//0£º48 d£º100 
{
	
  if(g_usart_rx_buf[0] == 100)//d:100 ×ö°üÍ·£¬f£º102×ö°üÎ²£¬arr[1]ÒÔºóÓÃ×÷Ä£Ê½ÇĞ»»
	{
    float num = 0;//ÔÌº¬ĞÅÏ¢×ª»»ºó´æ·ÅÔÚnumÖĞ
	  float point_ten = 10;//ÓÃÓÚµİ½øÊ®·ÖÎ»°Ù·ÖÎ»
		uint8_t i = 2;//Êı¾İĞÅÏ¢´ÓµÚ2Î»¿ªÊ¼
		
		uint8_t point = 0;//ÓÃÓÚÇø·ÖĞ¡ÊıµãÇ°ºó
		while(g_usart_rx_buf[i] != 102)//²»ÊÇ°üÎ²
		{
		  if(g_usart_rx_buf[i] == 46){point=1;i++;continue;}//¸ÃÎ»ÊÇĞ¡Êıµã
			if(point ==1){num += (g_usart_rx_buf[i]-48)/point_ten;point_ten*=10;i++;continue;}
		  else {num *=10; num +=g_usart_rx_buf[i]-48;i++;continue;}		
		}
		if(g_usart_rx_buf[1] ==50)     {AD9834_Freq = num;AD9834_WaveSeting(AD9834_Freq,0,1,0);AD9834_AmpSet(255);}//µÚ¶şÎÊ
		else if(g_usart_rx_buf[1] ==51){printf("t4.txt=\"1.0V\"\xff\xff\xff");AD9834_WaveSeting(1000,0,1,0);AD9834_AmpSet(60+1); }//µÚÈıÎÊ
		else if(g_usart_rx_buf[1] ==52){printf("t4.txt=\"%.2fV\"\xff\xff\xff",num);Filter_Amp = num; AD9834_Amp =74.5* Filter_Amp/gain(AD9834_Freq);AD9834_AmpSet(AD9834_Amp);}
		else if(g_usart_rx_buf[1] ==53){AD9834_Freq = num;AD9834_WaveSeting(AD9834_Freq,0,1,0);}
		else if(g_usart_rx_buf[1] ==54){Design_Filter();}//d6f
		printf("t1.txt=\"%.1f\"\xff\xff\xff",AD9834_Freq);
		
	}
    Rx_Delete();
}

void Rx_Delete(void)
{
  for(uint8_t i=0;i<50;i++)g_usart_rx_buf[i] = 0;
  g_usart_rx_sta =0;
}
void Scan_Freq(void)
{
    // printf("t2.txt=\"¿ªÊ¼Ñ§Ï°\"\xff\xff\xff"); 
     AD9834_AmpSet(76);//DDS·ù¶ÈÉèÖÃÎª1Vpp
	   for(uint16_t DDS_freq =Freq_Start;DDS_freq<=Freq_End;DDS_freq+=Freq_Step)
     {
		    AD9834_WaveSeting(DDS_freq,0,1,0);delay_ms(20);//¸ø³ö¼¤Àø
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_Buffer, FFT_LEN);         
			  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_Buffer, FFT_LEN);
        HAL_TIM_Base_Start(&htim3);			 
			  while(ADC1_Flag==0 || ADC2_Flag ==0){}
				ADC1_Flag = 0; ADC2_Flag =0;//²ÉÑùÍê³É
			  HAL_TIM_Base_Stop(&htim3);
for(uint16_t i=0;i<FFT_LEN;i++){

    ADC1_float[i] = (float)ADC1_Buffer[i]*3.3f/65535 ;
	  ADC1_float[i] *= flat_top_win[i];  //Ê±Óò¼Ó´°
    FFT_input[2*i] = ADC1_float[i];
    FFT_input[2*i+1] = 0;

    ADC2_float[i] = (float)ADC2_Buffer[i]*3.3f/65535;
	  ADC2_float[i] *= flat_top_win[i];  //Ê±Óò¼Ó´°
    FFT2_input[2*i] = ADC2_float[i];
    FFT2_input[2*i+1] = 0;
}
		
		arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
		FFT_mag[0]=0;FFT_mag[1]=0;FFT_mag[2]=0;FFT_mag[3]=0;
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT2_input, 0, 1);
    arm_cmplx_mag_f32(FFT2_input, FFT2_mag, FFT_LEN);
		
	 float Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
//	 data[point].freq = Finally_Index*36.62109375;
		data[point].freq =DDS_freq; 
			
	float FFT1_Amp = FFT_mag[Index];
	float FFT2_Amp = FFT2_mag[Index]/1.03f;

	
	 

	//   printf("·Å´ó±¶Êı:%.2f\r\n",1.8*FFT_Amp_2/Finally_Amp);

			
	// Phase: use peak bin directly (no weighted avg of complex values)
	// Weighted averaging of FFT bins with different phases causes 180deg jumps
	float real_1 = FFT_input[2*Index];
	float imag_1 = FFT_input[2*Index+1];
	float real_2 = FFT2_input[2*Index];
	float imag_2 = FFT2_input[2*Index+1];

	// Fallback: if peak SNR too low, use stronger neighbor bin
	float mag_peak = FFT_mag[Index];
	float mag_neighbor = (FFT_mag[Index-1] + FFT_mag[Index+1]) * 0.5f;
	if (mag_peak < mag_neighbor * 0.5f) {
	    if (FFT_mag[Index-1] > FFT_mag[Index+1]) {
	        real_1 = FFT_input[2*(Index-1)]; imag_1 = FFT_input[2*(Index-1)+1];
	        real_2 = FFT2_input[2*(Index-1)]; imag_2 = FFT2_input[2*(Index-1)+1];
	    } else {
	        real_1 = FFT_input[2*(Index+1)]; imag_1 = FFT_input[2*(Index+1)+1];
	        real_2 = FFT2_input[2*(Index+1)]; imag_2 = FFT2_input[2*(Index+1)+1];
	    }
	}

	float phase_1 = atan2f(imag_1, real_1) * 180.0f / PI;
	float phase_2 = atan2f(imag_2, real_2) * 180.0f / PI;

float Pre_Phase = phase_2 - phase_1;
while(Pre_Phase >= 180) Pre_Phase -= 360;
while(Pre_Phase <= -180) Pre_Phase += 360;

      
//    printf("Freq = %d hz \r\n",DDS_freq);
//	 	printf("FFT1_Amp = %.2f mv\r\n",FFT1_Amp); 
//	  printf("FFT2_Amp = %.2f mv\r\n",FFT2_Amp);
	 	data[point].gain_db = 20.0 * log10(FFT2_Amp/FFT1_Amp);
//	 printf("ÔöÒæ:%.2f db\r\n",data[point].gain_db);	 
	 
		 
		 data[point++].phase_deg = Pre_Phase;	 
			printf("²âÁ¿Æµµã:%.2f hz   ",data[point-1].freq);
			printf("ÔöÒæ:%.2f db   ",data[point-1].gain_db);
			printf("ÏàÎ»²î:%.2f ¶È\r\n",data[point-1].phase_deg);	
	
//	if(DDS_freq>=40000-1)while(1){};
}

	// ===== Phase cleanup: 3-point median filter to fix glitches =====
	{
	    float phase_orig[num_points];
	    for (int i = 0; i < point; i++) phase_orig[i] = data[i].phase_deg;
	    for (int i = 1; i < point - 1; i++) {
	        float a = data[i-1].phase_deg;
	        float b = data[i].phase_deg;
	        float c = data[i+1].phase_deg;
	        if (a > b) { float t = a; a = b; b = t; }
	        if (b > c) { float t = b; b = c; c = t; }
	        if (a > b) { float t = a; a = b; b = t; }
	        float median = b;
	        float diff = data[i].phase_deg - median;
	        while (diff > 180) diff -= 360;
	        while (diff < -180) diff += 360;
		    if (fabsf(diff) > 20.0f) {
		        data[i].phase_deg = median;
		        printf("Phase fixed at %.0f Hz\r\n", data[i].freq);
		    }
		}
	}

}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    //adc1_dma_halfcomplete_flag = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{   if(hadc->Instance == ADC1){ ADC1_Flag= 1;}
    if(hadc->Instance == ADC2){ ADC2_Flag= 1;}
	  else if(hadc->Instance == ADC3){}
}
double gain(double f) {
    double omega = 2 * PI * f;
    double real = 1 - 1e-8 * omega * omega;      // Êµ²¿
    double imag = 3e-4 * omega;                   // Ğé²¿
    return 5.0 / sqrt(real * real + imag * imag); // ·ùÖµ
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


// ÄâºÏº¯Êı£ºÊäÈëÈ«²¿É¨ÆµÊı¾İ£¬Êä³ö¹éÒ»»¯¶ş½×´«µİº¯Êı²ÎÊı
int calcSecondOrderTF(BodeData* data, int cnt, SecondOrderTF* tf)
{
    if (cnt < 5 || cnt > num_points || tf == NULL || data == NULL)
        return -1;

    const int eq_num = 2 * cnt;
    const int para_n = 5;
    for (int k = 0; k < cnt; k++)
    {
        double f = data[k].freq;
        double g_db = data[k].gain_db;
        double p_deg = data[k].phase_deg;  // ÒÑ¾­ÊÇÂË²¨Æ÷ÕæÊµÏàÎ»£¨ÒÑ¼õ180¡ã£©

        double mag = pow(10.0, g_db / 20.0);
        double phase = p_deg * PI / 180.0;
        double omega = 2.0 * PI * f;
        double w2 = omega * omega;

        double x = mag * cos(phase);
        double y = mag * sin(phase);

        // Êµ²¿·½³Ì
        int r1 = 2 * k;
        Phi[r1 * 5 + 0] = -w2 * x;
        Phi[r1 * 5 + 1] = omega;
        Phi[r1 * 5 + 2] = 1.0;
        Phi[r1 * 5 + 3] = -omega * y;
        Phi[r1 * 5 + 4] = x;
        Y[r1] = w2 * x;

        // Ğé²¿·½³Ì
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

    // ===== ¹Ø¼ü£ºÈ·±£ÂË²¨Æ÷Ö±Á÷ÔöÒæÎªÕı =====
    // ¶ÔÓÚÕı³£ÂË²¨Æ÷£¨·Ç·´Ïà£©£¬µÍÆµÔöÒæÓ¦¸ÃÎªÕı
    double dc_gain = tf->B2 / tf->A2;
    if (dc_gain < 0) {
        // Èç¹ûÖ±Á÷ÔöÒæÎª¸º£¬ËµÃ÷Êı¾İÖĞ»¹ÓĞ²ĞÓàµÄ180¡ãÎÊÌâ
        // ÕûÌå·´×ª·Ö×Ó
        tf->B0 = -tf->B0;
        tf->B1 = -tf->B1;
        tf->B2 = -tf->B2;
        // ÖØĞÂ¼ÆËãÖ±Á÷ÔöÒæ
        dc_gain = tf->B2 / tf->A2;
    }
    
    // ÑéÖ¤Ö±Á÷ÔöÒæÊÇ·ñºÏÀí£¨¿ÉÑ¡£©
    printf("Fit check: DC gain = %.4f (%.2f dB)\r\n", dc_gain, 20.0 * log10(dc_gain));

    return 0;
}
void Design_Filter(void)
{ 
    point = 0;
	  FlatTopWin_Init();
    Scan_Freq();
    IIR_Filter iir_coeff;
    int ret = IIR_Design_Direct(data, num_points, 2000000.0f, &iir_coeff);
    
    if (ret == 0)
    {
        printf("\r\n===== IIRç³»æ•°è·å–æˆåŠŸï¼ˆ2é˜¶æœ€ä¼˜æ‹Ÿåˆï¼‰ =====\r\n");
        printf("float b0 = %.10ff;\r\n", iir_coeff.b0);
        printf("float b1 = %.10ff;\r\n", iir_coeff.b1);
        printf("float b2 = %.10ff;\r\n", iir_coeff.b2);
        printf("float a1 = %.10ff;\r\n", iir_coeff.a1);
        printf("float a2 = %.10ff;\r\n", iir_coeff.a2);
        printf("\r\n===== ç³»æ•°è·å–å®Œæˆ =====\r\n");
    }
    else
    {
        printf("\r\nIIRæ‹Ÿåˆå¤±è´¥ï¼\r\n");
    }
}

