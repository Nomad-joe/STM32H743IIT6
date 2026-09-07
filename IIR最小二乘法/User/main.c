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
#include <math.h>

#define POINTS_PER_CYCLE  20
#define CYCLE_NUM         3
#define TOTAL_POINTS      (POINTS_PER_CYCLE * CYCLE_NUM)

// 直通：输入=输出
static const float b[3] = {1.0F, 0.0F, 0.0F};
static const float a[3] = {1.0F, 0.0F, 0.0F};

static float x[3] = {0};
static float y[3] = {0};

float iir2_filter(float in);
void wave_generate_send(void);
void iir_reset(void);

int main(void)
{  
    sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(192, 5, 2, 4);
    delay_init(480);
    usart_init(115200);
    led_init();
    mpu_memory_protection();
    sdram_init();
    key_init();
    
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    
    while (1)
    {  
        wave_generate_send();
        delay_ms(100);
    }
}

void iir_reset(void)
{
    x[0] = x[1] = x[2] = 0.0f;
    y[0] = y[1] = y[2] = 0.0f;
}

float iir2_filter(float in)
{
    float out;
    
    x[2] = x[1];
    x[1] = x[0];
    x[0] = in;

    out = b[0] * x[0] + b[1] * x[1] + b[2] * x[2];
    out = out - a[1] * y[1] - a[2] * y[2];

    y[2] = y[1];
    y[1] = y[0];
    y[0] = out;

    return out;
}

void wave_generate_send(void)
{
    float square;
    float filtered;
    int i;

    iir_reset();

    for(i=0; i<TOTAL_POINTS; i++)
    {
        if( (i % POINTS_PER_CYCLE) < (POINTS_PER_CYCLE/2) )
            square = 1.0f;
        else
            square = -1.0f;

        filtered = iir2_filter(square);
        printf("%.6f\r\n", filtered);
    }
}