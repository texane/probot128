/* libstm32l_discovery headers */
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_dac.h"
#include "stm32l1xx_lcd.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_rtc.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_flash.h"
#include "stm32l1xx_syscfg.h"
#include "stm32l1xx_dbgmcu.h"


#define GPIO_HIGH(__a, __b) do { (__a)->BSRRL = (__b); } while (0)
#define GPIO_LOW(__a, __b) do { (__a)->BSRRH = (__b); } while (0)


/* motors.
   PA1: enable right motor
   PA2: enable left motor
   PA3: enable motors
 */

static void setup_motors(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  /* GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_LOW(GPIOA, GPIO_Pin_1);
  GPIO_LOW(GPIOA, GPIO_Pin_2);
  GPIO_LOW(GPIOA, GPIO_Pin_3);
}

static inline void enable_motors
(unsigned int lstate, unsigned int rstate)
{
  GPIO_LOW(GPIOA, GPIO_Pin_3);

  if (rstate) GPIO_HIGH(GPIOA, GPIO_Pin_2);
  else GPIO_LOW(GPIOA, GPIO_Pin_2);

  if (lstate) GPIO_HIGH(GPIOA, GPIO_Pin_1);
  else GPIO_LOW(GPIOA, GPIO_Pin_1);

  GPIO_HIGH(GPIOA, GPIO_Pin_3);
}

static inline void disable_motors(void)
{
  GPIO_LOW(GPIOA, GPIO_Pin_1);
  GPIO_LOW(GPIOA, GPIO_Pin_2);
  GPIO_LOW(GPIOA, GPIO_Pin_3);
}


/* led */

static void setup_leds(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_LOW(GPIOB, GPIO_Pin_6);
  GPIO_LOW(GPIOB, GPIO_Pin_7);
}


#define delay()						\
do {							\
  volatile unsigned int i;				\
  for (i = 0; i < 1000000; ++i)				\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)


static void RCC_Configuration(void)
{
  /* HSI is 16mhz RC clock directly fed to SYSCLK (rm00038, figure 9) */

  /* enable the HSI clock (high speed internal) */
  RCC_HSICmd(ENABLE);
  
  /* wail til HSI ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {}

  /* at startup, SYSCLK driven by MSI. set to HSI */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  
  /* set MSI to 4mhz */
  RCC_MSIRangeConfig(RCC_MSIRange_6);

  /* turn HSE off */
  RCC_HSEConfig(RCC_HSE_OFF);  
  if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
  {
    while (1) ;
  }
}


static void RTC_Configuration(void)
{
  /* Allow access to the RTC */
  PWR_RTCAccessCmd(ENABLE);

  /* Reset Backup Domain */
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);

  /* LSE Enable */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {}
  
  RCC_RTCCLKCmd(ENABLE);
   
  /* LCD Clock Source Selection */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

}


static inline void move_back(void)
{
  enable_motors(1, 0);
}

static inline void move_front(void)
{
  enable_motors(0, 1);
}

static inline void turn(void)
{
  enable_motors(0, 0);
}


static void do_roundtrip(void)
{
  while (1)
  {
    delay();
    delay();

    move_front();
    delay();
    disable_motors();
    delay();

    turn();
    delay();
    disable_motors();
    delay();

    move_front();
    delay();
    disable_motors();
    delay();

    turn();
    delay();
    disable_motors();
    delay();
  }
}


static void setup_button(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

#if 0 /* already done by setup_led */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#endif

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


static void do_button(void)
{
  while (1)
  {
    uint8_t bit = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);

    if (bit == Bit_SET)
    {
      GPIO_LOW(GPIOB, GPIO_Pin_6);
      GPIO_HIGH(GPIOB, GPIO_Pin_7);
    }
    else /* if (bit == Bit_RESET) */
    {
      GPIO_HIGH(GPIOB, GPIO_Pin_6);
      GPIO_LOW(GPIOB, GPIO_Pin_7);
    }

    delay();
  }
}


void main(void)
{
  /* Configure Clocks for Application need */
  RCC_Configuration();
  
  /* Configure RTC Clocks */
  RTC_Configuration();

  setup_motors();

  setup_leds();

  setup_button();

  do_button();
  do_roundtrip();

  while (1)
  {
    GPIO_LOW(GPIOB, GPIO_Pin_6);
    GPIO_HIGH(GPIOB, GPIO_Pin_7);
    delay();

    GPIO_HIGH(GPIOB, GPIO_Pin_6);
    GPIO_LOW(GPIOB, GPIO_Pin_7);
    delay();
  }
}
