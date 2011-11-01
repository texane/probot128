/* libstm32l_discovery headers */
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_comp.h"
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


extern void mcs_init(void);


#define GPIO_HIGH(__a, __b) do { (__a)->BSRRL = (__b); } while (0)
#define GPIO_LOW(__a, __b) do { (__a)->BSRRH = (__b); } while (0)


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


#if 0 /* toremove */

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

#endif /* toremove */


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


#if 0 /* toremove */

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

#endif /* toremove */


#if 0 /* unused, tim2 general purpose timer */

#include "stm32l1xx_tim.h"
#include "misc.h"

#define enableInterrupts() __set_PRIMASK(0)
#define disableInterrupts() __set_PRIMASK(1)

void TIM2_IRQHandler(void)
{
  /* 10hz frequency */

  static unsigned int state = 0;
  static unsigned int n = 10;

  /* todo: reduce the scope by capturing variables */
  disableInterrupts();

  if (--n) goto on_done;
  n = 10;

  if (state)
  {
    GPIO_LOW(GPIOB, GPIO_Pin_6);
    GPIO_HIGH(GPIOB, GPIO_Pin_7);
    state = 0;
  }
  else
  {
    GPIO_HIGH(GPIOB, GPIO_Pin_6);
    GPIO_LOW(GPIOB, GPIO_Pin_7);
    state = 1;
  }

 on_done:
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  enableInterrupts();
}


static void setup_tim2(void)
{
  /* rcc is 16mhz, tim2 is 10hz */

  TIM_TimeBaseInitTypeDef TIM_TimeBase;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* init timebase */
  TIM_TimeBase.TIM_Prescaler = 16000; /* for 1khz clock */
  TIM_TimeBase.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBase.TIM_Period = 100; /* 10 hz */
  TIM_TimeBase.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBase);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  TIM_Cmd(TIM2, ENABLE);
}

#endif /* unused */


#if 1 /* unused */

/* pwm */

#include "stm32l1xx_tim.h"
#include "misc.h"

#define CONFIG_PWM_RCHAN 0
#define CONFIG_PWM_LCHAN 1

static void set_pwm(unsigned int duty, unsigned int chan)
{
  TIM_OCInitTypeDef TIM_OCInit;

  TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInit.TIM_Pulse = duty;
  TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;

  if (chan == CONFIG_PWM_LCHAN)
  {
    TIM_OC2Init(TIM3, &TIM_OCInit);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  }
  else /* if (chan == CONFIG_PWM_RCHAN) */
  {
    TIM_OC1Init(TIM3, &TIM_OCInit);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  }
}

static inline void set_lpwm(unsigned int duty)
{
  /* lpwm mapped on PORTB5 */
  set_pwm(duty, CONFIG_PWM_LCHAN);
}

static inline void set_rpwm(unsigned int duty)
{
  /* lpwm is mapped on PORTB4 */
  set_pwm(duty, CONFIG_PWM_RCHAN);
}

static void setup_pwms(void)
{
  /* for pin alternate functions, refer to CD00277537.pdf, table 4 */

  TIM_TimeBaseInitTypeDef TIM_TimeBase;
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* configure gpio */
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

  /* init timebase. PWM freq equals 10 khz */
  TIM_TimeBaseStructInit(&TIM_TimeBase);
  TIM_TimeBase.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBase.TIM_Prescaler = 8 - 1; /* for 1 Mhz clock */
  TIM_TimeBase.TIM_Period = 200 - 1; /* 10 khz */
  TIM_TimeBase.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBase);

  set_lpwm(0);
  set_rpwm(0);

  TIM_Cmd(TIM3, ENABLE);
}

#endif /* unused */


/* left and right encoders */

#include "misc.h"

#define enableInterrupts() __set_PRIMASK(0)
#define disableInterrupts() __set_PRIMASK(1)

#if 1 /* gpio switch version */

static void EncoderHandler(void)
{
  disableInterrupts();

  if (GPIOA->IDR & GPIO_Pin_3)
    GPIO_LOW(GPIOB, GPIO_Pin_6);
  else
    GPIO_HIGH(GPIOB, GPIO_Pin_6);

  if (GPIOA->IDR & GPIO_Pin_4)
    GPIO_LOW(GPIOB, GPIO_Pin_7);
  else
    GPIO_HIGH(GPIOB, GPIO_Pin_7);

  EXTI_ClearITPendingBit(EXTI_Line0);

  enableInterrupts();
}

void EXTI3_IRQHandler(void)
{
  EncoderHandler();
}

void EXTI4_IRQHandler(void)
{
  EncoderHandler();
}

static void setup_encoders(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* setup PORTA3 interrupt */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);

  EXTI_ClearITPendingBit(EXTI_Line3);
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* setup PORTA4 interrupt */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);

  EXTI_ClearITPendingBit(EXTI_Line4);
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* setup ir led */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_HIGH(GPIOB, GPIO_Pin_8);
}

#else

void COMP_IRQHandler(void)
{
  disableInterrupts();

#if 0
  if (GPIOB->IDR & GPIO_Pin_3)
    GPIO_LOW(GPIOB, GPIO_Pin_6);
  else
    GPIO_HIGH(GPIOB, GPIO_Pin_6);

  if (GPIOB->IDR & GPIO_Pin_4)
    GPIO_LOW(GPIOB, GPIO_Pin_7);
  else
    GPIO_HIGH(GPIOB, GPIO_Pin_7);
#else
  {
    static int state = 0;
    if (state & 1) GPIO_LOW(GPIOB, GPIO_Pin_6);
    else GPIO_HIGH(GPIOB, GPIO_Pin_6);
    state ^= 1;
  }
#endif

  EXTI_ClearITPendingBit(EXTI_Line21);
  /* EXTI_ClearITPendingBit(EXTI_Line22); for comparator 2 */

  enableInterrupts();
}

static inline void set_dac1_mv(uint32_t mv)
{
  /* mv the millivolts */

  /* vref in millivolts */
  /* #define CONFIG_VREF 5000 */
#define CONFIG_VREF 3000

  /* resolution in bits */
#define CONFIG_DAC_RES 12

  const uint16_t n = (mv * (1 << CONFIG_DAC_RES)) / CONFIG_VREF;
  DAC_SetChannel1Data(DAC_Align_12b_R, n);
}

static void setup_encoders(void)
{
  COMP_InitTypeDef COMP_Init;
  GPIO_InitTypeDef GPIO_InitStructure;
  DAC_InitTypeDef DAC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* setup interrupt line */

  EXTI_InitStructure.EXTI_Line = EXTI_Line21;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = COMP_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  /* setup the dac for voltage comparison */

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  DAC_StructInit(&DAC_InitStructure);
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Cmd(DAC_Channel_1, ENABLE);

/*   set_dac1_mv(3800); */
  set_dac1_mv(500);

  /* left encoder */

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  COMP_Init.COMP_Speed = COMP_Speed_Slow;
  COMP_Init.COMP_InvertingInput = COMP_InvertingInput_DAC1;
  COMP_Init.COMP_OutputSelect = COMP_OutputSelect_None;

  COMP_Cmd(ENABLE);

  /* enable switch control */
  SYSCFG_RISwitchControlModeCmd();
  /* close VCOMP switch */
  SYSCFG_RIIOSwitchConfig();
  /* close io switch number n */
  SYSCFG_RIIOSwitchConfig();
}

#endif


void main(void)
{
  /* Configure Clocks for Application need */
  RCC_Configuration();
  
  /* Configure RTC Clocks */
  RTC_Configuration();

  setup_leds();
  GPIO_LOW(GPIOB, GPIO_Pin_6);
  GPIO_HIGH(GPIOB, GPIO_Pin_7);

#if 0
  setup_encoders();
#endif

#if 0
  uint8_t old_state = 0;
  while (1)
  {
    uint8_t new_state = COMP_GetOutputLevel(COMP_Selection_COMP1);
    if (new_state == old_state) continue ;
    COMP_IRQHandler();
    old_state = new_state;
  }
#endif

#if 1
  setup_pwms();

  /* motor enabling signal */
  {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_HIGH(GPIOB, GPIO_Pin_15);
  }

#if 0
  {
    static const unsigned int duties[] = { 0, 50, 100, 150 };
    unsigned int i = 0;
    while (1)
    {
      set_rpwm(duties[(i++) & 3]);
      delay();
    }
  }
#else

  set_lpwm(100);
  set_rpwm(100);

#endif

#endif

#if 0
  mcs_init();
#endif

#if 0 /* toremove */
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
#endif /* toremove */

  enableInterrupts();

  while (1) ;
}
