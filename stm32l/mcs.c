/* mcs, motor controlling system */

#include "stm32l1xx_exti.h"
#include "stm32l1xx_nvic.h"
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
#include "stm32l1xx_tim.h"
#include "misc.h"
#include "stdint.h"


/* globals */

#define CONFIG_START_POWER_FWD 240
#define CONFIG_TICKS_PER_CM 3
#define CONFIG_SPEED_INTEGRATOR 2
#define CONFIG_DEFAULT_SPEED 128

static uint32_t dir; /* forward direction */
static uint32_t ldir, rdir; /* motors independant directions */
static uint32_t speed; /* speed */
static uint32_t lpwm, rpwm; /* motor pwms */
static uint32_t lenc, renc; /* encoding counters */
static uint32_t lodo, rodo; /* odometer counters */


/* pwms */

#define CONFIG_PWM_LCHAN 0
#define CONFIG_PWM_RCHAN 1

static void set_pwm(unsigned int duty, unsigned int chan)
{
  TIM_OCInitTypeDef TIM_OCInit;

  TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInit.TIM_Pulse = duty;
  TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;

  if (chan == CONFIG_PWM_LCHAN)
  {
    TIM_OC1Init(TIM3, &TIM_OCInit);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  }
  else
  {
    TIM_OC2Init(TIM3, &TIM_OCInit);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  }
}

static inline void set_lpwm(unsigned int duty)
{
  set_pwm(duty, CONFIG_PWM_LCHAN);
}

static inline void set_rpwm(unsigned int duty)
{
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
  TIM_TimeBase.TIM_Period = 0x100 - 1; /* 10 khz */
  TIM_TimeBase.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBase);

  set_lpwm(100);
  set_rpwm(100);

  TIM_Cmd(TIM3, ENABLE);
}


/* helper macros */

#define enableInterrupts() __set_PRIMASK(0)
#define disableInterrupts() __set_PRIMASK(1)


/* encoder isr handler */

void EXTI0_IRQHandler(void)
{
  disableInterrupts();

  if (GPIOB->IDR & GPIO_Pin_13)
  {
    ++lenc;
    ++lodo;
  }
  else if (GPIOB->IDR & GPIO_Pin_14)
  {
    ++renc;
    ++rodo;
  }

  EXTI_ClearITPendingBit(EXTI_Line0);

  enableInterrupts();
}


/* tim2 interrupt handler */

static inline int32_t clamp(int32_t n, int32_t lo, int32_t hi) 
{
  if (n < lo) return lo;
  else if (n > hi) return hi;
  return n;
}

void TIM2_IRQHandler(void)
{
  /* 10hz frequency */

  /* todo: reduce the scope by capturing variables */
  disableInterrupts();

  /* forward */
  if ((dir != 1) || (speed <= 0)) goto on_done;

  if (lenc < speed) lpwm += 2; /* I_up */
  else if (lenc > speed) lpwm -= 2;

  if (renc < speed) rpwm += 2;
  else if (renc > speed) rpwm -= 2;

  err = clamp(err + lenc - renc, -128, 128);

  /* reset encoder counters */
  lenc = 0;
  renc = 0;

  lpwm = clamp(CONFIG_START_POWER_FWD + lpwm - err, 0, 255);
  rpwm = clamp(CONFIG_START_POWER_FWD + rpwm - err, 0, 255);

  set_lpwm(lpwm);
  set_rpwm(rpwm);

 on_done:
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  enableInterrupts();
}


/* motors routines.
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


/* general purpose timer2 setup
 */

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



/* mcs initialisation
 */

void mcs_init(void)
{
  /* todo: init globals */
  /* todo: encoder interrupts */

  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* init globals */
  dir = 0;
  ldir = 0;
  rdir = 0;
  speed = 128;
  lpwm = 128;
  rpwm = 128;
  lenc = 0;
  renc = 0;
  lodo = 0;
  rodo = 0;

  /* setup motors pins */
#if 0 /* todo */
  setup_motors();
#endif

  /* setup pins as input + exti  */

  /* fixme: what if already enabled */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* connect PORTB to interrupt line 0 */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

  /* exti from gpio state change */
  EXTI_ClearITPendingBit(EXTI_Line0);
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void mcs_stop(void)
{
  disableInterrupts();

  set_lpwm(128);
  set_rpwm(128);

  enableInterrupts();
}


static void do_encoder(uint32_t enc)
{
  uint32_t i;
  int32_t lspeed, rspeed;
  int32_t diff;

  /* reset odometer counters */
  rodo = 0;
  lodo = 0;

  /* set pwms from speed */
  rspeed = clamp(speed / 2, 0, 128);
  lspeed = rspeed;
  set_pwm0(128 + (ldir == 1 ? lspeed : -lspeed));
  set_pwm1(128 + (rdir == 1 ? rspeed : -rspeed));

  /* wait until dont */
  i = 0;
  while (i < enc)
  {
    i += (lodo + rodo) / 2;

    diff = (int32_t)(lenc - renc);

    if (diff > 0)
    {
      /* left faster than right */
      if ((lspeed > speed) || (rspeed > 244))
	lspeed -= CONFIG_SPEED_INTEGRATOR;
      else
	rspeed += CONFIG_SPEED_INTEGRATOR;
    }
    else if (diff < 0)
    {
      /* right faster than left */
      if ((rspeed > speed) || (lspeed > 244))
	rspeed -= CONFIG_SPEED_INTEGRATOR;
      else
	lspeed += CONFIG_SPEED_INTEGRATOR;
    }

    /* reset the odometer */
    rodo = 0;
    lodo = 0;

    /* set motor pwms */
    lspeed = clamp(lspeed / 2, 0, 128);
    rspeed = clamp(rspeed / 2, 0, 128);
    set_pwm0(128 + (ldir == 1 ? lspeed : -lspeed));
    set_pwm1(128 + (rdir == 1 ? rspeed : -rspeed));

    /* small delay */
    {
      volatile int i;
      for (i = 0; i < 1000; ++i) __asm__ __volatile__ ("nop\n\t");
    }
  }

  /* stop the motor */
  set_pwm0(0);
  set_pwm1(0);
}


void mcs_turn(int32_t a)
{
  /* a the angle in degrees */

  /* degrees to ticks */
  uint32_t enc = (abs(a) * 10) / 37 - 3;

  disableInterrupts();

  /* turn left or right */
  if (a < 0)
  {
    ldir = 0;
    rdir = 1;
  }
  else
  {
    ldir = 1;
    rdir = 0;
  }

  do_encoder(enc);

  enableInterrupts();
}


void mcs_move(int32_t d)
{
  /* d the distance in cms */

  /* convert distance to tick */
  uint32_t enc = abs(d) * CONFIG_TICK_PER_CM;

  disableInterrupts();

  /* forward or backward direction */
  if (d < 0)
  {
    ldir = 0;
    rdir = 0;
  }
  else
  {
    ldir = 1;
    rdir = 1;
  }

  do_encoder();

  enableInterrupts();
}


void mcs_speed(uint32_t v)
{
  /* v the speed in [1:255] */

  disableInterrupts();

  speed = v;

  enableInterrupts();
}
