
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    // These are the TIM pins on the butterfly connector.
    PWM1  | (MAP_TO_PPM_INPUT << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),

    // These are the four through-hole headers on the board.
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),

    // These are TIMG1. They can be anything.
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),

    // These are TIMG2. They can be anything.
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    // These are the TIM pins on the butterfly connector.
    PWM1  | (MAP_TO_SERVO_OUTPUT << 8), // PPM input
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),

    // These are the four through-hole headers on the board.
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),

    // These are TIMG1. They can be anything.
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10 | (MAP_TO_PWM_INPUT << 8),
    PWM11 | (MAP_TO_PWM_INPUT << 8),
    PWM12 | (MAP_TO_PWM_INPUT << 8),

    // These are TIMG2. They can be anything.
    PWM13 | (MAP_TO_PWM_INPUT << 8),
    PWM14 | (MAP_TO_PWM_INPUT << 8),
    PWM15 | (MAP_TO_PWM_INPUT << 8),
    PWM16 | (MAP_TO_PWM_INPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    // TODO
    0xFFFF
};

const uint16_t airPWM[] = {
    // TODO
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // TIM1-4
// Don't claim the serial debug clock line when we have a debug board.
#ifdef V4_BOARD
  #ifndef DEBUG_BOARD
      { TIM8,  IO_TAG(PA14),  TIM_Channel_2, TIM8_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_5 },
  #endif
      { TIM17, IO_TAG(PB9),   TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP, GPIO_AF_1 },
      { TIM16, IO_TAG(PB8),   TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_1 },
      { TIM2,  IO_TAG(PB10),  TIM_Channel_3, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1 },
#else
    { TIM16, IO_TAG(PB8),   TIM_Channel_1, TIM1_UP_TIM16_IRQn,      0, IOCFG_AF_PP, GPIO_AF_1 },
    { TIM17, IO_TAG(PB9),   TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP, GPIO_AF_1 },
    { TIM2,  IO_TAG(PB10),  TIM_Channel_3, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1 },
    { TIM15,  IO_TAG(PA3),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, GPIO_AF_9 },
#endif

    // Internal timer group for the first four outputs.
    { TIM1,  IO_TAG(PE9),   TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM1,  IO_TAG(PE11),  TIM_Channel_2, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM1,  IO_TAG(PE13),  TIM_Channel_3, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM1,  IO_TAG(PE14),  TIM_Channel_4, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_2 },

    // TIMG1_CH1-4
    { TIM3, IO_TAG(PC9), TIM_Channel_4, TIM3_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM3, IO_TAG(PC8), TIM_Channel_3, TIM3_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM3, IO_TAG(PC7), TIM_Channel_2, TIM3_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM3, IO_TAG(PC6), TIM_Channel_1, TIM3_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_2 },

    // TIMG2_CH1-4
    { TIM4, IO_TAG(PD15),  TIM_Channel_4, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM4, IO_TAG(PD14),  TIM_Channel_3, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM4, IO_TAG(PD13),  TIM_Channel_2, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM4, IO_TAG(PD12),  TIM_Channel_1, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_2 },
};
