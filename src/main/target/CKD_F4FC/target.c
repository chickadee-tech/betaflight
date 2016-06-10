
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    // These are the TIM pins on the butterfly connector.
    PWM1  | (MAP_TO_PPM_INPUT << 8), // debug swclk
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8), // PPM input
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
    #ifdef V4_BOARD
        { TIM8,  IO_TAG(PC8),  TIM_Channel_3, TIM8_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_TIM8 },
        { TIM3,  IO_TAG(PC9),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_TIM3 },
        { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_TIM2 },
        { TIM11, IO_TAG(PB9),  TIM_Channel_1, TIM1_TRG_COM_TIM11_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM11 },
    #else
      // Don't claim the serial debug clock line when we have a debug board.
      #ifndef DEBUG_BOARD
          { TIM9,  IO_TAG(PE6),  TIM_Channel_2, TIM1_BRK_TIM9_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_TIM9 },
      #endif
          { TIM11, IO_TAG(PB9),   TIM_Channel_1, TIM1_TRG_COM_TIM11_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM11 },
          { TIM10, IO_TAG(PB8),   TIM_Channel_1, TIM1_UP_TIM10_IRQn,      1, IOCFG_AF_PP, GPIO_AF_TIM10 },
          { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_TIM3 },
    #endif
    // Internal timer group for the first four outputs.
    { TIM1,  IO_TAG(PE9),   TIM_Channel_1, TIM1_UP_TIM10_IRQn,      1, IOCFG_AF_PP, GPIO_AF_TIM1 },
    { TIM1,  IO_TAG(PE11),  TIM_Channel_2, TIM1_UP_TIM10_IRQn,      1, IOCFG_AF_PP, GPIO_AF_TIM1 },
    { TIM1,  IO_TAG(PE13),  TIM_Channel_3, TIM1_UP_TIM10_IRQn,      1, IOCFG_AF_PP, GPIO_AF_TIM1 },
    { TIM1,  IO_TAG(PE14),  TIM_Channel_4, TIM1_UP_TIM10_IRQn,      1, IOCFG_AF_PP, GPIO_AF_TIM1 },

    // TIMG1_CH1-4
    #ifdef V4_BOARD
      { TIM5, IO_TAG(PA0), TIM_Channel_1, TIM5_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM5 },
      { TIM5, IO_TAG(PA1), TIM_Channel_2, TIM5_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM5 },
      { TIM5, IO_TAG(PA2), TIM_Channel_3, TIM5_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM5 },
      { TIM5, IO_TAG(PA3), TIM_Channel_4, TIM5_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM5 },
    #else
      { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM2_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM2 },
      { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM2 },
      { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM2_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM2 },
      { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM2_IRQn,                  1, IOCFG_AF_PP, GPIO_AF_TIM2 },
    #endif

    // TIMG2_CH1-4
    { TIM4, IO_TAG(PD15),  TIM_Channel_1, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_TIM4 },
    { TIM4, IO_TAG(PD14),  TIM_Channel_2, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_TIM4 },
    { TIM4, IO_TAG(PD13),  TIM_Channel_3, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_TIM4 },
    { TIM4, IO_TAG(PD12),  TIM_Channel_4, TIM4_IRQn,                1, IOCFG_AF_PP, GPIO_AF_TIM4 },
};
