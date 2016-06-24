/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// The Chickadee F3FC has the standard FC Stack Butterfly connector. Here is the
// pin mapping for it:
// Pin  1 - TIM1     - PA14
// Pin  2 - TIM2     - PB9
// Pin  3 - TIM3     - PB8
// Pin  4 - TIM4     - PB10
// Pin  5 - GPIO1    - PA13
// Pin  6 - GPIO2    - PE5
// Pin  7 - GPIO3    - PE4
// Pin  8 - GPIO4    - PE3
// Pin  9 - GPIO5    - PE2
// Pin 10 - GPIO6    - PB7
// Pin 11 - i2c_SDA  - PA10
// Pin 12 - i2c_SCL  - PA9
// Pin 13 - HEIGHT_4 - NC
// Pin 14 - HEIGHT_2 - NC
// Pin 15 - HEIGHT_1 - NC
// Pin 16 - 3V3      - VSS
// Pin 17 - 3V3E     - NC
// Pin 18 - +Batt    - PC0
// Pin 19 - 24 - 5V
// Pin 25 - UART8_TX - NC
// Pin 26 - UART8_RX - NC
// Pin 27 - UART7_TX - PB10 (softserial)
// Pin 28 - UART7_RX - PB11 (softserial)
// Pin 29 - UART6_TX - PA2 (softserial)
// Pin 30 - UART6_RX - PA3 (softserial)
// Pin 31 - UART5_TX - PC12 (uart5)
// Pin 32 - UART5_RX - PD2 (uart5)
// Pin 33 - UART4_TX - PC10 (uart4)
// Pin 34 - UART4_RX - PC11 (uart4)
// Pin 35 - UART3_TX - PE1 (usart1)
// Pin 36 - UART3_RX - PE0 (usart1)
// Pin 37 - UART2_TX - PD5 (usart2)
// Pin 38 - UART2_RX - PD6 (usart2)
// Pin 39 - UART1_TX - PD8 (usart3)
// Pin 40 - UART1_RX - PD9 (usart3)

// Outside pins
// Pin 41 - TIMG1_CH1 - PC9 (TIM3_CH4)
// Pin 42 - TIMG1_CH2 - PC8 (TIM3_CH3)
// Pin 43 - TIMG1_CH3 - PC7 (TIM3_CH2)
// Pin 44 - TIMG1_CH4 - PC6 (TIM3_CH1)
// Pin 45 - TIMG2_CH1 - PD15 (TIM4_CH4)
// Pin 46 - TIMG2_CH2 - PD14 (TIM4_CH3)
// Pin 47 - TIMG2_CH3 - PD13 (TIM4_CH2)
// Pin 48 - TIMG2_CH4 - PD12 (TIM4_CH1)
// Pin 49 - ADC1      - PC1
// Pin 50 - ADC2      - PC2
// Pin 51 - SDMMC_D0  - NC
// Pin 52 - SDMMC_D1  - NC
// Pin 53 - SDMMC_D2  - NC
// Pin 54 - SDMMC_D3  - NC
// Pin 55 - SDMMC_CLK - NC
// Pin 56 - SDMMC_CMD - NC
// Pin 57 - 64        - GND
// Pin 65 - BOOT0     - BOOT0
// Pin 66 - Reset     - NRST
// Pin 67 - CAN_HI    - NC
// Pin 68 - CAN_LO    - NC
// Pin 69 - SPI3_NSS  - NC
// Pin 70 - SPI3_SCK  - NC
// Pin 71 - SPI3_MISO - NC
// Pin 72 - SPI3_MOSI - NC
// Pin 73 - SPI2_NSS  - PA15 (SPI3)
// Pin 74 - SPI2_SCK  - PB3 (SPI3)
// Pin 75 - SPI2_MISO - PB4 (SPI3)
// Pin 76 - SPI2_MOSI - PB5 (SPI3)
// Pin 77 - SPI1_NSS  - PB12 (SPI2)
// Pin 78 - SPI1_SCK  - PB13 (SPI2)
// Pin 79 - SPI1_MISO - PB14 (SPI2)
// Pin 80 - SPI1_MOSI - PB15 (SPI2)

#define TARGET_BOARD_IDENTIFIER "CKD3"

#define CONFIG_FASTLOOP_PREFERRED_ACC 1

// Define DEBUG_BOARD if you have a breakout board above the main fc board.
//#define DEBUG_BOARD

//#define V3_BOARD
//#define V4_BOARD
#define V5_BOARD

#define USE_POLYSTACK
#define POLYSTACK_I2C_INSTANCE I2CDEV_2

#define LED0   PC13
#define LED1   PC14
#define LED2   PC15

#ifdef DEBUG_BOARD
#define BEEPER    PA14
#else
#define BEEPER    PA13
#endif
#define BEEPER_INVERTED

#define SPEKTRUM_BIND
#define BIND_PIN   PD8

#define USE_SPI
#ifdef V3_BOARD
  #define MPU6500_CS_PIN                   PB12
  #define MPU6500_SPI_INSTANCE             SPI2
#else  // V4
  #define MPU6500_CS_PIN                   PA4
  #define MPU6500_SPI_INSTANCE             SPI1
#endif

// V5
#define MPU6000_CS_PIN                   PA4
#define MPU6000_SPI_INSTANCE             SPI1

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
// TODO(tannewt): Support SPI 3.

#ifdef DEBUG_BOARD
#define USABLE_TIMER_CHANNEL_COUNT 15
#else
#define USABLE_TIMER_CHANNEL_COUNT 16
#endif  // DEBUG_BOARD

#define GYRO
// V5
#define USE_GYRO_MPU6000
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW0_DEG

// <= V4
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#ifdef V3_BOARD
  #define GYRO_MPU6500_ALIGN CW180_DEG
#else
  #define GYRO_MPU6500_ALIGN CW0_DEG
#endif

//#define USE_FAKE_GYRO

#define ACC

// V5
#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW0_DEG

// <= V4
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#ifdef V3_BOARD
  #define ACC_MPU6500_ALIGN CW180_DEG
#else
  #define ACC_MPU6500_ALIGN CW0_DEG
#endif

// TODO(tannewt): Insert baro defines here.
// TODO(tannewt): Insert mag defines here.

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
// TODO(tannewt): Support softserial if needed.
#define SERIAL_PORT_COUNT 6

#define UART1_TX_PIN        PE0
#define UART1_RX_PIN        PE1

#define UART2_TX_PIN        PD5
#define UART2_RX_PIN        PD6

#define UART3_TX_PIN        PD8
#define UART3_RX_PIN        PD9

#define UART4_TX_PIN        PC10
#define UART4_RX_PIN        PC11

#define UART5_TX_PIN        PC12
#define UART5_RX_PIN        PD2

// #define INVERTER
// #define INVERTER_PIN Pin_4
// #define INVERTER_GPIO GPIOE
// #define INVERTER_PERIPHERAL RCC_AHBPeriph_GPIOE
// #define INVERTER_USART USART2

#define POLYSTACK_SERIAL_PORT_ORDER {SERIAL_PORT_USART3, SERIAL_PORT_USART2, SERIAL_PORT_USART1, SERIAL_PORT_USART4, SERIAL_PORT_USART5}
#define POLYSTACK_GPIO_PORT_ORDER {IO_TAG(PA13), IO_TAG(PA14), IO_TAG(PE4), IO_TAG(PE3), IO_TAG(PE2), IO_TAG(PB7)}

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

#define I2C2_SCL         PA9
#define I2C2_SDA         PA10

#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SDCARD
#define USE_SDCARD_SPI2

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PE4

#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_GPIO                  SPI2_GPIO
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

// Note, this is the same DMA channel as USART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA1_FLAG_TC5

#define USE_ADC

#define ADC_INSTANCE                ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

#define VBAT_ADC_PIN                PC0
#define CURRENT_METER_ADC_PIN    PC3

#define USE_EXTI

#ifdef V3_BOARD
  #ifdef DEBUG_BOARD
    #define MPU_INT_EXTI PE5
  #else
    #define MPU_INT_EXTI PA14
  #endif
#else
  #define MPU_INT_EXTI PB0
#endif
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define BLACKBOX
#define GPS
//#define GTUNE
//#define LED_STRIP
//#define LED_STRIP_TIMER TIM16
#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - 303 in 100pin package
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0x00ff

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(15) | TIM_N(16) |TIM_N(17))
