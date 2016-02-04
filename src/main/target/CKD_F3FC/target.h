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

// Define DEBUG_BOARD if you have a breakout board above the main fc board.
//#define DEBUG_BOARD

//#define V3_BOARD

#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_13
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOC
//#define LED0_INVERTED
#define LED1_GPIO   GPIOC
#define LED1_PIN    Pin_14
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOC
//#define LED1_INVERTED
#define LED2_GPIO   GPIOC
#define LED2_PIN    Pin_15
#define LED2_PERIPHERAL RCC_AHBPeriph_GPIOC
//#define LED2_INVERTED

// #ifdef DEBUG_BOARD
// #define BEEP_GPIO   GPIOE
// #define BEEP_PIN    Pin_5
// #define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOE
// #else
// #define BEEP_GPIO   GPIOA
// #define BEEP_PIN    Pin_14
// #define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOA
// #endif
// #define BEEPER_INVERTED
// #define BEEPER


#define USE_SPI
#ifdef V3_BOARD
  #define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOB
  #define MPU6500_CS_GPIO                  GPIOB
  #define MPU6500_CS_PIN                   GPIO_Pin_12
  #define MPU6500_SPI_INSTANCE             SPI2
#else
  #define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOA
  #define MPU6500_CS_GPIO                  GPIOA
  #define MPU6500_CS_PIN                   GPIO_Pin_4
  #define MPU6500_SPI_INSTANCE             SPI1
#endif
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
// TODO(tannewt): Support SPI 3.

#ifdef DEBUG_BOARD
#define USABLE_TIMER_CHANNEL_COUNT 15
#else
#define USABLE_TIMER_CHANNEL_COUNT 16
#endif  // DEBUG_BOARD

#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#ifdef V3_BOARD
  #define GYRO_MPU6500_ALIGN CW180_DEG
#else
  #define GYRO_MPU6500_ALIGN CW0_DEG
#endif

//#define USE_FAKE_GYRO

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#ifdef V3_BOARD
  #define ACC_MPU6500_ALIGN CW180_DEG
#else
  #define ACC_MPU6500_ALIGN CW0_DEG
#endif

// MPU6500 interrupt
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

// TODO(tannewt): Insert baro defines here.
// TODO(tannewt): Insert mag defines here.

//#define BEEPER
#define LED0
#define LED1
#define LED2

#define USE_VCP
#define USE_USART1
#define USE_USART2
#define USE_USART3
// TODO(tannewt): Support UART4 and UART5.
// TODO(tannewt): Support softserial if needed.
#define SERIAL_PORT_COUNT 4

#define UART1_TX_PIN        GPIO_Pin_8 // PD8
#define UART1_RX_PIN        GPIO_Pin_9 // PD9
#define UART1_GPIO          GPIOD
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource8
#define UART1_RX_PINSOURCE  GPIO_PinSource9

#define UART2_TX_PIN        GPIO_Pin_5 // PD5
#define UART2_RX_PIN        GPIO_Pin_6 // PD6
#define UART2_GPIO          GPIOD
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource5
#define UART2_RX_PINSOURCE  GPIO_PinSource6

#define UART3_TX_PIN        GPIO_Pin_0 // PE0
#define UART3_RX_PIN        GPIO_Pin_1 // PE1
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOE
#define UART3_TX_PINSOURCE  GPIO_PinSource0
#define UART3_RX_PINSOURCE  GPIO_PinSource1

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

#define I2C2_SCL_GPIO        GPIOA
#define I2C2_SCL_GPIO_AF     GPIO_AF_4
#define I2C2_SCL_PIN         GPIO_Pin_9
#define I2C2_SCL_PIN_SOURCE  GPIO_PinSource9
#define I2C2_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOA
#define I2C2_SDA_GPIO        GPIOA
#define I2C2_SDA_GPIO_AF     GPIO_AF_4
#define I2C2_SDA_PIN         GPIO_Pin_10
#define I2C2_SDA_PIN_SOURCE  GPIO_PinSource10
#define I2C2_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOA

// #define USE_ADC
//
// #define ADC_INSTANCE                ADC1
// #define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
// #define ADC_DMA_CHANNEL             DMA1_Channel1
//
// #define VBAT_ADC_GPIO               GPIOC
// #define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
// #define VBAT_ADC_CHANNEL            ADC_Channel_6

#define BLACKBOX
#define GPS
#define GTUNE
//#define LED_STRIP
//#define LED_STRIP_TIMER TIM16
#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI

// This is the pass through for ESCs.
// #define USE_SERIAL_1WIRE
//
// // STM32F3DISCOVERY TX - PD5 connects to UART RX
// #define S1W_TX_GPIO         GPIOD
// #define S1W_TX_PIN          GPIO_Pin_5
// // STM32F3DISCOVERY RX - PD6 connects to UART TX
// #define S1W_RX_GPIO         GPIOD
// #define S1W_RX_PIN          GPIO_Pin_6
