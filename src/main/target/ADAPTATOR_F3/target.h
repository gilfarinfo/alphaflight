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

#if defined(RMDO)
#define TARGET_BOARD_IDENTIFIER "RMDO"
#elif defined(ZCOREF3)
#define TARGET_BOARD_IDENTIFIER "ZCF3"
#elif defined(FLIP32F3OSD)
#define TARGET_BOARD_IDENTIFIER "FLF3"
#elif defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define TARGET_BOARD_IDENTIFIER "SRF3"
#elif defined(AFROMINI)
#define TARGET_BOARD_IDENTIFIER "AFMN"
#elif defined(BEEBRAIN)
#define TARGET_BOARD_IDENTIFIER "BEBR"
#elif defined(ADAPTATOR_F1) || defined(NAZE)
#define TARGET_BOARD_IDENTIFIER "AFNA"
#else
#error "Board not defined"
#endif

//######## Type of Default Configuration ##########
#if defined(ADAPTATOR_F1) || defined(NAZE)
#define TARGET_CONFIG
#define USE_HARDWARE_REVISION_DETECTION
#define TARGET_BUS_INIT
#elif ADAPTATOR_F3 || defined(SPRACINGF3)
#elif defined(BEEBRAIN)
#define TARGET_CONFIG
#else
#endif

//###################### LED ######################
#if defined(ZCOREF3)
#define LED0                    PB8
#elif ADAPTATOR_F1
#define LED0                    PB3
#elif ADAPTATOR_F3
#define LED0                    PB3
#else
#define LED0                    PB3
#endif

#ifdef ADAPTATOR_F1
#define LED1                    PB4
#elif ADAPTATOR_F3
#else
#endif

#define LED_STRIP

//#######################  BEEPER ###########################
// Beeper configuration is handled in 'config.c', since it is dependent on hardware revision
#if defined(AFROMINI)
#define BEEPER_INVERTED
#elif defined(ADAPTATOR_F1) || defined(NAZE)
#define BEEPER                  PA12
#elif defined(ADAPTATOR_F3) || defined(SPRACINGF3)
#define BEEPER                  PC15
#define BEEPER_INVERTED
#else
#endif

//################## BRUSLESS OR BRUSHED ####################
#if defined(AFROMINI)
#elif defined(BEEBRAIN)
#define BRUSHED_MOTORS
#define TARGET_CONFIG
#define DEFAULT_FEATURES FEATURE_MOTOR_STOP
#else
#endif

#if !defined(BRUSHED_MOTORS)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

//############# CONFIG FAST LOOP ###########################
#if defined(ZCOREF3)
#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT
#elif (ADAPTATOR_F3) || defined(FLIP32F3OSD)
#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE
#elif (ADAPTATOR_F1) || defined(NAZE)
#undef CONFIG_FASTLOOP_PREFERRED_ACC
#else
#endif

//###################### SPI ######################
#if defined(NAZE) || defined(ADAPTATOR_F1)
#define USE_SPI
#define USE_SPI_DEVICE_2
// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

#define NAZE_SPI_INSTANCE       SPI2
#define NAZE_SPI_CS_PIN         PB12

#elif defined (SPRACINGF3) || defined(ADAPTATOR_F3)
#define USE_SPI
#define USE_SPI_DEVICE_1 // PB9,3,4,5 on AF5 SPI1 (MPU)
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI1_NSS_PIN            PB9
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

#else
#endif

//###################### UART ######################
#define USE_UART1
#define USE_UART2
#if defined(SPRACINGF3) || defined(ADAPTATOR_F3)
/* only 2 uarts available on the NAZE, add ifdef here if present on other boards */
#define USE_UART3

#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#endif

#if defined(ZCOREF3)
#define SERIAL_PORT_COUNT       3

#elif(FLIP32F3OSD) || defined(ADAPTATOR_F3)
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       5
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       4
#else
#endif

#if defined(USE_SOFTSERIAL1)
#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4 // PWM 5
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5 // PWM 6
#endif

#if defined(USE_SOFTSERIAL2)
#define SOFTSERIAL_2_TIMER TIM3
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6 // PWM 7
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7 // PWM 8
#endif

#if defined(NAZE) || defined(ADAPTATOR_F1)
#define INVERTER_PIN_USART2       PB2 // PB2 (BOOT1) abused as inverter select GPIO
#else
#endif

#if defined(SPRACINGF3) || defined(ADAPTATOR_F3)
// UART1 TX uses DMA1_Channel4, which is also used by dshot on motor 4
#if defined(USE_UART1_TX_DMA) && defined(USE_DSHOT)
#undef USE_UART1_TX_DMA
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)
#else
//STM32F10x default
    // UART1_TX    PA9
    // UART1_RX    PA10
    // UART2_TX    PA2
    // UART2_RX    PA3
#endif

//###################### I2C ######################
#define USE_I2C
#if defined(NAZE) || defined(ADPATATOR_F1)
#define I2C_DEVICE (I2CDEV_2)
// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67
#elif defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

#else
#endif

//###################### ADC ######################
#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#if defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define ADC_INSTANCE            ADC2
#elif defined(NAZE) || defined(ADPATATOR_F1)
#else
#endif

#if defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5
#define RSSI_ADC_PIN            PB2
#elif defined(NAZE) || defined(ADPATATOR_F1)
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PB1
#define RSSI_ADC_PIN            PA1
#else
#endif

#if defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#elif defined(NAZE) || defined(ADPATATOR_F1)
#define EXTERNAL1_ADC_PIN       PA5
#else
#endif

//################## EXTI #########################
#define USE_EXTI

#if defined(ZCOREF3)
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1
#elif defined(FLIP32F3OSD)
#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU data ready and MAG data ready
#elif defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#else
#endif

//##################### FLASHFS ######################
#define USE_FLASHFS
#define USE_FLASH_M25P16

#if defined(NAZE) || defined(ADAPTATOR_F1)
// We either have this 16mbit flash chip on SPI or the MPU6500 acc/gyro depending on board revision:
#define M25P16_CS_PIN           NAZE_SPI_CS_PIN
#define M25P16_SPI_INSTANCE     NAZE_SPI_INSTANCE
#elif defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2
#else
#endif

//################## BARO #########################
//#define BARO_XCLR_PIN           PC13
//#define BARO_EOC_PIN            PC14
#define BARO
#if defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define USE_BARO_MS5611 // needed for Flip32 board
#define USE_BARO_BMP280
#else
#endif

//################## MAG #########################
#if defined(FLIP32F3OSD)
#elif defined(RMDO)
#elif defined(ZCOREF3)
#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define MAG_INT_EXTI            PC14
//#define DEBUG_MAG_DATA_READY_INTERRUPT
#define USE_MAG_DATA_READY_SIGNAL
/*
#define MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG
*/

#elif defined(SPRACING_F3) || defined(ADAPTATOR_F3)
#define MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW270_DEG
#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH
#define MAG_INT_EXTI            PC14
#else
#endif

//################## SONAR #########################
//#define SONAR
//#define SONAR_TRIGGER_PIN       PB0
//#define SONAR_ECHO_PIN          PB1
//#define SONAR_TRIGGER_PIN_PWM   PB8
//#define SONAR_ECHO_PIN_PWM      PB9
#if defined(FLIP32F3OSD)
#define SONAR
#define SONAR_TRIGGER_PIN       PB0
#define SONAR_ECHO_PIN          PB1
#elif defined(RMDO)
#elif defined(ZCOREF3)
#elif defined(SPRACINGF3) || defined(ADPATATOR_F3)
#define SONAR_SOFTSERIAL2_EXCLUSIVE
#elif defined(NAZE) || defined(ADPATATOR_F1)
#else 
#endif

//################## GPS #########################
#if defined(FLIP32F3OSD)
#elif defined(RMDO)
#undef USE_GPS
#elif defined(ZCOREF3)
#elif defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#undef GPS
#else
#endif

//################## SPEKTRUM #########################
#define SPEKTRUM_BIND
#if defined(FLIP32F3OSD) || defined(RMDO) || defined(ZCOREF3) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
// USART3,
#define BIND_PIN                PB11
#elif defined(NAZE) || defined(ADAPTATOR_F1)
// USART2, PA3
#define BIND_PIN                PA3
#else
#endif

//####################### MPU6500 PIN and other ########################
#define MPU_INT_EXTI            PC13
#if defined(FLIP32F3OSD)
#elif defined(ZCOREF3)
#define MPU6500_CS_PIN          PB9
#define MPU6500_SPI_INSTANCE    SPI1
#elif defined(RMDO) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define MPU6500_CS_PIN          NAZE_SPI_CS_PIN
#define MPU6500_SPI_INSTANCE    NAZE_SPI_INSTANCE
#else
#endif
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#if defined(FLIP32F3OSD) || defined(RMDO) || defined(ZCOREF3) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define ENSURE_MPU_DATA_READY_IS_LOW
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#else
#endif

//####################### ACC ########################
#define ACC
#if defined(FLIP32F3OSD)
#define USE_ACC_MPU6500
#define ACC_MPU6500_ALIGN CW270_DEG
#elif defined(ZCOREF3)
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG
#elif defined(RMDO) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW270_DEG
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define USE_ACC_ADXL345
#define USE_ACC_BMA280
#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_ADXL345_ALIGN       CW270_DEG
#define ACC_MPU6050_ALIGN       CW0_DEG
#define ACC_MMA8452_ALIGN       CW90_DEG
#define ACC_BMA280_ALIGN        CW0_DEG
#define ACC_MPU6500_ALIGN       CW0_DEG
#else
#endif

//####################### GYRO ########################
#define GYRO
#if defined(FLIP32F3OSD)
#define USE_GYRO_MPU6500
#define GYRO_MPU6500_ALIGN CW270_DEG
#elif defined(ZCOREF3)
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW180_DEG
#define MPU6500_CS_PIN          PB9
#define MPU6500_SPI_INSTANCE    SPI1
#elif defined(RMDO) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW270_DEG
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU3050_ALIGN      CW0_DEG
#define GYRO_MPU6050_ALIGN      CW0_DEG
#define GYRO_MPU6500_ALIGN      CW0_DEG
#else
#endif

//##################### DIVERS ###################
#if defined(FLIP32F3OSD) || defined(RMDO) || defined(ZCOREF3) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define USE_ESC_SENSOR
#define REMAP_TIM17_DMA
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#else
#endif

//######################## Feature config default #####################
#if defined(FLIP32F3OSD) || defined(RMDO) || defined(ZCOREF3) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        (FEATURE_TRANSPONDER | FEATURE_BLACKBOX | FEATURE_RSSI_ADC | FEATURE_CURRENT_METER | FEATURE_TELEMETRY)
#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#else
#endif

//############################ TIMERS ##########################
#if defined(FLIP32F3OSD) || defined(RMDO) || defined(ZCOREF3) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) | TIM_N(17) )

#elif defined(NAZE) || defined(ADAPTATOR_F1)
#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )

#else
#endif

//############################ PORT PACKAGE ##########################
#if defined(FLIP32F3OSD) || defined(RMDO) || defined(ZCOREF3) || defined(SPRACINGF3) || defined(ADAPTATOR_F3)
// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#elif defined(NAZE) || defined(ADAPTATOR_F1)
// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#else
#endif
