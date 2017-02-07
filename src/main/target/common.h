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
//Ordinary not used. But which use it ?
//#define USE_HAL_DRIVER

// type conversion warnings.
// -Wconversion can be turned on to enable the process of eliminating these warnings
//#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
// -Wpadded can be turned on to check padding of structs
//#pragma GCC diagnostic warning "-Wpadded"

//#define SCHEDULER_DEBUG // define this to use scheduler debug[] values. Undefined by default for performance reasons
#define DEBUG_MODE DEBUG_NONE // change this to change initial debug mode

#define I2C1_OVERCLOCK true
#define I2C2_OVERCLOCK true

//####################################################
#ifdef STM32F7
#define STM_FAST_TARGET
#define I2C3_OVERCLOCK true
#define I2C4_OVERCLOCK true
#define TELEMETRY_IBUS
#endif

/****************************
  STM32 F4 specific settings.
****************************/
//####################################################
#ifdef STM32F4
#define STM_FAST_TARGET
#define USE_DSHOT
#define I2C3_OVERCLOCK true
#define TELEMETRY_IBUS
#endif

//####################################################
#ifdef STM32F3
#define USE_DSHOT
#endif
//####################################################
#ifdef STM32F1
// Using RX DMA disables the use of receive callbacks
#define USE_UART1_RX_DMA
#define USE_UART1_TX_DMA

#define CLI_MINIMAL_VERBOSITY
#endif

//####################################################
#if defined(RMDO)
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
#define USE_PWM
#define USE_PPM
//####################################################
#elif defined(ZCOREF3)
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
#define USE_PWM
#define USE_PPM
//####################################################
#elif defined(FLIP32F3OSD)
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
#define USE_PWM
#define USE_PPM
//####################################################
#elif defined(SPRACINGF3)
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
#define USE_PWM
#define USE_PPM
//####################################################
#elif defined(ADAPTATOR_F3)
#define SERIAL_RX
//#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
//#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
//#define USE_SERIALRX_SUMD       // Graupner Hott protocol
//#define USE_SERIALRX_SUMH       // Graupner legacy protocol
//#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
//#define USE_PWM
#define USE_PPM
//####################################################
#elif defined(AFROMINI)
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
#define USE_PWM
#define USE_PPM
//####################################################
#elif defined(BEEBRAIN)
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
#define USE_PWM
#define USE_PPM
//####################################################
#elif defined(ADAPTATOR_F1) 
#define SERIAL_RX
//#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
//#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
//#define USE_SERIALRX_SUMD       // Graupner Hott protocol
//#define USE_SERIALRX_SUMH       // Graupner legacy protocol
//#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
//#define USE_PWM
#define USE_PPM
//####################################################
#elif  defined(NAZE)
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_CLI
#define USE_PWM
#define USE_PPM
//####################################################
#else
#error "Board not defined"
#endif



#if defined(STM_FAST_TARGET)
#define MAX_AUX_CHANNELS                99
#define TASK_GYROPID_DESIRED_PERIOD     125
#define SCHEDULER_DELAY_LIMIT           10
#else
#define MAX_AUX_CHANNELS                6
#define TASK_GYROPID_DESIRED_PERIOD     1000
#define SCHEDULER_DELAY_LIMIT           100
#endif

#if (FLASH_SIZE > 64)

//####################################################
#if defined(RMDO)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif defined(ZCOREF3)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif defined(FLIP32F3OSD)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif defined(SPRACINGF3)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif defined(ADAPTATOR_F3)
#define BLACKBOX
//#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
//#define TELEMETRY_HOTT
//#define TELEMETRY_LTM
//#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif defined(AFROMINI)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif defined(BEEBRAIN)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif defined(ADAPTATOR_F1) 
#define BLACKBOX
//#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
//#define TELEMETRY_HOTT
//#define TELEMETRY_LTM
//#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#elif  defined(NAZE)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_SERVOS
#define USE_RESOURCE_MGMT

//####################################################
#else
#error "Board not defined"
#endif

#endif

#if (FLASH_SIZE > 128)
	
//####################################################
#if defined(RMDO)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define TELEMETRY_CRSF
#define TELEMETRY_SRXL
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif defined(ZCOREF3)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define TELEMETRY_CRSF
#define TELEMETRY_SRXL
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif defined(FLIP32F3OSD)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define TELEMETRY_CRSF
#define TELEMETRY_SRXL
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif defined(SPRACINGF3)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define TELEMETRY_CRSF
#define TELEMETRY_SRXL
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif defined(ADAPTATOR_F3)
//#define CMS
//#define USE_DASHBOARD
//#define USE_MSP_DISPLAYPORT
//#define TELEMETRY_CRSF
//#define TELEMETRY_SRXL
//#define TELEMETRY_JETIEXBUS
//#define TELEMETRY_MAVLINK
#define USE_RX_MSP
//#define USE_SERIALRX_JETIEXBUS
//#define VTX_COMMON
//#define VTX_CONTROL
//#define VTX_SMARTAUDIO
//#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif defined(AFROMINI)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define TELEMETRY_CRSF
#define TELEMETRY_SRXL
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif defined(BEEBRAIN)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define TELEMETRY_CRSF
#define TELEMETRY_SRXL
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif defined(ADAPTATOR_F1) 
//#define CMS
//#define USE_DASHBOARD
//#define USE_MSP_DISPLAYPORT
//#define TELEMETRY_CRSF
//#define TELEMETRY_SRXL
//#define TELEMETRY_JETIEXBUS
//#define TELEMETRY_MAVLINK
#define USE_RX_MSP
//#define USE_SERIALRX_JETIEXBUS
//#define VTX_COMMON
//#define VTX_CONTROL
//#define VTX_SMARTAUDIO
//#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#elif  defined(NAZE)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define TELEMETRY_CRSF
#define TELEMETRY_SRXL
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#define USE_SENSOR_NAMES
//####################################################
#else
#error "Board not defined"
#endif




#else
#define SKIP_CLI_COMMAND_HELP
#endif

//*********************************************
//*********  DIMENSIONNEMENT ******************
//*********************************************

//####################################################
#if defined(RMDO)
#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif defined(ZCOREF3)
#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif defined(FLIP32F3OSD)
#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif defined(SPRACINGF3)
#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif defined(ADAPTATOR_F3)
#define MAX_SUPPORTED_MOTORS 4
#define MAX_SUPPORTED_SERVOS 2
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif defined(AFROMINI)
#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif defined(BEEBRAIN)
#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif defined(ADAPTATOR_F1) 
#define MAX_SUPPORTED_MOTORS 4
#define MAX_SUPPORTED_SERVOS 2
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#elif  defined(NAZE)
#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define RC_CHANNEL_COUNT 16 // standard variant of the protocol has 16 RC channels nrf24 i_nav

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

//####################################################
#else
#error "Board not defined"
#endif


/*
  C:\STM32Toolchain\quadri\quadri_dev\alphaflight\src\main\target\ALIENFLIGHTF1\target.h (1 hit)
	Line 23: #define BRUSHED_ESC_AUTODETECT
	#define TARGET_BOARD_IDENTIFIER "AFF1" 
	#define BRUSHED_ESC_AUTODETECT
  C:\STM32Toolchain\quadri\quadri_dev\alphaflight\src\main\target\ALIENFLIGHTF3\target.h (1 hit)
	Line 29: #define BRUSHED_ESC_AUTODETECT
	#define TARGET_BOARD_IDENTIFIER "AFF3"
	#define BRUSHED_ESC_AUTODETECT
  C:\STM32Toolchain\quadri\quadri_dev\alphaflight\src\main\target\ALIENFLIGHTF4\target.h (1 hit)
	Line 24: #define BRUSHED_ESC_AUTODETECT
	#define TARGET_BOARD_IDENTIFIER "AFF4"
	#define BRUSHED_ESC_AUTODETECT
  C:\STM32Toolchain\quadri\quadri_dev\alphaflight\src\main\target\SPRACINGF3EVO\target.h (1 hit)
	Line 28: #define BRUSHED_ESC_AUTODETECT
	#ifdef AIORACERF3
#define TARGET_BOARD_IDENTIFIER "ARF3"
#else
#define TARGET_BOARD_IDENTIFIER "SPEV"
#endif
	#define BRUSHED_ESC_AUTODETECT
  C:\STM32Toolchain\quadri\quadri_dev\alphaflight\src\main\target\SPRACINGF3MINI\target.h (1 hit)
	Line 176: #define BRUSHED_ESC_AUTODETECT
	TINYBEEF3 -> #define TARGET_BOARD_IDENTIFIER "TBF3"
	#ifdef TINYBEEF3
#define BRUSHED_ESC_AUTODETECT
#else
	#define TARGET_BOARD_IDENTIFIER "SRFM"

*/
//####################################################
#if defined(RMDO)

//####################################################
#elif defined(ZCOREF3)

//####################################################
#elif defined(FLIP32F3OSD)

//####################################################
#elif defined(SPRACINGF3)

//####################################################
#elif defined(ADAPTATOR_F3)

//####################################################
#elif defined(AFROMINI)

//####################################################
#elif defined(BEEBRAIN)

//####################################################
#elif defined(ADAPTATOR_F1) 

//####################################################
#elif  defined(NAZE)

//####################################################
#else
#error "Board not defined"
#endif
