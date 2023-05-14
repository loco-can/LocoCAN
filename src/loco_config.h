/*
 * LOCO-CAN global setup file
 * 
 * @author: Thomas H Winkler
 * @copyright: 2020
 * @lizence: GG0
 */

// =====================================
// LIBRARIES
//
// https://github.com/ricaun/ArduinoUniqueID
// https://github.com/mrbio/Arduino-rokkit-hash
// https://github.com/Chris--A/EEWrap
// =====================================


#ifndef LOCO_CONFIG_H
#define LOCO_CONFIG_H


// =====================================
//
// GLOBAL DEFINITIONS
//
// =====================================


// =====================================
// Board versions:
// 
#define BOARD_V_1_0 0x01 // V1.0
#define BOARD_V_1_2 0x02 // V1.2
#define BOARD_V_2_0 0x03 // V2.0
#define BOARD_WIFI_V_2_1 0x04 // WIFI V2.1
#define BOARD_UNIV_V_2_1 0x05 // UNIVERSIAL V2.1
#define BOARD_MICRO_V_2_2 0x70 // ATtiny85 MICRO BOARD V2.2


#define BOARD_DIGISPARK 0xFF// Digispark developement
// =====================================


// uncommend to get Serial debug information 
#define DEBUG

// convert bytes to a 16/32 bit integer
#define char2int(MSB, LSB) (uint16_t)(((uint8_t) MSB & 0xFF) << 8 | ((int8_t) LSB & 0xFF))
#define char2long(MSB, LSB, MSB1, LSB1) (((uint32_t) ((uint8_t) MSB)) & 0xFF) << 24 | (((uint32_t) ((uint8_t) LSB)) & 0xFF) << 16 | (((uint32_t) ((uint8_t) MSB1)) & 0xFF) << 8 | (((int8_t) LSB1) & 0xFF)


// =====================================
// Global settings
#define RED INTELLILED_RED
#define GREEN INTELLILED_GREEN
#define YELLOW INTELLILED_YELLOW

#define MODULE_NAME_LENGTH 15

#define HEARTBEAT 500

#define BLINKTIME 500

// =====================================
// setup data in EEPROM
//
// The module setup data is stored in the EEPROM usind
// the EEWrap library. The data struct must contain:
//   uint16_a uuid
//
// uuid must be set to the mcu-uuid, to indicate, that
// valid data was stored.


// =====================================
// CAN identifier definition 0x000 t0 0x7FF (11bit)
// The LSB of ID is the device number (0-F)

// 0xNMO    N=0     reset
//          N=1-6   data type
//          N=7     setup data
//          M       subtype
//          o       module ID       defined when module is registered


// =====================================
//
// MODULE DECLARATION
//
// =====================================

// =====================================
// Each module type is identified by a type group and a sub identifier.
// The highest bit of the 8 bit code indicates wireless ability.
// Therefore 8 groups with 8 subgroups are possible for wired and wireless modules.

// MODULES TYPE GROUP
// 0x0n CONTROLLER
#define MODULE_CONTROLLER 0x10 // wired controller module
#define MODULE_CONTROLLER_WIFI 0x90 // WiFi controller module

// 0x1n SENSOR
#define MODULE_SENSOR 0x20 // UI Sensor module

// 0x2n SWITCH
#define MODULE_SWITCH 0x30

// 0x3n ACTUATOR
#define MODULE_SERVO 0x40

// 0x4n

// 0x5n

// 0x6n

// 0x7n


// ==================================
//
// ATMEL ATMEGA8 & 168/328 / ARDUINO
//
// ==================================

//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+


// ==================================
//
// Microchip MCP2515 wiring
//
// ==================================

// MCP2515        Atmega328
//   VCC             5V
//   GND             GND
//   SCK             SCK
//   SO              MISO
//   SI              MOSI
//   CS              10
//   INT             2  (optional)
//
// Espressif ESP32 wiring
//
// CAN-trans        ESP32
//   3V3             3V3
//   GND             GND
//   CTX              5
//   CRX              4


// ==================================
//
// Print V1.0 extension bus
//
// ==================================

// 10x1 PIN (2,54mm Raster)

// 1   VCC
// 2   D9
// 3   D8
// 4   D7
// 5   D6
// 6   A0
// 7   A1
// 8   A2
// 9   A3
// 10  GND

//  ICP-Connector
// 4x1 PIN (2,54mm Raster)
// Mitte: 1200/600

// 1   VCC
// 2   TX
// 3   RX
// 4   DTR (patch) original GND
// 4a  GND (patch)

// CAN-Connector JST-XH 2,5mm Raster
// 1   VCC
// 2   CAN-H
// 3   CAN-L
// 4   GND

// ==================================
//
// Print V1.2 extension bus
//
// ==================================

// 8x2 pins
//
//  1   VCC         2   D2
//  3   A5          4   D3 (PWM)
//  5   A4          6   D4
//  7   A3          8   D5 (PWM)
//  8   A2          10  D6 (PWM)
//  11  A1          12  D7
//  13  A0          14  D8
//  15  D9 (PWM)    16  GND

// big extension bus version
// 9x2 pins
//  ...
//  17  +12V        18  GND

// ICP-Connector
// 6x1 PIN (2,54mm Raster)
// Mitte: 625/125

// 1   DTR
// 2   TX
// 3   RX
// 4   VCC
// 5   CTS
// 6   GND

// CAN-Connector JST-XH 2,5mm Raster
// 1   VCC
// 2   CAN-H
// 3   CAN-L
// 4   GND

// ==================================
//
// Print V2.x
//
// ==================================

// The version 2 is an integraded board wirth MCU, CAN-bus
// and function hardware in one without an extension bus.


#endif