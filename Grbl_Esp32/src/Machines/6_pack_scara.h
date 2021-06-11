#pragma once
// clang-format off

/*
    6_pack_trinamic_V1.h
    Part of Grbl_ESP32
    Pin assignments for the ESP32 SPI 6-axis board
    2018    - Bart Dring
    2020    - Mitch Bradley
    2020    - Michiyasu Odaki
    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.
*/
#define MACHINE_NAME            "6 Pack Controller - Custom Config"
#define CUSTOM_CODE_FILENAME "Custom/scara.cpp"

#define N_AXIS 4

// I2S (steppers & other output-only pins)
#define USE_I2S_OUT
#define USE_I2S_STEPS
//#define DEFAULT_STEPPER ST_I2S_STATIC

#define I2S_OUT_BCK      GPIO_NUM_22
#define I2S_OUT_WS       GPIO_NUM_17
#define I2S_OUT_DATA     GPIO_NUM_21

#define TRINAMIC_RUN_MODE           Motors::TrinamicMode::StealthChop
#define TRINAMIC_HOMING_MODE        Motors::TrinamicMode::StealthChop

// Motor Socket #1
// #define C_TRINAMIC_DRIVER       2130
// #define C_DISABLE_PIN           I2SO(0)
// #define C_DIRECTION_PIN         I2SO(1)
// #define C_STEP_PIN              I2SO(2)
// #define C_CS_PIN                I2SO(3)
// #define C_RSENSE                TMC2130_RSENSE_DEFAULT

// Motor Socket #2
// #define B_TRINAMIC_DRIVER       2130
// #define B_DIRECTION_PIN         I2SO(4)
// #define B_STEP_PIN              I2SO(5)
// #define B_DISABLE_PIN           I2SO(7)
// #define B_CS_PIN                I2SO(6)
// #define B_RSENSE                TMC2130_RSENSE_DEFAULT

// Motor Socket #3
#define A_TRINAMIC_DRIVER       2130
#define A_DISABLE_PIN           I2SO(8)
#define A_DIRECTION_PIN         I2SO(9)
#define A_STEP_PIN              I2SO(10)
#define A_CS_PIN                I2SO(11)
#define A_RSENSE                TMC2130_RSENSE_DEFAULT

// Motor Socket #4
#define Y_TRINAMIC_DRIVER       2130
#define Y_DIRECTION_PIN         I2SO(12)
#define Y_STEP_PIN              I2SO(13)
#define Y_DISABLE_PIN           I2SO(15)
#define Y_CS_PIN                I2SO(14)
#define Y_RSENSE                TMC2130_RSENSE_DEFAULT

// Motor Socket #5
#define X_TRINAMIC_DRIVER       2130
#define X_DISABLE_PIN           I2SO(16)
#define X_DIRECTION_PIN         I2SO(17)
#define X_STEP_PIN              I2SO(18)
#define X_CS_PIN                I2SO(19)
#define X_RSENSE                TMC2130_RSENSE_DEFAULT
#define X_LENGTH                150

// Motor Socket #6
#define Z_TRINAMIC_DRIVER       2130
#define Z_DIRECTION_PIN         I2SO(20)
#define Z_STEP_PIN              I2SO(21)
#define Z_DISABLE_PIN           I2SO(23)
#define Z_CS_PIN                I2SO(22)
#define Z_RSENSE                TMC2130_RSENSE_DEFAULT
#define Y_LENGTH                90

/*
    Socket I/O reference
    The list of modules is here...
    https://github.com/bdring/6-Pack_CNC_Controller/wiki/CNC-I-O-Module-List
    Click on each module to get example for using the modules in the sockets

Socket #1
#1 GPIO_NUM_33 (Sg1)
#2 GPIO_NUM_32 (Sg2)
#3 GPIO_NUM_35 (Sg3) (input only)
#4 GPIO_NUM_34 (Sg4) (input only)

Socket #2
#1 GPIO_NUM_2
#2 GPIO_NUM_25
#3 GPIO_NUM_39 (Sg5) (input only)
#4 GPIO_NUM_36 (Sg6) (input only)

Socket #3
#1 GPIO_NUM_26
#2 GPIO_NUM_4
#3 GPIO_NUM_16
#4 GPIO_NUM_27

Socket #4
#1 GPIO_NUM_14
#2 GPIO_NUM_13
#3 GPIO_NUM_15
#4 GPIO_NUM_12

Socket #5
#1 I2SO(24)  (output only)
#2 I2SO(25)  (output only)
#3 I2SO26)  (output only)

*/

// Socket #1
#define X_LIMIT_PIN             GPIO_NUM_33
#define Y_LIMIT_PIN             GPIO_NUM_32
#define Y2_LIMIT_PIN            GPIO_NUM_35
#define Z_LIMIT_PIN             GPIO_NUM_34

// Socket #2
#define SPINDLE_TYPE            SpindleType::RELAY
#define SPINDLE_OUTPUT_PIN      GPIO_NUM_2 // relay as spindle on module socket #2

// Socket #3
// #define PROBE_PIN               GPIO_NUM_26

// Default Settings

// #define DEFAULT_HOMING_CYCLE_0 0b00000100
// #define DEFAULT_HOMING_CYCLE_1 0b00000001
// #define DEFAULT_HOMING_CYCLE_2 0b00000010

#define DEFAULT_DIRECTION_INVERT_MASK 0b00000111

#define DEFAULT_INVERT_LIMIT_PINS false
#define DEFAULT_HARD_LIMIT_ENABLE false
#define DEFAULT_SOFT_LIMIT_ENABLE false
#define DEFAULT_HOMING_ENABLE false

#define DEFAULT_HOMING_DIR_MASK 0b00000001
#define DEFAULT_HOMING_SQUARED_AXES 0b00000010
#define DEFAULT_HOMING_FEED_RATE 150.000
#define DEFAULT_HOMING_SEEK_RATE 2000.000
#define DEFAULT_HOMING_DEBOUNCE_DELAY 100.000
#define DEFAULT_HOMING_PULLOFF 1.500

#define DEFAULT_X_STEPS_PER_MM 331.52
#define	DEFAULT_Y_STEPS_PER_MM 331.52
#define	DEFAULT_Z_STEPS_PER_MM 800
#define	DEFAULT_A_STEPS_PER_MM 32

#define DEFAULT_X_MAX_RATE 4000
#define DEFAULT_Y_MAX_RATE 4000
#define DEFAULT_Z_MAX_RATE 4000
#define DEFAULT_A_MAX_RATE 10000

#define DEFAULT_X_ACCELERATION 200
#define DEFAULT_Y_ACCELERATION 200
#define DEFAULT_Z_ACCELERATION 200
#define DEFAULT_A_ACCELERATION 1000

#define DEFAULT_X_HOMING_MPOS 0
#define DEFAULT_Y_HOMING_MPOS 0
#define DEFAULT_Z_HOMING_MPOS 0
#define DEFAULT_A_HOMING_MPOS 0

#define DEFAULT_X_MAX_TRAVEL 100
#define DEFAULT_Y_MAX_TRAVEL 100
#define DEFAULT_Z_MAX_TRAVEL 100
#define DEFAULT_A_MAX_TRAVEL 100

#define DEFAULT_X_CURRENT 1
#define DEFAULT_Y_CURRENT 1
#define DEFAULT_Z_CURRENT 1
#define DEFAULT_A_CURRENT 0.1

#define DEFAULT_X_HOLD_CURRENT 0.25
#define DEFAULT_Y_HOLD_CURRENT 0.25
#define DEFAULT_Z_HOLD_CURRENT 0.25
#define DEFAULT_A_HOLD_CURRENT 0.1

#define DEFAULT_STEPPER_IDLE_LOCK_TIME 255
