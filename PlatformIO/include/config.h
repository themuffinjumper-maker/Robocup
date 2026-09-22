#pragma once

#include <Arduino.h>

// ============================================================
// PIN / HARDWARE ASSIGNMENTS
// ============================================================

namespace Pins {

    // --------------------------------------------------------
    // Drivetrain
    // Tracked chassis, PPM-controlled DC motor driver (DFR0513)
    // --------------------------------------------------------

    // Historical placeholders only: D2-D5 are now reserved for the encoder
    // board on Digital Raw 2 and must not also be enabled as motor outputs.
    constexpr uint8_t DRIVE_LEFT  = 2;
    constexpr uint8_t DRIVE_RIGHT = 3;


    // --------------------------------------------------------
    // Reel intake
    // 2x SKU365231 motors, PPM-controlled DC motor driver (DFR0513)
    // --------------------------------------------------------

    constexpr uint8_t REEL_LEFT  = 4;
    constexpr uint8_t REEL_RIGHT = 5;

    // Digital Raw 2 signal order on the CPU board is D5, D4, D3, D2.
    // The supplied 112_Encoder example pairs these as D2/D3 and D4/D5.
    constexpr uint8_t ENCODER_1_A = 2;
    constexpr uint8_t ENCODER_1_B = 3;
    constexpr uint8_t ENCODER_2_A = 4;
    constexpr uint8_t ENCODER_2_B = 5;


    // --------------------------------------------------------
    // Herkulex smart servo bus
    //
    // Teensy 4.0 Serial1:
    // RX1 = pin 0
    // TX1 = pin 1
    //
    // Sorting gate + kicker cam share this serial bus and are
    // addressed by servo ID rather than individual signal pins.
    // --------------------------------------------------------

    constexpr uint8_t SMART_SERVO_RX = 0;
    constexpr uint8_t SMART_SERVO_TX = 1;

    // CH9143 matched Bluetooth board connected to SERIAL1.
    constexpr uint8_t BLUETOOTH_RX = 0;
    constexpr uint8_t BLUETOOTH_TX = 1;

    // Two 203_DCMotor controllers. Each channel takes a servo-style pulse,
    // not UART data; the four outputs are independent digital pins.
    constexpr uint8_t DC_MOTOR_203_CHANNEL_A = 27;
    constexpr uint8_t DC_MOTOR_203_CHANNEL_B = 26;
    constexpr uint8_t DC_MOTOR_203_SECOND_CHANNEL_A = 25;
    constexpr uint8_t DC_MOTOR_203_SECOND_CHANNEL_B = 15;

    // Digital Raw 1 -> Digital Level Shift servo outputs. D30-to-D was
    // confirmed physically; the remaining channels follow the reverse order.
    constexpr uint8_t HX12K_OUTPUT_A = 33;
    constexpr uint8_t HX12K_OUTPUT_B = 32;
    constexpr uint8_t HX12K_OUTPUT_C = 31;
    constexpr uint8_t HX12K_OUTPUT_D = 30;


    // --------------------------------------------------------
    // Sensors
    // --------------------------------------------------------

    // TOF and colour sensors share I2C address 0x29 by default;
    // split across Teensy's two I2C buses until a mux/address
    // translator choice is finalised.

    constexpr uint8_t TOF_I2C_BUS    = 0; // Wire
    constexpr uint8_t COLOUR_I2C_BUS = 1; // Wire1

    constexpr uint8_t IR_LEFT  = A0; // Also D14; unavailable while ultrasound echo uses D14.
    constexpr uint8_t IR_RIGHT = A1; // Also D15; unavailable while second motor driver uses D15.

    constexpr uint8_t ULTRASOUND_FRONT_TRIG = 14;
    constexpr uint8_t ULTRASOUND_FRONT_ECHO = 24;
    constexpr uint8_t ULTRASOUND_B_TRIG = 22;
    constexpr uint8_t ULTRASOUND_B_ECHO = 20;

    constexpr uint8_t INDUCTIVE_PROXIMITY = 21;

} // namespace Pins


// ============================================================
// HERKULEX CONFIG
// ============================================================

namespace HerkulexConfig {

    // Teensy hardware UART used for all Herkulex servos.
    // Bluetooth occupies Serial1, so plug the Herkulex bus into SERIAL2.
    inline HardwareSerial& PORT = Serial2;

    constexpr uint32_t BAUD = 115200;


    // --------------------------------------------------------
    // Servo IDs
    // --------------------------------------------------------

    namespace Id {

        constexpr uint8_t SORTING_GATE = 1;
        constexpr uint8_t KICKER_CAM   = 2;

    } // namespace Id


    // --------------------------------------------------------
    // Mechanical / positional limits
    //
    // Current Herkulex library uses positional mode and clamps
    // commands to approximately +/-160 degrees.
    // --------------------------------------------------------

    constexpr float MIN_ANGLE_DEG = -160.0f;
    constexpr float MAX_ANGLE_DEG =  160.0f;


    // --------------------------------------------------------
    // Default movement times
    // --------------------------------------------------------

    constexpr uint16_t DEFAULT_MOVE_TIME_MS = 500;

    constexpr uint16_t SORTING_GATE_MOVE_TIME_MS = 300;
    constexpr uint16_t KICKER_CAM_MOVE_TIME_MS   = 250;


    // --------------------------------------------------------
    // Library / servo startup timing
    // --------------------------------------------------------

    constexpr uint16_t REBOOT_DELAY_MS      = 700;
    constexpr uint16_t CLEAR_ERROR_DELAY_MS = 100;
    constexpr uint16_t TORQUE_ON_DELAY_MS   = 100;


    // --------------------------------------------------------
    // Sorting gate positions
    //
    // Placeholder values.
    // Tune these once the actual mechanism is assembled.
    // --------------------------------------------------------

    namespace SortingGate {

        constexpr float LEFT_DEG   = -45.0f;
        constexpr float CENTRE_DEG =   0.0f;
        constexpr float RIGHT_DEG  =  45.0f;

    } // namespace SortingGate


    // --------------------------------------------------------
    // Kicker cam positions
    //
    // Placeholder values.
    // Tune these once the actual mechanism is assembled.
    // --------------------------------------------------------

    namespace KickerCam {

        constexpr float HOME_DEG = 0.0f;
        constexpr float KICK_DEG = 90.0f;

    } // namespace KickerCam

} // namespace HerkulexConfig

namespace BluetoothConfig {

    inline HardwareSerialIMXRT& PORT = Serial1;
    constexpr uint32_t BAUD = 115200;
    constexpr uint32_t TELEMETRY_INTERVAL_MS = 500;

} // namespace BluetoothConfig


// ============================================================
// TOF SENSOR CONFIG
// ============================================================
//
// TOF sensor XSHUT pins are controlled using the onboard
// SX1509 IO expander rather than raw Teensy GPIO.
//
// Current fitted layout: one VL53L1X long-range sensor on XSHUT1.
// ============================================================

namespace TofConfig {

    constexpr uint8_t EXPANDER_I2C_ADDRESS = 0x3F;
    constexpr uint8_t MAX_SENSOR_COUNT = 8;
    // 0x33 is reserved by the SEN0628 8x8 sensor.
    constexpr uint8_t FIRST_ASSIGNED_ADDRESS = 0x40;

} // namespace TofConfig

namespace Tof8x8Config {

    constexpr uint8_t I2C_ADDRESS = 0x33;
    constexpr uint32_t FRAME_INTERVAL_MS = 500;

} // namespace Tof8x8Config


// ============================================================
// IR SENSOR CONFIG
// ============================================================
//
// The team's bench testing found analogue IR distance readings
// too noisy to trust for accurate ranging, particularly when
// objects are viewed off-angle.
//
// Prefer using these as object-presence sensors where possible.
// ============================================================

namespace IrConfig {

    constexpr uint8_t COUNT = 2;

    constexpr uint8_t PINS[COUNT] = {
        Pins::IR_LEFT,
        Pins::IR_RIGHT
    };

    // Raw ADC threshold above which an object is considered
    // present.
    //
    // Tune this using the actual mounted sensors.
    constexpr uint16_t PRESENCE_THRESHOLD = 400;

} // namespace IrConfig
