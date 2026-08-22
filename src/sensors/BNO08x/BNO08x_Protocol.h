/* Minimal SHTP/SH-2 constants used by the BNO08x I2C driver. */
#ifndef ARDUINONRF_IMU_BNO08X_PROTOCOL_H
#define ARDUINONRF_IMU_BNO08X_PROTOCOL_H

#include <Arduino.h>

namespace nimu {
namespace bno08x {

static constexpr uint8_t kAddrDefault = 0x4B;
static constexpr uint8_t kAddrAlternate = 0x4A;

static constexpr uint8_t CHANNEL_COMMAND = 0;
static constexpr uint8_t CHANNEL_EXECUTABLE = 1;
static constexpr uint8_t CHANNEL_CONTROL = 2;
static constexpr uint8_t CHANNEL_REPORTS = 3;
static constexpr uint8_t CHANNEL_WAKE_REPORTS = 4;
static constexpr uint8_t CHANNEL_GYRO = 5;

static constexpr uint8_t REPORT_COMMAND_REQUEST = 0xF2;
static constexpr uint8_t REPORT_COMMAND_RESPONSE = 0xF1;
static constexpr uint8_t REPORT_PRODUCT_ID_RESPONSE = 0xF8;
static constexpr uint8_t REPORT_PRODUCT_ID_REQUEST = 0xF9;
static constexpr uint8_t REPORT_BASE_TIMESTAMP = 0xFB;
static constexpr uint8_t REPORT_SET_FEATURE = 0xFD;

static constexpr uint8_t SENSOR_ACCELEROMETER = 0x01;
static constexpr uint8_t SENSOR_GYROSCOPE = 0x02;
static constexpr uint8_t SENSOR_MAGNETIC_FIELD = 0x03;
static constexpr uint8_t SENSOR_LINEAR_ACCELERATION = 0x04;
static constexpr uint8_t SENSOR_ROTATION_VECTOR = 0x05;
static constexpr uint8_t SENSOR_GRAVITY = 0x06;
static constexpr uint8_t SENSOR_GAME_ROTATION_VECTOR = 0x08;
static constexpr uint8_t SENSOR_GEOMAGNETIC_ROTATION_VECTOR = 0x09;
static constexpr uint8_t SENSOR_TAP_DETECTOR = 0x10;
static constexpr uint8_t SENSOR_STEP_COUNTER = 0x11;
static constexpr uint8_t SENSOR_STABILITY_CLASSIFIER = 0x13;
static constexpr uint8_t SENSOR_PERSONAL_ACTIVITY_CLASSIFIER = 0x1E;

// Verified as delivered by a BNO085 running SH-2 3.2.13.
static constexpr uint8_t SENSOR_GYROSCOPE_UNCALIBRATED = 0x07;
static constexpr uint8_t SENSOR_MAGNETIC_FIELD_UNCALIBRATED = 0x0F;
static constexpr uint8_t SENSOR_STEP_DETECTOR = 0x18;
static constexpr uint8_t SENSOR_SHAKE_DETECTOR = 0x19;
static constexpr uint8_t SENSOR_STABILITY_DETECTOR = 0x1C;
static constexpr uint8_t SENSOR_ARVR_ROTATION_VECTOR = 0x28;
static constexpr uint8_t SENSOR_ARVR_GAME_ROTATION_VECTOR = 0x29;

static constexpr uint8_t COMMAND_TARE = 0x03;
static constexpr uint8_t COMMAND_SAVE_DCD = 0x06;
static constexpr uint8_t COMMAND_ME_CALIBRATION = 0x07;
static constexpr uint8_t COMMAND_DCD_PERIODIC_SAVE = 0x09;

}  // namespace bno08x
}  // namespace nimu

#endif
