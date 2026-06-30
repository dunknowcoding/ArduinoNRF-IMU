/*
  ArduinoNRF_IMU.h - Umbrella header for the ArduinoNRF-IMU library.

  A unified, friendly IMU library for the ArduinoNRF board package. Each chip
  lives under src/sensors/ and each multi-chip breakout under src/boards/; the
  inertial sensors share one interface (nimu::IMUSensor) so your code reads the
  same regardless of which one is fitted.

  You normally include the specific board or sensor you have, e.g.:

      #include <GY91.h>     // GY-91 board: MPU-9250/6500 + BMP280
      #include <GY87.h>     // GY-87 board: MPU-6050 + BMP180 + compass
      #include <GY86.h>     // GY-86 board: MPU-6050 + MS5611 + compass
      #include <GY88.h>     // GY-88 board: MPU-6050 + BMP085/BMP180 + compass
      #include <GY63.h>     // GY-63 board: MS5611 barometer
      #include <GY85.h>     // GY-85 board: ADXL345 + ITG3205 + compass
      #include <GY80.h>     // GY-80/GY-801 board: ADXL345 + L3G4200D + BMP + compass
      #include <GY801.h>    // GY-801 board alias for GY-80-style modules
      #include <GY291.h>    // GY-291 board: ADXL345 accelerometer
      #include <GY45.h>     // GY-45 board: MMA8452Q accelerometer
      #include <GY50.h>     // GY-50 board: L3G4200D gyroscope
      #include <GY68.h>     // GY-68 board: BMP180/BMP085 barometer
      #include <GY271.h>    // GY-271 board: HMC5883L/QMC5883L compass
      #include <GY273.h>    // GY-273 board: HMC5883L/QMC5883L compass
      #include <GY511.h>    // GY-511 board: LSM303DLHC accel + compass
      #include <GY9250.h>   // GY-9250 board: MPU-9250 (9-axis)
      #include <GY521.h>    // GY-521 board: MPU-6050 (6-axis)
      #include <GYLSM6DS3.h>// GY-LSM6DS3 board: LSM6DS3 (6-axis)
      #include <GYBNO055.h> // GY-BNO055 board: BNO055 (9-axis fusion)
      #include <MPU9250.h>  // bare MPU-9250 chip (9-axis)
      #include <MPU9255.h>  // bare MPU-9255 chip (9-axis)
      #include <MPU6500.h>  // bare MPU-6500 chip (6-axis, no magnetometer)
      #include <MPU6050.h>  // bare MPU-6050 chip (6-axis, no magnetometer)
      #include <MPU6886.h>  // bare MPU-6886 chip (6-axis, no magnetometer)
      #include <ICM20602.h> // bare ICM-20602 chip (6-axis, no magnetometer)
      #include <ICM20689.h> // bare ICM-20689 chip (6-axis, no magnetometer)
      #include <ADXL345.h>  // bare ADXL345 accelerometer
      #include <MMA8452Q.h> // bare MMA8452Q accelerometer
      #include <ITG3205.h>  // bare ITG-3200/ITG-3205 gyroscope
      #include <L3G4200D.h> // bare L3G4200D gyroscope
      #include <L3GD20.h>   // L3GD20/L3GD20H gyroscope
      #include <LSM303_L3GD20.h> // common 9DOF clone composition
      #include <LSM303DLHC.h>// bare LSM303DLHC chip (accel + magnetometer)
      #include <LSM303D.h>  // single-chip accel + magnetometer clone variant
      #include <LSM6DS3.h>  // bare LSM6DS3 chip (6-axis, no magnetometer)
      #include <LSM6DSV.h>  // LSM6DSV with SFLP fusion and sensor hub
      #include <LSM6DSV320X.h> // dual low-g/high-g IMU with SFLP and AI core
      #include <LSM6DS3TRC.h> // LSM6DS3TR-C with embedded event functions
      #include <LSM6DSOX.h> // bare LSM6DSOX chip (6-axis, no magnetometer)
      #include <LSM9DS1.h>  // bare LSM9DS1 chip (9-axis)
      #include <ICM20948.h> // bare ICM-20948 chip (basic accel/gyro path)
      #include <ICM45686.h> // bare ICM-45686 chip (6-axis + auxiliary I2C)
      #include <BMI160.h>   // bare BMI160 chip (6-axis, no magnetometer)
      #include <BMI088.h>   // BMI088 accel + gyro pair (two I2C addresses)
      #include <BMI270.h>   // bare BMI270 chip (6-axis, no magnetometer)
      #include <BMI323.h>   // bare BMI323 chip (word-addressed I2C)
      #include <BMX055.h>   // BMA280 + BMG160 + BMM150 (9-axis)
      #include <QMI8658.h>  // bare QMI8658 chip (6-axis, no magnetometer)
      #include <ICM42688P.h>// bare ICM-42688-P chip (6-axis, no magnetometer)
      #include <BNO055.h>   // bare BNO055 chip (9-axis fusion)
      #include <BNO08x.h>   // BNO085/BNO086 SH-2 sensor hub (9-axis fusion)
      #include <MUMO.h>     // ICM-45686 + QMC6309 board
      #include <CJMCU055.h>  // purple CJMCU-055 BMX055 board
      #include <GY601N1.h>  // GY-601N1 ICM-42688-P/ICM-45686/BMI323 board
      #include <QMC6309.h>  // QMC6309 magnetometer
      #include <GYBNO085.h> // exact ten-pad purple GY-BNO085 board
      #include <BMP280.h>   // bare BMP280 barometer (pressure + temperature)
      #include <MS5611.h>   // bare MS5611 barometer (pressure + temperature)
      #include <BMP180.h>   // bare BMP180 barometer (pressure + temperature)

  Include THIS header only if you want the framework pieces (types, the bus
  helper, the base class) without a particular sensor - for instance when
  writing your own driver. See docs/ADDING_A_SENSOR.md.
*/
#ifndef ARDUINONRF_IMU_H
#define ARDUINONRF_IMU_H

#define ARDUINONRF_IMU_VERSION_MAJOR 0
#define ARDUINONRF_IMU_VERSION_MINOR 3
#define ARDUINONRF_IMU_VERSION_PATCH 0
#define ARDUINONRF_IMU_VERSION "0.3.0"

#include "imu/IMUBus.h"
#include "imu/IMUSensor.h"
#include "imu/IMUTypes.h"

#endif  // ARDUINONRF_IMU_H
