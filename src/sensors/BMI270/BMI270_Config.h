#ifndef ARDUINONRF_IMU_BMI270_CONFIG_H
#define ARDUINONRF_IMU_BMI270_CONFIG_H

#include <Arduino.h>

#include <stddef.h>
#include <stdint.h>

namespace nimu {
namespace bmi270 {

// Bosch Sensortec's standard BMI270 configuration image. It is distributed
// with the BMI270 SensorAPI under BSD-3-Clause; the complete notice is kept in
// LICENSE.Bosch.txt beside this file. Keeping the image in one .cpp avoids a
// copy in every translation unit. PROGMEM prevents it consuming scarce SRAM
// on AVR, while the driver performs architecture-correct byte reads.
extern const uint8_t kDefaultConfigImage[8192] PROGMEM;
static const size_t kDefaultConfigImageLength = 8192;

}  // namespace bmi270
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BMI270_CONFIG_H
