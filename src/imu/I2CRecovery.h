#ifndef NIUS_IMU_I2C_RECOVERY_H
#define NIUS_IMU_I2C_RECOVERY_H

#include <Wire.h>

#if defined(ARDUINO_ARCH_ESP32) && defined(__has_include)
#if __has_include(<driver/i2c_master.h>)
#include <driver/i2c_master.h>
#define NIMU_HAS_ESP32_I2C_BUS_RESET 1
#else
#define NIMU_HAS_ESP32_I2C_BUS_RESET 0
#endif
#else
#define NIMU_HAS_ESP32_I2C_BUS_RESET 0
#endif

namespace nimu {
namespace detail {

// Recover only the MCU controller state after a completed transfer reports an
// error. This does not pulse the bus pins, restart Wire, or address a device.
inline void resetI2CControllerAfterError(TwoWire& wire) {
#if NIMU_HAS_ESP32_I2C_BUS_RESET
  i2c_master_bus_handle_t handle = nullptr;
  if (i2c_master_get_bus_handle(
          static_cast<i2c_port_num_t>(wire.getBusNum()), &handle) == ESP_OK &&
      handle != nullptr) {
    (void)i2c_master_bus_reset(handle);
  }
#else
  (void)wire;
#endif
}

}  // namespace detail
}  // namespace nimu

#undef NIMU_HAS_ESP32_I2C_BUS_RESET

#endif  // NIUS_IMU_I2C_RECOVERY_H
