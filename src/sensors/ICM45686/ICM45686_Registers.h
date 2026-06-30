#ifndef ARDUINONRF_IMU_ICM45686_REGISTERS_H
#define ARDUINONRF_IMU_ICM45686_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace icm45686 {

static constexpr uint8_t kAddr = 0x68;
static constexpr uint8_t kWhoAmI = 0xE9;

static constexpr uint8_t ACCEL_DATA_X1 = 0x00;
static constexpr uint8_t GYRO_DATA_X1 = 0x06;
static constexpr uint8_t TEMP_DATA1 = 0x0C;
static constexpr uint8_t PWR_MGMT0 = 0x10;
static constexpr uint8_t INT1_CONFIG0 = 0x16;
static constexpr uint8_t INT1_CONFIG1 = 0x17;
static constexpr uint8_t INT1_CONFIG2 = 0x18;
static constexpr uint8_t INT1_STATUS0 = 0x19;
static constexpr uint8_t ACCEL_CONFIG0 = 0x1B;
static constexpr uint8_t GYRO_CONFIG0 = 0x1C;
static constexpr uint8_t IOC_PAD_SCENARIO_AUX_OVRD = 0x30;
static constexpr uint8_t IOC_PAD_SCENARIO = 0x2F;
static constexpr uint8_t INT2_CONFIG0 = 0x56;
static constexpr uint8_t INT2_CONFIG1 = 0x57;
static constexpr uint8_t INT2_CONFIG2 = 0x58;
static constexpr uint8_t INT2_STATUS0 = 0x59;
static constexpr uint8_t WHO_AM_I = 0x72;
static constexpr uint8_t IREG_ADDR = 0x7C;
static constexpr uint8_t IREG_DATA = 0x7E;
static constexpr uint8_t REG_MISC2 = 0x7F;

static constexpr uint8_t PWR_ACCEL_GYRO_LN = 0x0F;
static constexpr uint8_t SOFT_RESET = 0x03;
static constexpr uint8_t IREG_DONE = 0x01;
static constexpr uint8_t INT_DRDY = 0x04;
static constexpr uint8_t INT_AUX_DRDY = 0x08;
static constexpr uint8_t INT_OPEN_DRAIN = 0x04;
static constexpr uint8_t INT_LATCHED = 0x02;
static constexpr uint8_t INT_ACTIVE_LOW = 0x01;
static constexpr uint8_t AUX_OVERRIDE_MODE = 0x10;
static constexpr uint8_t AUX_OVERRIDE_ENABLE = 0x02;
static constexpr uint8_t AUX_ENABLE = 0x01;
static constexpr uint8_t AUX_MODE_I2C_MASTER = 0x04;
static constexpr uint8_t AUX_MODE_I2C_BYPASS = 0x08;

static constexpr uint16_t IPREG_TOP1 = 0xA200;
static constexpr uint8_t I2CM_COMMAND0 = 0x06;
static constexpr uint8_t I2CM_DEV_PROFILE0 = 0x0E;
static constexpr uint8_t I2CM_DEV_PROFILE1 = 0x0F;
static constexpr uint8_t I2CM_CONTROL = 0x16;
static constexpr uint8_t I2CM_STATUS = 0x18;
static constexpr uint8_t I2CM_READ_DATA0 = 0x1B;
static constexpr uint8_t I2CM_WRITE_DATA0 = 0x33;
static constexpr uint8_t I2CM_STATUS_DONE = 0x02;
static constexpr uint8_t I2CM_STATUS_BUSY = 0x01;

}  // namespace icm45686
}  // namespace nimu

#endif
