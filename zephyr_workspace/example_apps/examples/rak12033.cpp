/**
 * @file iim42652_zephyr.cpp
 * @brief TDK IIM-42652 6-axis IMU driver for Zephyr RTOS
 * @version 1.0
 * @date 2024
 *
 * Converted from Arduino RAK12033 library to Zephyr RTOS
 * Original copyright (c) 2022 RAKwireless
 */

#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define I2C_DEV_NODE DT_NODELABEL(i2c0)
LOG_MODULE_REGISTER(iim42652, LOG_LEVEL_INF);

/* ============================================================================
 * Register Definitions
 * ============================================================================ */

// Device address
#define IIM42652_I2C_ADDR 0x69

// User Bank 0 Registers
#define IIM42652_REG_DEVICE_CONFIG 0x11
#define IIM42652_REG_DRIVE_CONFIG 0x13
#define IIM42652_REG_INT_CONFIG 0x14
#define IIM42652_REG_FIFO_CONFIG 0x16
#define IIM42652_REG_TEMP_DATA1_UI 0x1D
#define IIM42652_REG_TEMP_DATA0_UI 0x1E
#define IIM42652_REG_ACCEL_DATA_X1_UI 0x1F
#define IIM42652_REG_ACCEL_DATA_X0_UI 0x20
#define IIM42652_REG_ACCEL_DATA_Y1_UI 0x21
#define IIM42652_REG_ACCEL_DATA_Y0_UI 0x22
#define IIM42652_REG_ACCEL_DATA_Z1_UI 0x23
#define IIM42652_REG_ACCEL_DATA_Z0_UI 0x24
#define IIM42652_REG_GYRO_DATA_X1_UI 0x25
#define IIM42652_REG_GYRO_DATA_X0_UI 0x26
#define IIM42652_REG_GYRO_DATA_Y1_UI 0x27
#define IIM42652_REG_GYRO_DATA_Y0_UI 0x28
#define IIM42652_REG_GYRO_DATA_Z1_UI 0x29
#define IIM42652_REG_GYRO_DATA_Z0_UI 0x2A
#define IIM42652_REG_INT_STATUS 0x2D
#define IIM42652_REG_INT_STATUS2 0x37
#define IIM42652_REG_INT_STATUS3 0x38
#define IIM42652_REG_SIGNAL_PATH_RESET 0x4B
#define IIM42652_REG_INTF_CONFIG0 0x4C
#define IIM42652_REG_INTF_CONFIG1 0x4D
#define IIM42652_REG_PWR_MGMT0 0x4E
#define IIM42652_REG_GYRO_CONFIG0 0x4F
#define IIM42652_REG_ACCEL_CONFIG0 0x50
#define IIM42652_REG_GYRO_CONFIG1 0x51
#define IIM42652_REG_GYRO_ACCEL_CONFIG0 0x52
#define IIM42652_REG_ACCEL_CONFIG1 0x53
#define IIM42652_REG_APEX_CONFIG0 0x56
#define IIM42652_REG_SMD_CONFIG 0x57
#define IIM42652_REG_INT_CONFIG0 0x63
#define IIM42652_REG_INT_CONFIG1 0x64
#define IIM42652_REG_INT_SOURCE0 0x65
#define IIM42652_REG_INT_SOURCE1 0x66
#define IIM42652_REG_WHO_AM_I 0x75
#define IIM42652_REG_BANK_SEL 0x76

// User Bank 4 Registers
#define IIM42652_REG_APEX_CONFIG1 0x40
#define IIM42652_REG_ACCEL_WOM_X_THR 0x4A
#define IIM42652_REG_ACCEL_WOM_Y_THR 0x4B
#define IIM42652_REG_ACCEL_WOM_Z_THR 0x4C
#define IIM42652_REG_INT_SOURCE6 0x4D

// Chip ID
#define IIM42652_CHIP_ID 0x6F

// Bank Selection
#define IIM42652_SET_BANK_0 0x00
#define IIM42652_SET_BANK_1 0x01
#define IIM42652_SET_BANK_2 0x02
#define IIM42652_SET_BANK_3 0x03
#define IIM42652_SET_BANK_4 0x04

// Power Management
#define IIM42652_SET_TEMPERATURE_DISABLED 0x20
#define IIM42652_SET_GYRO_TLOW_NOISE_MODE 0x0C
#define IIM42652_SET_ACCEL_LOW_NOISE_MODE 0x03

// Bit Masks and Positions
#define BIT_ACCEL_CONFIG0_FS_SEL_POS 5
#define BIT_ACCEL_CONFIG0_FS_SEL_MASK (7 << BIT_ACCEL_CONFIG0_FS_SEL_POS)
#define BIT_ACCEL_CONFIG0_ODR_POS 0
#define BIT_ACCEL_CONFIG0_ODR_MASK 0x0F
#define BIT_GYRO_CONFIG0_FS_SEL_POS 5
#define BIT_GYRO_CONFIG0_FS_SEL_MASK (7 << BIT_GYRO_CONFIG0_FS_SEL_POS)
#define BIT_GYRO_CONFIG0_ODR_POS 0
#define BIT_GYRO_CONFIG0_ODR_MASK 0x0F
#define BIT_PWR_MGMT_0_ACCEL_MODE_MASK 0x03
#define BIT_ACCEL_LP_CLK_SEL_MASK 0x07

// Power Management Constants
#define IIM42652_PWR_MGMT_0_ACCEL_MODE_LP 0x02
#define IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC 0x03

// Wake on Motion
#define X_INT1_EN 0x01
#define Y_INT1_EN 0x02
#define Z_INT1_EN 0x04

/* ============================================================================
 * Enumerations and Structures
 * ============================================================================ */

/**
 * @brief Accelerometer Full Scale Range selection
 */
typedef enum {
    IIM42652_ACCEL_CONFIG0_FS_SEL_16g = (0x0 << BIT_ACCEL_CONFIG0_FS_SEL_POS),
    IIM42652_ACCEL_CONFIG0_FS_SEL_8g = (0x1 << BIT_ACCEL_CONFIG0_FS_SEL_POS),
    IIM42652_ACCEL_CONFIG0_FS_SEL_4g = (0x2 << BIT_ACCEL_CONFIG0_FS_SEL_POS),
    IIM42652_ACCEL_CONFIG0_FS_SEL_2g = (0x3 << BIT_ACCEL_CONFIG0_FS_SEL_POS),
} IIM42652_ACCEL_CONFIG0_FS_SEL_t;

/**
 * @brief Accelerometer Output Data Rate selection
 */
typedef enum {
    IIM42652_ACCEL_CONFIG0_ODR_32_KHZ = 0x1,
    IIM42652_ACCEL_CONFIG0_ODR_16_KHZ = 0x2,
    IIM42652_ACCEL_CONFIG0_ODR_8_KHZ = 0x3,
    IIM42652_ACCEL_CONFIG0_ODR_4_KHZ = 0x4,
    IIM42652_ACCEL_CONFIG0_ODR_2_KHZ = 0x5,
    IIM42652_ACCEL_CONFIG0_ODR_1_KHZ = 0x6,
    IIM42652_ACCEL_CONFIG0_ODR_200_HZ = 0x7,
    IIM42652_ACCEL_CONFIG0_ODR_100_HZ = 0x8,
    IIM42652_ACCEL_CONFIG0_ODR_50_HZ = 0x9,
    IIM42652_ACCEL_CONFIG0_ODR_25_HZ = 0xA,
    IIM42652_ACCEL_CONFIG0_ODR_12_5_HZ = 0xB,
    IIM42652_ACCEL_CONFIG0_ODR_6_25_HZ = 0xC,
    IIM42652_ACCEL_CONFIG0_ODR_3_125_HZ = 0xD,
    IIM42652_ACCEL_CONFIG0_ODR_1_5625_HZ = 0xE,
    IIM42652_ACCEL_CONFIG0_ODR_500_HZ = 0xF,
} IIM42652_ACCEL_CONFIG0_ODR_t;

/**
 * @brief Gyroscope Full Scale Range selection
 */
typedef enum {
    IIM42652_GYRO_CONFIG0_FS_SEL_2000dps = (0 << BIT_GYRO_CONFIG0_FS_SEL_POS),
    IIM42652_GYRO_CONFIG0_FS_SEL_1000dps = (1 << BIT_GYRO_CONFIG0_FS_SEL_POS),
    IIM42652_GYRO_CONFIG0_FS_SEL_500dps = (2 << BIT_GYRO_CONFIG0_FS_SEL_POS),
    IIM42652_GYRO_CONFIG0_FS_SEL_250dps = (3 << BIT_GYRO_CONFIG0_FS_SEL_POS),
    IIM42652_GYRO_CONFIG0_FS_SEL_125dps = (4 << BIT_GYRO_CONFIG0_FS_SEL_POS),
    IIM42652_GYRO_CONFIG0_FS_SEL_62dps = (5 << BIT_GYRO_CONFIG0_FS_SEL_POS),
    IIM42652_GYRO_CONFIG0_FS_SEL_31dps = (6 << BIT_GYRO_CONFIG0_FS_SEL_POS),
    IIM42652_GYRO_CONFIG0_FS_SEL_16dps = (7 << BIT_GYRO_CONFIG0_FS_SEL_POS),
} IIM42652_GYRO_CONFIG0_FS_SEL_t;

/**
 * @brief Gyroscope Output Data Rate selection
 */
typedef enum {
    IIM42652_GYRO_CONFIG0_ODR_32_KHZ = 0x01,
    IIM42652_GYRO_CONFIG0_ODR_16_KHZ = 0x02,
    IIM42652_GYRO_CONFIG0_ODR_8_KHZ = 0x03,
    IIM42652_GYRO_CONFIG0_ODR_4_KHZ = 0x04,
    IIM42652_GYRO_CONFIG0_ODR_2_KHZ = 0x05,
    IIM42652_GYRO_CONFIG0_ODR_1_KHZ = 0x06,
    IIM42652_GYRO_CONFIG0_ODR_200_HZ = 0x07,
    IIM42652_GYRO_CONFIG0_ODR_100_HZ = 0x08,
    IIM42652_GYRO_CONFIG0_ODR_50_HZ = 0x09,
    IIM42652_GYRO_CONFIG0_ODR_25_HZ = 0x0A,
    IIM42652_GYRO_CONFIG0_ODR_12_5_HZ = 0x0B,
    IIM42652_GYRO_CONFIG0_ODR_500_HZ = 0x0F,
} IIM42652_GYRO_CONFIG0_ODR_t;

/**
 * @brief Axis data structure
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} IIM42652_axis_t;

/* ============================================================================
 * IIM42652 Class Definition
 * ============================================================================ */

class IIM42652 {
   public:
    /**
     * @brief Constructor
     * @param i2c_dev Pointer to Zephyr I2C device
     * @param addr I2C address (default 0x69)
     */
    IIM42652(const struct device* i2c_dev, uint8_t addr = IIM42652_I2C_ADDR);

    /**
     * @brief Initialize the IIM42652 sensor
     * @return 0 on success, negative error code on failure
     */
    int begin(void);

    /**
     * @brief Get device ID
     * @param device_id Pointer to store device ID
     * @return 0 on success, negative error code on failure
     */
    int get_device_id(uint8_t* device_id);

    /**
     * @brief Select register bank
     * @param bank_sel Bank number (0-4)
     * @return 0 on success, negative error code on failure
     */
    int bank_selection(uint8_t bank_sel);

    /**
     * @brief Enable temperature sensor
     * @return 0 on success, negative error code on failure
     */
    int temperature_enable(void);

    /**
     * @brief Disable temperature sensor
     * @return 0 on success, negative error code on failure
     */
    int temperature_disable(void);

    /**
     * @brief Enable gyroscope
     * @return 0 on success, negative error code on failure
     */
    int gyroscope_enable(void);

    /**
     * @brief Disable gyroscope
     * @return 0 on success, negative error code on failure
     */
    int gyroscope_disable(void);

    /**
     * @brief Enable accelerometer
     * @return 0 on success, negative error code on failure
     */
    int accelerometer_enable(void);

    /**
     * @brief Disable accelerometer
     * @return 0 on success, negative error code on failure
     */
    int accelerometer_disable(void);

    /**
     * @brief Enter idle mode (powers off RC oscillator)
     * @return 0 on success, negative error code on failure
     */
    int idle(void);

    /**
     * @brief Exit idle mode (keeps RC oscillator on)
     * @return 0 on success, negative error code on failure
     */
    int ex_idle(void);

    /**
     * @brief Perform software reset
     * @return 0 on success, negative error code on failure
     */
    int soft_reset(void);

    /**
     * @brief Get accelerometer data
     * @param accel_data Pointer to axis structure
     * @return 0 on success, negative error code on failure
     */
    int get_accel_data(IIM42652_axis_t* accel_data);

    /**
     * @brief Get gyroscope data
     * @param gyro_data Pointer to axis structure
     * @return 0 on success, negative error code on failure
     */
    int get_gyro_data(IIM42652_axis_t* gyro_data);

    /**
     * @brief Get temperature
     * @param temperature Pointer to temperature variable
     * @return 0 on success, negative error code on failure
     */
    int get_temperature(float* temperature);

    /**
     * @brief Set accelerometer full scale range
     * @param accel_fsr_g Full scale range
     * @return 0 on success, negative error code on failure
     */
    int set_accel_fsr(IIM42652_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g);

    /**
     * @brief Set accelerometer output data rate
     * @param frequency Output data rate
     * @return 0 on success, negative error code on failure
     */
    int set_accel_frequency(IIM42652_ACCEL_CONFIG0_ODR_t frequency);

    /**
     * @brief Set gyroscope full scale range
     * @param gyro_fsr_dps Full scale range
     * @return 0 on success, negative error code on failure
     */
    int set_gyro_fsr(IIM42652_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps);

    /**
     * @brief Set gyroscope output data rate
     * @param frequency Output data rate
     * @return 0 on success, negative error code on failure
     */
    int set_gyro_frequency(IIM42652_GYRO_CONFIG0_ODR_t frequency);

    /**
     * @brief Configure wake on motion
     * @param x_th X-axis threshold
     * @param y_th Y-axis threshold
     * @param z_th Z-axis threshold
     * @return 0 on success, negative error code on failure
     */
    int wake_on_motion_configuration(uint8_t x_th, uint8_t y_th, uint8_t z_th);

    /**
     * @brief Get wake on motion interrupt status
     * @return Interrupt status byte
     */
    uint8_t get_WOM_INT(void);

    /**
     * @brief Enable accelerometer low power mode
     * @return 0 on success, negative error code on failure
     */
    int enable_accel_low_power_mode(void);

    /**
     * @brief Configure pedometer
     * @return 0 on success, negative error code on failure
     */
    int pedometer_configuration(void);

    /**
     * @brief Get pedometer step count
     * @return Step count
     */
    uint16_t get_pedometer_data(void);

    /**
     * @brief Write to register(s)
     * @param reg_addr Register address
     * @param write_data Pointer to data to write
     * @param size Number of bytes to write
     * @return 0 on success, negative error code on failure
     */
    int write_register(uint8_t reg_addr, uint8_t* write_data, uint8_t size);

    /**
     * @brief Read from register(s)
     * @param reg_addr Register address
     * @param read_data Pointer to buffer for read data
     * @param size Number of bytes to read
     * @return 0 on success, negative error code on failure
     */
    int read_register(uint8_t reg_addr, uint8_t* read_data, uint8_t size);

   private:
    const struct device* i2c_dev;
    uint8_t device_address;
};

/* ============================================================================
 * IIM42652 Class Implementation
 * ============================================================================ */

IIM42652::IIM42652(const struct device* i2c_dev, uint8_t addr) : i2c_dev(i2c_dev), device_address(addr) {}

int IIM42652::write_register(uint8_t reg_addr, uint8_t* write_data, uint8_t size) {
    uint8_t buf[size + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], write_data, size);

    int ret = i2c_write(i2c_dev, buf, size + 1, device_address);
    if (ret != 0) {
        LOG_ERR("I2C write failed: %d", ret);
        return ret;
    }
    return 0;
}

int IIM42652::read_register(uint8_t reg_addr, uint8_t* read_data, uint8_t size) {
    int ret = i2c_write_read(i2c_dev, device_address, &reg_addr, 1, read_data, size);
    if (ret != 0) {
        LOG_ERR("I2C read failed: %d", ret);
        return ret;
    }
    return 0;
}

int IIM42652::begin(void) {
    uint8_t sensor_id;

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    LOG_INF("Initializing IIM42652");
    k_msleep(100);

    int ret = get_device_id(&sensor_id);
    if (ret != 0) {
        return ret;
    }

    LOG_INF("Device ID: 0x%02X", sensor_id);

    if (sensor_id == IIM42652_CHIP_ID) {
        ret = soft_reset();
        if (ret == 0) {
            LOG_INF("IIM42652 initialized successfully");
        }
        return ret;
    } else {
        LOG_ERR("Invalid chip ID: 0x%02X (expected 0x%02X)", sensor_id, IIM42652_CHIP_ID);
        return -EINVAL;
    }
}

int IIM42652::bank_selection(uint8_t bank_sel) {
    int ret = write_register(IIM42652_REG_BANK_SEL, &bank_sel, 1);
    if (ret != 0) {
        return ret;
    }

    uint8_t tmp;
    ret = read_register(IIM42652_REG_BANK_SEL, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    LOG_DBG("Bank selected: %d", tmp);
    k_msleep(1);
    return 0;
}

int IIM42652::get_device_id(uint8_t* device_id) { return read_register(IIM42652_REG_WHO_AM_I, device_id, 1); }

int IIM42652::temperature_enable(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= ~IIM42652_SET_TEMPERATURE_DISABLED;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::temperature_disable(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= IIM42652_SET_TEMPERATURE_DISABLED;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::idle(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= 0xEF;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::ex_idle(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= ~0xEF;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::gyroscope_enable(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= IIM42652_SET_GYRO_TLOW_NOISE_MODE;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::gyroscope_disable(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= 0xF3;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::accelerometer_enable(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= IIM42652_SET_ACCEL_LOW_NOISE_MODE;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::accelerometer_disable(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= 0xFC;
    return write_register(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

int IIM42652::soft_reset(void) {
    uint8_t tmp;
    int ret = read_register(IIM42652_REG_DEVICE_CONFIG, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= 0x01;
    ret = write_register(IIM42652_REG_DEVICE_CONFIG, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    k_msleep(10);
    return 0;
}

int IIM42652::get_accel_data(IIM42652_axis_t* accel_data) {
    uint8_t rx_buf[6];
    uint16_t tmp;

    int ret = read_register(IIM42652_REG_ACCEL_DATA_X1_UI, rx_buf, 6);
    if (ret != 0) {
        return ret;
    }

    tmp = rx_buf[0];
    tmp <<= 8;
    tmp |= rx_buf[1];
    accel_data->x = (int16_t)tmp;

    tmp = rx_buf[2];
    tmp <<= 8;
    tmp |= rx_buf[3];
    accel_data->y = (int16_t)tmp;

    tmp = rx_buf[4];
    tmp <<= 8;
    tmp |= rx_buf[5];
    accel_data->z = (int16_t)tmp;

    return 0;
}

int IIM42652::get_gyro_data(IIM42652_axis_t* gyro_data) {
    uint8_t rx_buf[6];
    uint16_t tmp;

    int ret = read_register(IIM42652_REG_GYRO_DATA_X1_UI, rx_buf, 6);
    if (ret != 0) {
        return ret;
    }

    tmp = rx_buf[0];
    tmp <<= 8;
    tmp |= rx_buf[1];
    gyro_data->x = (int16_t)tmp;

    tmp = rx_buf[2];
    tmp <<= 8;
    tmp |= rx_buf[3];
    gyro_data->y = (int16_t)tmp;

    tmp = rx_buf[4];
    tmp <<= 8;
    tmp |= rx_buf[5];
    gyro_data->z = (int16_t)tmp;

    return 0;
}

int IIM42652::get_temperature(float* temperature) {
    uint8_t rx_buf[2];
    int16_t tmp;

    int ret = read_register(IIM42652_REG_TEMP_DATA1_UI, rx_buf, 2);
    if (ret != 0) {
        return ret;
    }

    tmp = rx_buf[0];
    tmp <<= 8;
    tmp |= rx_buf[1];

    *temperature = (float)tmp;
    *temperature /= 132.48f;
    *temperature += 25.0f;

    return 0;
}

int IIM42652::set_accel_fsr(IIM42652_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g) {
    uint8_t accel_cfg_0_reg;

    int ret = read_register(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
    accel_cfg_0_reg |= (uint8_t)accel_fsr_g;

    return write_register(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
}

int IIM42652::set_accel_frequency(IIM42652_ACCEL_CONFIG0_ODR_t frequency) {
    uint8_t accel_cfg_0_reg;

    int ret = read_register(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_ODR_MASK;
    accel_cfg_0_reg |= (uint8_t)frequency;

    return write_register(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
}

int IIM42652::set_gyro_fsr(IIM42652_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps) {
    uint8_t gyro_cfg_0_reg;

    int ret = read_register(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_FS_SEL_MASK;
    gyro_cfg_0_reg |= (uint8_t)gyro_fsr_dps;

    return write_register(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
}

int IIM42652::set_gyro_frequency(IIM42652_GYRO_CONFIG0_ODR_t frequency) {
    uint8_t gyro_cfg_0_reg;

    int ret = read_register(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_ODR_MASK;
    gyro_cfg_0_reg |= (uint8_t)frequency;

    return write_register(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
}

uint8_t IIM42652::get_WOM_INT(void) {
    uint8_t data;
    read_register(IIM42652_REG_INT_STATUS2, &data, 1);
    LOG_DBG("WOM INT status: 0x%02X", data);
    return data;
}

int IIM42652::wake_on_motion_configuration(uint8_t x_th, uint8_t y_th, uint8_t z_th) {
    uint8_t data[3];
    int ret;

    ret = read_register(IIM42652_REG_INT_CONFIG, data, 1);
    if (ret != 0) return ret;

    data[0] |= 0x02;
    ret = write_register(IIM42652_REG_INT_CONFIG, data, 1);
    if (ret != 0) return ret;

    // Set memory bank 4
    ret = bank_selection(IIM42652_SET_BANK_4);
    if (ret != 0) return ret;

    data[0] = x_th;
    data[1] = y_th;
    data[2] = z_th;
    ret = write_register(IIM42652_REG_ACCEL_WOM_X_THR, data, 3);
    if (ret != 0) return ret;

    k_msleep(1);

    // Set memory bank 0
    ret = bank_selection(IIM42652_SET_BANK_0);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_INT_SOURCE1, data, 1);
    if (ret != 0) return ret;

    data[0] |= X_INT1_EN | Y_INT1_EN | Z_INT1_EN;
    ret = write_register(IIM42652_REG_INT_SOURCE1, data, 1);
    if (ret != 0) return ret;

    k_msleep(100);

    // Turn on WOM feature
    ret = read_register(IIM42652_REG_SMD_CONFIG, data, 1);
    if (ret != 0) return ret;

    data[0] |= 0x05;
    return write_register(IIM42652_REG_SMD_CONFIG, data, 1);
}

int IIM42652::enable_accel_low_power_mode(void) {
    uint8_t pwr_mgmt0_reg;
    int ret;

    ret = read_register(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
    pwr_mgmt0_reg |= IIM42652_PWR_MGMT_0_ACCEL_MODE_LP;
    ret = write_register(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg &= ~BIT_ACCEL_LP_CLK_SEL_MASK;
    pwr_mgmt0_reg |= IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC;
    ret = write_register(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    k_msleep(1);
    return 0;
}

int IIM42652::pedometer_configuration(void) {
    uint8_t pwr_mgmt0_reg;
    int ret;

    // Enable/Switch the accelerometer in/to low power mode
    ret = read_register(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
    pwr_mgmt0_reg |= IIM42652_PWR_MGMT_0_ACCEL_MODE_LP;
    ret = write_register(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg &= ~BIT_ACCEL_LP_CLK_SEL_MASK;
    pwr_mgmt0_reg |= IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC;
    ret = write_register(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    // Set DMP ODR = 50 Hz and turn on Pedometer feature
    ret = read_register(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg |= 0x22;
    ret = write_register(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    k_msleep(1);

    ret = read_register(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg |= 0x20;
    ret = write_register(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    k_msleep(1);

    // Set LOW_ENERGY_AMP_TH_SEL to 10 (Register 0x40h in Bank 4)
    ret = bank_selection(IIM42652_SET_BANK_4);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_APEX_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg |= 0xA0;
    ret = write_register(IIM42652_REG_APEX_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    // Set DMP_INIT_EN to 1
    ret = bank_selection(IIM42652_SET_BANK_0);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg |= 0x40;
    ret = write_register(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    k_msleep(50);

    ret = bank_selection(IIM42652_SET_BANK_4);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_INT_SOURCE6, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg |= 0x20;
    ret = write_register(IIM42652_REG_INT_SOURCE6, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    // Disable freefall
    ret = bank_selection(IIM42652_SET_BANK_0);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg &= 0xFB;
    ret = write_register(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    ret = read_register(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg |= 0x20;
    return write_register(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
}

uint16_t IIM42652::get_pedometer_data(void) {
    uint8_t data[2];
    uint16_t tmp;

    read_register(0x31, data, 2);  // IIM42652_REG_APEX_DATA0

    tmp = data[1];
    tmp <<= 8;
    tmp |= data[0];

    return tmp;
}

/* ============================================================================
 * Example Usage (main function)
 * ============================================================================ */

/**
 * @brief Example usage - basic IMU reading
 *
 * To use this in your Zephyr application:
 * 1. Add this to your prj.conf:
 *    CONFIG_I2C=y
 *    CONFIG_LOG=y
 *
 * 2. In your device tree overlay, define your I2C bus:
 *    &i2c0 {
 *        status = "okay";
 *        clock-frequency = <I2C_BITRATE_FAST>;
 *    };
 *
 * 3. Call this function from your main:
 */
void iim42652_example(void) {
    // Get I2C device from device tree
    const struct device* i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return;
    }

    // Create IIM42652 instance
    IIM42652 imu(i2c_dev, IIM42652_I2C_ADDR);

    // Initialize sensor
    if (imu.begin() != 0) {
        LOG_ERR("Failed to initialize IIM42652");
        return;
    }

    // Main loop
    while (1) {
        IIM42652_axis_t accel_data;
        IIM42652_axis_t gyro_data;
        float temperature;

        // Enable sensors
        imu.ex_idle();
        imu.accelerometer_enable();
        imu.gyroscope_enable();
        imu.temperature_enable();

        k_msleep(100);

        // Read sensor data
        if (imu.get_accel_data(&accel_data) == 0) {
            // Convert to g (±16g range: 2048 LSB/g)
            float acc_x = (float)accel_data.x / 2048.0f;
            float acc_y = (float)accel_data.y / 2048.0f;
            float acc_z = (float)accel_data.z / 2048.0f;

            LOG_INF("Accel X: %.3f g, Y: %.3f g, Z: %.3f g", acc_x, acc_y, acc_z);
        }

        if (imu.get_gyro_data(&gyro_data) == 0) {
            // Convert to °/s (±2000°/s range: 16.4 LSB/(°/s))
            float gyro_x = (float)gyro_data.x / 16.4f;
            float gyro_y = (float)gyro_data.y / 16.4f;
            float gyro_z = (float)gyro_data.z / 16.4f;

            LOG_INF("Gyro X: %.2f °/s, Y: %.2f °/s, Z: %.2f °/s", gyro_x, gyro_y, gyro_z);
        }

        if (imu.get_temperature(&temperature) == 0) {
            LOG_INF("Temperature: %.2f °C", temperature);
        }

        // Disable sensors to save power
        imu.accelerometer_disable();
        imu.gyroscope_disable();
        imu.temperature_disable();
        imu.idle();

        k_msleep(2000);
    }
}

// Uncomment to use as standalone application
int main(void) {
    iim42652_example();
    return 0;
}