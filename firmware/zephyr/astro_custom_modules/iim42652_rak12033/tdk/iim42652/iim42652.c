/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tdk_iim42652

#include "iim42652.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(IIM42652, CONFIG_SENSOR_LOG_LEVEL);

struct iim42652_config {
    struct i2c_dt_spec i2c_bus;
};

struct iim42652_data {
    struct k_mutex bus_lock;
    struct iim42652_vector3_data acc;
    struct iim42652_vector3_data gyro;
    float temperature;
};

static int write_register(const struct device* dev, uint8_t reg_addr, uint8_t* write_data, uint8_t size) {
    const struct iim42652_config* config = dev->config;
    uint8_t buf[size + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], write_data, size);

    int ret = i2c_write_dt(&config->i2c_bus, buf, size + 1);
    if (ret != 0) {
        // LOG_ERR("I2C write failed: %d", ret);
        return ret;
    }
    return 0;
}

static int read_register(const struct device* dev, uint8_t reg_addr, uint8_t* read_data, uint8_t size) {
    const struct iim42652_config* config = dev->config;
    int ret = i2c_write_read_dt(&config->i2c_bus, &reg_addr, 1, read_data, size);
    if (ret != 0) {
        LOG_ERR("I2C read reg 0x%02X failed: %d", reg_addr, ret);
        return ret;
    }
    return 0;
}

static int get_device_id(const struct device* dev, uint8_t* device_id) {
    return read_register(dev, IIM42652_REG_WHO_AM_I, device_id, 1);
}

static int bank_selection(const struct device* dev, uint8_t bank_sel) {
    int ret = write_register(dev, IIM42652_REG_BANK_SEL, &bank_sel, 1);
    if (ret != 0) {
        return ret;
    }

    uint8_t tmp;
    ret = read_register(dev, IIM42652_REG_BANK_SEL, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    LOG_DBG("Bank selected: %d", tmp);
    k_msleep(1);

    /* Verify the bank switch actually took effect.  After an MCU soft-reset
     * without a power cycle the sensor can be in a non-zero bank; if the
     * read-back doesn't match we must signal failure so the caller retries. */
    if (tmp != bank_sel) {
        LOG_ERR("Bank select mismatch: wrote %d, read back %d", bank_sel, tmp);
        return -EIO;
    }

    return 0;
}

static int temperature_enable(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= ~IIM42652_SET_TEMPERATURE_DISABLED;
    return write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

static int temperature_disable(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= IIM42652_SET_TEMPERATURE_DISABLED;
    return write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

static int gyroscope_enable(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= IIM42652_SET_GYRO_TLOW_NOISE_MODE;
    int ret2 = write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret2 != 0) return ret2;
    k_usleep(200); /* datasheet: wait 200µs after transitioning from OFF before next register write */
    return 0;
}

static int gyroscope_disable(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= (uint8_t)~IIM42652_SET_GYRO_TLOW_NOISE_MODE;
    return write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

static int accelerometer_enable(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= IIM42652_SET_ACCEL_LOW_NOISE_MODE;
    int ret2 = write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret2 != 0) return ret2;
    k_usleep(200); /* datasheet: wait 200µs after transitioning from OFF before next register write */
    return 0;
}

static int accelerometer_disable(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= (uint8_t)~IIM42652_SET_ACCEL_LOW_NOISE_MODE;
    return write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

static int idle(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp &= 0xEF;
    return write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

static int ex_idle(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= ~0xEF;
    return write_register(dev, IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

static int soft_reset(const struct device* dev) {
    uint8_t tmp;
    int ret = read_register(dev, IIM42652_REG_DEVICE_CONFIG, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    tmp |= 0x01;
    ret = write_register(dev, IIM42652_REG_DEVICE_CONFIG, &tmp, 1);
    if (ret != 0) {
        return ret;
    }

    k_msleep(10);
    return 0;
}

static int get_accel_data(const struct device* dev, IIM42652_axis_t* accel_data) {
    uint8_t rx_buf[6];
    uint16_t tmp;

    int ret = read_register(dev, IIM42652_REG_ACCEL_DATA_X1_UI, rx_buf, 6);
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

static int get_gyro_data(const struct device* dev, IIM42652_axis_t* gyro_data) {
    uint8_t rx_buf[6];
    uint16_t tmp;

    int ret = read_register(dev, IIM42652_REG_GYRO_DATA_X1_UI, rx_buf, 6);
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

static int get_temperature(const struct device* dev, float* temperature) {
    uint8_t rx_buf[2];
    int16_t tmp;

    int ret = read_register(dev, IIM42652_REG_TEMP_DATA1_UI, rx_buf, 2);
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

static int set_accel_fsr(const struct device* dev, IIM42652_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g) {
    uint8_t accel_cfg_0_reg;

    int ret = read_register(dev, IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
    accel_cfg_0_reg |= (uint8_t)accel_fsr_g;

    return write_register(dev, IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
}

static int set_accel_frequency(const struct device* dev, IIM42652_ACCEL_CONFIG0_ODR_t frequency) {
    uint8_t accel_cfg_0_reg;

    int ret = read_register(dev, IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_ODR_MASK;
    accel_cfg_0_reg |= (uint8_t)frequency;

    return write_register(dev, IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
}

static int set_gyro_fsr(const struct device* dev, IIM42652_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps) {
    uint8_t gyro_cfg_0_reg;

    int ret = read_register(dev, IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_FS_SEL_MASK;
    gyro_cfg_0_reg |= (uint8_t)gyro_fsr_dps;

    return write_register(dev, IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
}

static int set_gyro_frequency(const struct device* dev, IIM42652_GYRO_CONFIG0_ODR_t frequency) {
    uint8_t gyro_cfg_0_reg;

    int ret = read_register(dev, IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
    if (ret != 0) {
        return ret;
    }

    gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_ODR_MASK;
    gyro_cfg_0_reg |= (uint8_t)frequency;

    return write_register(dev, IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
}

static int wake_on_motion_configuration(const struct device* dev, uint8_t x_th, uint8_t y_th, uint8_t z_th) {
    uint8_t data[3];
    int ret;

    ret = read_register(dev, IIM42652_REG_INT_CONFIG, data, 1);
    if (ret != 0) return ret;

    data[0] |= 0x02;
    ret = write_register(dev, IIM42652_REG_INT_CONFIG, data, 1);
    if (ret != 0) return ret;

    // Set memory bank 4
    ret = bank_selection(dev, IIM42652_SET_BANK_4);
    if (ret != 0) return ret;

    data[0] = x_th;
    data[1] = y_th;
    data[2] = z_th;
    ret = write_register(dev, IIM42652_REG_ACCEL_WOM_X_THR, data, 3);
    if (ret != 0) return ret;

    k_msleep(1);

    // Set memory bank 0
    ret = bank_selection(dev, IIM42652_SET_BANK_0);
    if (ret != 0) return ret;

    ret = read_register(dev, IIM42652_REG_INT_SOURCE1, data, 1);
    if (ret != 0) return ret;

    data[0] |= X_INT1_EN | Y_INT1_EN | Z_INT1_EN;
    ret = write_register(dev, IIM42652_REG_INT_SOURCE1, data, 1);
    if (ret != 0) return ret;

    k_msleep(100);

    // Turn on WOM feature
    ret = read_register(dev, IIM42652_REG_SMD_CONFIG, data, 1);
    if (ret != 0) return ret;

    data[0] |= 0x05;
    return write_register(dev, IIM42652_REG_SMD_CONFIG, data, 1);
}

uint8_t get_WOM_INT(const struct device* dev) {
    uint8_t data = 0;
    int ret = read_register(dev, IIM42652_REG_INT_STATUS2, &data, 1);
    if (ret != 0) {
        LOG_ERR("Failed to read WOM INT status: %d", ret);
        return 0;
    }
    LOG_DBG("WOM INT status: 0x%02X", data);
    return data;
}

static int enable_accel_low_power_mode(const struct device* dev) {
    uint8_t pwr_mgmt0_reg;
    int ret;

    ret = read_register(dev, IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
    pwr_mgmt0_reg |= IIM42652_PWR_MGMT_0_ACCEL_MODE_LP;
    ret = write_register(dev, IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    ret = read_register(dev, IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    pwr_mgmt0_reg &= ~BIT_ACCEL_LP_CLK_SEL_MASK;
    pwr_mgmt0_reg |= IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC;
    ret = write_register(dev, IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
    if (ret != 0) return ret;

    k_msleep(1);
    return 0;
}

static int iim42652_attr_set(const struct device* dev, enum sensor_channel chan, enum sensor_attribute attr,
                             const struct sensor_value* val) {
    if (attr == SENSOR_ATTR_FULL_SCALE) {
        if (chan == SENSOR_CHAN_ACCEL_XYZ) {
            IIM42652_ACCEL_CONFIG0_FS_SEL_t fsr;
            switch (val->val1) {
                case 2:
                    fsr = IIM42652_ACCEL_CONFIG0_FS_SEL_2g;
                    break;
                case 4:
                    fsr = IIM42652_ACCEL_CONFIG0_FS_SEL_4g;
                    break;
                case 8:
                    fsr = IIM42652_ACCEL_CONFIG0_FS_SEL_8g;
                    break;
                case 16:
                    fsr = IIM42652_ACCEL_CONFIG0_FS_SEL_16g;
                    break;
                default:
                    return -EINVAL;
            }
            return set_accel_fsr(dev, fsr);
        }
        if (chan == SENSOR_CHAN_GYRO_XYZ) {
            IIM42652_GYRO_CONFIG0_FS_SEL_t fsr;
            switch (val->val1) {
                case 16:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_16dps;
                    break;
                case 31:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_31dps;
                    break;
                case 62:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_62dps;
                    break;
                case 125:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_125dps;
                    break;
                case 250:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_250dps;
                    break;
                case 500:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_500dps;
                    break;
                case 1000:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_1000dps;
                    break;
                case 2000:
                    fsr = IIM42652_GYRO_CONFIG0_FS_SEL_2000dps;
                    break;
                default:
                    return -EINVAL;
            }
            return set_gyro_fsr(dev, fsr);
        }
        return -ENOTSUP;
    }

    if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
        /* val1 is integer Hz, val2 is fractional micro-Hz */
        if (chan == SENSOR_CHAN_ACCEL_XYZ) {
            IIM42652_ACCEL_CONFIG0_ODR_t odr;
            if (val->val1 == 32000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_32_KHZ;
            else if (val->val1 == 16000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_16_KHZ;
            else if (val->val1 == 8000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_8_KHZ;
            else if (val->val1 == 4000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_4_KHZ;
            else if (val->val1 == 2000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_2_KHZ;
            else if (val->val1 == 1000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_1_KHZ;
            else if (val->val1 == 500)
                odr = IIM42652_ACCEL_CONFIG0_ODR_500_HZ;
            else if (val->val1 == 200)
                odr = IIM42652_ACCEL_CONFIG0_ODR_200_HZ;
            else if (val->val1 == 100)
                odr = IIM42652_ACCEL_CONFIG0_ODR_100_HZ;
            else if (val->val1 == 50)
                odr = IIM42652_ACCEL_CONFIG0_ODR_50_HZ;
            else if (val->val1 == 25)
                odr = IIM42652_ACCEL_CONFIG0_ODR_25_HZ;
            else if (val->val1 == 12 && val->val2 == 500000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_12_5_HZ;
            else if (val->val1 == 6 && val->val2 == 250000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_6_25_HZ;
            else if (val->val1 == 3 && val->val2 == 125000)
                odr = IIM42652_ACCEL_CONFIG0_ODR_3_125_HZ;
            else if (val->val1 == 1 && val->val2 == 562500)
                odr = IIM42652_ACCEL_CONFIG0_ODR_1_5625_HZ;
            else
                return -EINVAL;
            return set_accel_frequency(dev, odr);
        }
        if (chan == SENSOR_CHAN_GYRO_XYZ) {
            IIM42652_GYRO_CONFIG0_ODR_t odr;
            if (val->val1 == 32000)
                odr = IIM42652_GYRO_CONFIG0_ODR_32_KHZ;
            else if (val->val1 == 16000)
                odr = IIM42652_GYRO_CONFIG0_ODR_16_KHZ;
            else if (val->val1 == 8000)
                odr = IIM42652_GYRO_CONFIG0_ODR_8_KHZ;
            else if (val->val1 == 4000)
                odr = IIM42652_GYRO_CONFIG0_ODR_4_KHZ;
            else if (val->val1 == 2000)
                odr = IIM42652_GYRO_CONFIG0_ODR_2_KHZ;
            else if (val->val1 == 1000)
                odr = IIM42652_GYRO_CONFIG0_ODR_1_KHZ;
            else if (val->val1 == 500)
                odr = IIM42652_GYRO_CONFIG0_ODR_500_HZ;
            else if (val->val1 == 200)
                odr = IIM42652_GYRO_CONFIG0_ODR_200_HZ;
            else if (val->val1 == 100)
                odr = IIM42652_GYRO_CONFIG0_ODR_100_HZ;
            else if (val->val1 == 50)
                odr = IIM42652_GYRO_CONFIG0_ODR_50_HZ;
            else if (val->val1 == 25)
                odr = IIM42652_GYRO_CONFIG0_ODR_25_HZ;
            else if (val->val1 == 12 && val->val2 == 500000)
                odr = IIM42652_GYRO_CONFIG0_ODR_12_5_HZ;
            else
                return -EINVAL;
            return set_gyro_frequency(dev, odr);
        }
        return -ENOTSUP;
    }

    return -ENOTSUP;
}

static void float_to_sensor_value(float f, struct sensor_value* v) {
    /* Zephyr sensor_value requires val1 and val2 to have the same sign,
     * with val2 representing the *unsigned* fractional part (0..999999).
     * Casting a negative float like -0.5 gives val1=0, val2=-500000 which
     * violates the contract and causes sensor_value_to_double() to return 0
     * for all values in (-1, 0).  Use truncation toward -infinity instead. */
    v->val1 = (int32_t)floorf(f);
    v->val2 = (int32_t)((f - (float)v->val1) * 1000000.0f);
}

static int iim42652_channel_get(const struct device* dev, enum sensor_channel chan, struct sensor_value* val) {
    struct iim42652_data* data = dev->data;

    if (chan == SENSOR_CHAN_ACCEL_XYZ) {
        float_to_sensor_value(data->acc.x, &val[0]);
        float_to_sensor_value(data->acc.y, &val[1]);
        float_to_sensor_value(data->acc.z, &val[2]);
        return 0;
    }

    if (chan == SENSOR_CHAN_GYRO_XYZ) {
        float_to_sensor_value(data->gyro.x, &val[0]);
        float_to_sensor_value(data->gyro.y, &val[1]);
        float_to_sensor_value(data->gyro.z, &val[2]);
        return 0;
    }

    if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
        float_to_sensor_value(data->temperature, &val[0]);
        return 0;
    }

    return -ENOTSUP;
}

static int iim42652_sample_fetch(const struct device* dev, enum sensor_channel chan) {
    struct iim42652_data* data = dev->data;
    int ret;

    k_mutex_lock(&data->bus_lock, K_FOREVER);

    /* Burst-read temp + accel + gyro in a single I2C transaction (0x1D–0x2A,
     * 14 bytes) so accel and gyro always come from the same latch cycle.
     * The mutex prevents any other driver on the shared i2c0 bus from
     * interleaving between the two transfers. */
    uint8_t raw[14];
    ret = read_register(dev, IIM42652_REG_TEMP_DATA1_UI, raw, sizeof(raw));
    k_mutex_unlock(&data->bus_lock);
    if (ret != 0) {
        return ret;
    }

    /* Temperature: raw[0]=TEMP1, raw[1]=TEMP0 */
    int16_t temp_raw = (int16_t)((raw[0] << 8) | raw[1]);
    data->temperature = (float)temp_raw / 132.48f + 25.0f;

    /* Accel: raw[2..7] = X1,X0,Y1,Y0,Z1,Z0 */
    int16_t ax = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t ay = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t az = (int16_t)((raw[6] << 8) | raw[7]);
    /* FSR ±16g default: 2048 LSB/g */
    data->acc.x = (float)ax / 2048.0f * 9.80665f;
    data->acc.y = (float)ay / 2048.0f * 9.80665f;
    data->acc.z = (float)az / 2048.0f * 9.80665f;

    /* Gyro: raw[8..13] = X1,X0,Y1,Y0,Z1,Z0 */
    int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
    /* FSR ±2000 dps default: 16.4 LSB/(dps) */
    data->gyro.x = (float)gx / 16.4f;
    data->gyro.y = (float)gy / 16.4f;
    data->gyro.z = (float)gz / 16.4f;

    LOG_DBG("IIM42652 accel %.3f %.3f %.3f m/s2  gyro %.2f %.2f %.2f dps", data->acc.x, data->acc.y, data->acc.z,
            data->gyro.x, data->gyro.y, data->gyro.z);

    return 0;
}

static int iim42652_init(const struct device* dev) {
    // Initialize I2C bus
    const struct iim42652_config* config = dev->config;
    uint8_t sensor_id;

    if (!i2c_is_ready_dt(&config->i2c_bus)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    LOG_INF("Initializing IIM42652");
    k_mutex_init(&((struct iim42652_data*)dev->data)->bus_lock);
    k_msleep(100);

    /* After a warm MCU reset the I2C bus can be left stuck (SDA held low by
     * the sensor mid-transaction).  Recover it with 9 SCL pulses before
     * attempting any register access. */
    int ret = i2c_recover_bus(config->i2c_bus.bus);
    if (ret != 0) {
        LOG_WRN("I2C bus recovery failed: %d (continuing anyway)", ret);
    }
    k_msleep(10);

    /* After an MCU soft-reset (without power-cycling the sensor) the IIM42652
     * retains its register-bank selection.  WHO_AM_I (0x75) only returns 0x6F
     * when bank 0 is active; any other bank maps that address to a different
     * register (e.g. 0x5F or 0x7F), causing a spurious "Invalid chip ID" error.
     * Always select bank 0 before the ID read so init is idempotent. */
    ret = bank_selection(dev, 0);
    if (ret != 0) {
        LOG_ERR("Failed to select bank 0: %d", ret);
        return ret;
    }

    ret = get_device_id(dev, &sensor_id);
    if (ret != 0) {
        return ret;
    }

    LOG_INF("Device ID: 0x%02X", sensor_id);

    if (sensor_id == IIM42652_CHIP_ID) {
        ret = soft_reset(dev);
        if (ret != 0) {
            return ret;
        }

        /* Enable all sensors persistently — they stay on between fetches */
        ret = ex_idle(dev);
        if (ret != 0) return ret;
        ret = accelerometer_enable(dev);
        if (ret != 0) return ret;
        ret = gyroscope_enable(dev);
        if (ret != 0) return ret;
        ret = temperature_enable(dev);
        if (ret != 0) return ret;

        /* Set default ODR: 100 Hz for both accel and gyro.
         * Without an explicit ODR write the sensor powers up at hardware-reset
         * default (1 kHz), but leaving it unconfigured means sample_fetch's
         * DRDY gate is the only protection — an explicit rate is safer. */
        set_accel_frequency(dev, IIM42652_ACCEL_CONFIG0_ODR_100_HZ);
        set_gyro_frequency(dev, IIM42652_GYRO_CONFIG0_ODR_100_HZ);

        /* Gyro needs 45ms minimum on-time before first valid sample (datasheet) */
        k_msleep(50);

        /* Read back PWR_MGMT0 to verify sensors are enabled */
        uint8_t pwr_check;
        if (read_register(dev, IIM42652_REG_PWR_MGMT0, &pwr_check, 1) == 0) {
            LOG_INF("PWR_MGMT0 after init: 0x%02X (expected 0x1F)", pwr_check);
            if ((pwr_check & 0x0F) != 0x0F) {
                LOG_ERR("IIM42652: sensors not enabled (PWR_MGMT0=0x%02X) - accel/gyro bits wrong", pwr_check);
            }
        }

        LOG_INF("IIM42652 initialized successfully");
        return 0;
    } else {
        LOG_ERR("Invalid chip ID: 0x%02X (expected 0x%02X)", sensor_id, IIM42652_CHIP_ID);
        return -EINVAL;
    }
}

static const struct sensor_driver_api iim42652_driver_api = {
    .attr_set = iim42652_attr_set,
    .sample_fetch = iim42652_sample_fetch,
    .channel_get = iim42652_channel_get,
#if IIM42652_USE_IRQ
    .trigger_set = iim42652_trigger_set,
#endif
};

#define IIM42652_INIT(n)                                                                                 \
    static struct iim42652_config iim42652_config_##n = {                                                \
        .i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                              \
    };                                                                                                   \
    static struct iim42652_data iim42652_data_##n;                                                       \
    DEVICE_DT_INST_DEFINE(n, iim42652_init, NULL, &iim42652_data_##n, &iim42652_config_##n, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &iim42652_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IIM42652_INIT)
