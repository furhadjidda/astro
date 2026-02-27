/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bno055

#include "bno055.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

LOG_MODULE_REGISTER(BNO055, CONFIG_SENSOR_LOG_LEVEL);

struct bno055_config {
    struct i2c_dt_spec i2c_bus;
    bool use_xtal;
    bool deferred;
};

struct bno055_data {
    uint8_t current_page;
    bno055_opmode_t mode;
    bno055_powermode_t power;

    struct bno055_vector3_data acc;
    struct bno055_vector3_data mag;
    struct bno055_vector3_data gyr;

    struct bno055_vector3_data eul;
    struct bno055_vector4_data qua;
    struct bno055_vector3_data lia;
    struct bno055_vector3_data grv;

    struct bno055_calib_data calib;
    struct bno055_system_status sys_status;
};

static int bno055_set_page(const struct device* dev, enum bno055_PageId page) {
    const struct bno055_config* config = dev->config;
    struct bno055_data* data = dev->data;
    uint8_t reg;
    int err;

    LOG_DBG("FUNC PAGE[%d]", page);
    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_PAGE_ID_ADDR, &reg);
    if (err < 0) {
        return err;
    }

    if (data->current_page != (reg & BNO055_PAGE_ID_MASK)) {
        LOG_WRN("Update page index from I2C page register [%d]<-[%d]!!", data->current_page, reg & BNO055_PAGE_ID_MASK);
        data->current_page = reg & BNO055_PAGE_ID_MASK;
    }

    if ((reg & BNO055_PAGE_ID_MASK) == page) {
        LOG_DBG("I2C page register already good!!");
        return 0;
    }

    /* Write PAGE */
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_PAGE_ID_ADDR, page);
    if (err < 0) {
        return err;
    }

    /* Read PAGE */
    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_PAGE_ID_ADDR, &reg);
    if (err < 0) {
        return err;
    }

    if ((reg & BNO055_PAGE_ID_MASK) != page) {
        LOG_ERR("I2C communication compromised [%d]!=[%d]!!", page, reg & BNO055_PAGE_ID_MASK);
        return -ECANCELED;
    }

    data->current_page = reg & BNO055_PAGE_ID_MASK;
    LOG_DBG("FUNC PAGE[%d]", page);
    return 0;
}

static int bno055_set_config(const struct device* dev, bno055_opmode_t mode, bool fusion) {
    const struct bno055_config* config = dev->config;
    struct bno055_data* data = dev->data;
    uint8_t reg;
    int err;

    LOG_DBG("FUNC MODE[%d]", mode);

    /* Switch to Page 0 */
    err = bno055_set_page(dev, BNO055_PAGE_ZERO);
    if (err < 0) {
        return err;
    }

    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, &reg);
    if (err < 0) {
        return err;
    }

    if (data->mode != (reg & BNO055_OPERATION_MODE_MASK)) {
        LOG_WRN("Update mode from I2C mode register [%d]<-[%d]!!", data->mode, reg & BNO055_OPERATION_MODE_MASK);
        data->mode = reg & BNO055_OPERATION_MODE_MASK;
    }

    if ((reg & BNO055_OPERATION_MODE_MASK) != OPERATION_MODE_CONFIG) {
        err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
        if (err < 0) {
            return err;
        }
        LOG_DBG("MODE[%d]", OPERATION_MODE_CONFIG);
        k_sleep(K_MSEC(BNO055_TIMING_SWITCH_FROM_ANY));
    }

    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, &reg);
    if (err < 0) {
        return err;
    }

    if ((reg & BNO055_OPERATION_MODE_MASK) != OPERATION_MODE_CONFIG) {
        LOG_ERR("I2C communication compromised [%d]!=[%d]!!", OPERATION_MODE_CONFIG, reg & BNO055_OPERATION_MODE_MASK);
        return -ECANCELED;
    }
    data->mode = reg & BNO055_OPERATION_MODE_MASK;

    if (mode == OPERATION_MODE_CONFIG) {
        LOG_DBG("I2C mode register already good!!");
        return 0;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, mode);
    if (err < 0) {
        return err;
    }
    k_sleep(K_MSEC(33 * BNO055_TIMING_SWITCH_FROM_CONFIG)); /* /!\ Datasheet not confrom WRONG DATASHEET */

    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, &reg);
    if (err < 0) {
        return err;
    }

    if ((reg & BNO055_PAGE_ID_MASK) != mode) {
        LOG_ERR("I2C communication compromised [%d]!=[%d]!!", mode, reg & BNO055_OPERATION_MODE_MASK);
        return -ECANCELED;
    }

    data->mode = reg & BNO055_OPERATION_MODE_MASK;
    LOG_DBG("FUNC MODE[%d]", mode);
    return 0;
}

static int bno055_set_power(const struct device* dev, bno055_powermode_t power) {
    const struct bno055_config* config = dev->config;
    struct bno055_data* data = dev->data;
    uint8_t reg;
    int err;

    LOG_DBG("FUNC POWER[%d]", power);

    bno055_opmode_t mode = data->mode;
    err = bno055_set_config(dev, OPERATION_MODE_CONFIG, false);
    if (err < 0) {
        return err;
    }

    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_PWR_MODE_ADDR, &reg);
    if (err < 0) {
        return err;
    }

    if (data->power != (reg & BNO055_POWER_MODE_MASK)) {
        LOG_WRN("Update power mode from I2C power register [%d]<-[%d]!!", data->power, reg & BNO055_POWER_MODE_MASK);
        data->power = reg & BNO055_POWER_MODE_MASK;
    }

    if ((reg & BNO055_POWER_MODE_MASK) == power) {
        LOG_DBG("I2C power register already good!!");
    } else {
        err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_PWR_MODE_ADDR, power);
        if (err < 0) {
            return err;
        }

        err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_PWR_MODE_ADDR, &power);
        if (err < 0) {
            return err;
        }

        if ((reg & BNO055_POWER_MODE_MASK) != mode) {
            LOG_ERR("I2C communication compromised [%d]!=[%d]!!", mode, reg & BNO055_POWER_MODE_MASK);
            return -ECANCELED;
        }
        data->power = reg & BNO055_POWER_MODE_MASK;
    }

    err = bno055_set_config(dev, mode, mode < OPERATION_MODE_IMUPLUS ? false : true);
    if (err < 0) {
        return err;
    }

    LOG_DBG("FUNC POWER[%d]", power);
    return 0;
}

static int get_vector(const struct device* dev, const uint8_t data_register, struct bno055_vector3_data* data) {
    uint8_t buffer[6] = {0};
    const struct bno055_config* config = dev->config;
    int16_t x, y, z;
    x = y = z = 0;

    /* Read vector data (6 bytes) */
    // get vecotor 3
    // LOG_DBG(">>>> Reading vector data from register[0x%02x]", data_register);
    int err = i2c_burst_read_dt(&config->i2c_bus, data_register, buffer, sizeof(buffer));

    x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    data->x = x;
    data->y = y;
    data->z = z;
    return 0;
}

// get vector 4
static int get_vector4(const struct device* dev, const uint8_t data_register, struct bno055_vector4_data* data) {
    const struct bno055_config* config = dev->config;
    uint8_t buffer[8] = {0};  // Quaternion data is 8 bytes (4 components, 2 bytes each)

    // Read 8 bytes starting at the quaternion data register
    int err = i2c_burst_read_dt(&config->i2c_bus, data_register, buffer, sizeof(buffer));
    if (err < 0) {
        return err;
    }

    // Convert the raw data into signed integers
    int16_t w = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    int16_t x = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    int16_t y = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
    int16_t z = ((int16_t)buffer[6]) | (((int16_t)buffer[7]) << 8);

    data->w = w;
    data->x = x;
    data->y = y;
    data->z = z;

    return 0;
}

static int get_system_status(const struct device* dev) {
    const struct bno055_config* config = dev->config;
    struct bno055_data* data = dev->data;
    uint8_t system_status;
    uint8_t self_test_result;
    uint8_t system_error;

    int err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_PAGE_ID_ADDR, 0x00);
    if (err < 0) {
        return err;
    }

    /* System Status (see section 4.3.58)
       0 = Idle
       1 = System Error
       2 = Initializing Peripherals
       3 = System Iniitalization
       4 = Executing Self-Test
       5 = Sensor fusio algorithm running
       6 = System running without fusion algorithms
     */

    if (system_status != 0) {
        err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_SYS_STAT_ADDR, &system_status);
        if (err < 0) {
            LOG_ERR("I2C communication failed %d", err);
            return err;
        }
    }

    /* Self Test Results
       1 = test passed, 0 = test failed

       Bit 0 = Accelerometer self test
       Bit 1 = Magnetometer self test
       Bit 2 = Gyroscope self test
       Bit 3 = MCU self test

       0x0F = all good!
     */
    if (self_test_result != 0) {
        err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_SELFTEST_RESULT_ADDR, &self_test_result);
        if (err < 0) {
            LOG_ERR("I2C communication failed %d", err);
            return err;
        }
    }

    /* System Error (see section 4.3.59)
       0 = No error
       1 = Peripheral initialization error
       2 = System initialization error
       3 = Self test result failed
       4 = Register map value out of range
       5 = Register map address out of range
       6 = Register map write error
       7 = BNO low power mode not available for selected operat ion mode
       8 = Accelerometer power mode not available
       9 = Fusion algorithm configuration error
       A = Sensor configuration error
     */

    if (system_error != 0) {
        err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_SYS_ERR_ADDR, &system_error);
        if (err < 0) {
            LOG_ERR("I2C communication failed %d", err);
            return err;
        }
    }

    // bno055_write_register(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    // sleep_ms(100);
    data->sys_status.system_status = system_status;
    data->sys_status.self_test_result = self_test_result;
    data->sys_status.system_error = system_error;

    return 0;
}

static int check_firmware_version(const struct device* dev) {
    const struct bno055_config* config = dev->config;
    uint8_t sw_major = 0;
    int err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_SW_REV_ID_LSB_ADDR, &sw_major);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    uint8_t sw_minor = 0;
    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_SW_REV_ID_MSB_ADDR, &sw_minor);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    printk("Firmware Version: %u.%u\n", sw_major, sw_minor);

    return 0;
}

static int get_calibration(const struct device* dev, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    uint8_t calData = 0;
    const struct bno055_config* config = dev->config;
    int err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_CALIB_STAT_ADDR, &calData);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
    return 0;
}

static bool is_fully_calibrated(const struct device* dev) {
    uint8_t system = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
    get_calibration(dev, &system, &gyro, &accel, &mag);
    LOG_DBG("system: %x gyro %x accel %x mag %x\n", system, gyro, accel, mag);
    int mMode = OPERATION_MODE_NDOF;
    switch (mMode) {
        case OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OPERATION_MODE_GYRONLY:
        case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
            return (gyro == 3);
        case OPERATION_MODE_ACCMAG:
        case OPERATION_MODE_COMPASS:
            return (accel == 3 && mag == 3);
        case OPERATION_MODE_ACCGYRO:
        case OPERATION_MODE_IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}

static int set_ext_crystal_use(const struct device* dev, bool usextal) {
    /* Switch to config mode (just in case since this is the default) */
    // bno055_write_register(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    const struct bno055_config* config = dev->config;
    int err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    k_sleep(K_MSEC(25));
    // bno055_write_register(BNO055_PAGE_ID_ADDR, 0);
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_PAGE_ID_ADDR, 0x00);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    if (usextal) {
        // bno055_write_register(BNO055_SYS_TRIGGER_ADDR, 0x80);
        err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_SYS_TRIGGER_ADDR, 0x80);
        if (err < 0) {
            LOG_ERR("I2C communication failed %d", err);
            return err;
        }
    } else {
        err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_SYS_TRIGGER_ADDR, 0x00);
        if (err < 0) {
            LOG_ERR("I2C communication failed %d", err);
            return err;
        }
    }
    k_sleep(K_MSEC(10));
    /* Set the requested operating mode (see section 3.3) */
    // bno055_write_register(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    k_sleep(K_MSEC(20));
}

static int get_calibration_data(const struct device* dev) {
    // bno055_write_register(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    const struct bno055_config* config = dev->config;
    struct bno055_calib_data calibration_data;
    int err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    k_sleep(K_MSEC(25));

    err = i2c_reg_read_bytes_dt(&config->i2c_bus, 0x55, calibration_data, CALIBRATION_DATA_SIZE);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    if (err < 0) {
        LOG_ERR("I2C communication failed %d", err);
        return err;
    }
    k_sleep(K_MSEC(20));
    return 0;
}

static bool is_valid_calibration_data(const struct device* dev, const uint8_t* cal, size_t len) {
    if (len != 22) {
        printk("Calibration data length incorrect. Expected 22, got %zu\n", len);
        return false;
    }
    get_calibration_data(dev);
    int16_t acc_offset_x = (int16_t)(cal[0] | (cal[1] << 8));
    int16_t acc_offset_y = (int16_t)(cal[2] | (cal[3] << 8));
    int16_t acc_offset_z = (int16_t)(cal[4] | (cal[5] << 8));

    int16_t mag_offset_x = (int16_t)(cal[6] | (cal[7] << 8));
    int16_t mag_offset_y = (int16_t)(cal[8] | (cal[9] << 8));
    int16_t mag_offset_z = (int16_t)(cal[10] | (cal[11] << 8));

    int16_t gyro_offset_x = (int16_t)(cal[12] | (cal[13] << 8));
    int16_t gyro_offset_y = (int16_t)(cal[14] | (cal[15] << 8));
    int16_t gyro_offset_z = (int16_t)(cal[16] | (cal[17] << 8));

    uint16_t acc_radius = (uint16_t)(cal[18] | (cal[19] << 8));
    uint16_t mag_radius = (uint16_t)(cal[20] | (cal[21] << 8));

    LOG_DBG("Accel offset: X=%d Y=%d Z=%d\n", acc_offset_x, acc_offset_y, acc_offset_z);
    LOG_DBG("Mag offset  : X=%d Y=%d Z=%d\n", mag_offset_x, mag_offset_y, mag_offset_z);
    LOG_DBG("Gyro offset : X=%d Y=%d Z=%d\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);
    LOG_DBG("Radius      : Acc=%u Mag=%u\n", acc_radius, mag_radius);

    // Basic sanity check: large offsets might indicate bad data
    if (abs(acc_offset_x) > 2000 || abs(acc_offset_y) > 2000 || abs(acc_offset_z) > 2000) {
        LOG_ERR("Accel offsets too large!\n");
        return false;
    }

    if (abs(mag_offset_x) > 2000 || abs(mag_offset_y) > 2000 || abs(mag_offset_z) > 2000) {
        LOG_ERR("Mag offsets too large!\n");
        return false;
    }

    if (abs(gyro_offset_x) > 500 || abs(gyro_offset_y) > 500 || abs(gyro_offset_z) > 500) {
        LOG_ERR("Gyro offsets too large!\n");
        return false;
    }

    return true;
}

static int bno055_channel_get(const struct device* dev, enum sensor_channel chan, struct sensor_value* val) {
    struct bno055_data* data = dev->data;

    // LOG_DBG("Getting channel data for channel %d\n", chan);

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_YRP) {
        // raw / 16 = degrees
        // radians = degrees × (π / 180)
        // radians = raw × (π / 2880)
        double convert_to_radians = M_PI / 2880.0f;  // 1 radian = 2880 LSB
        double rad_x = data->eul.x * convert_to_radians;
        val[0].val1 = (int32_t)rad_x;
        val[0].val2 = (int32_t)((rad_x - val[0].val1) * 1000000.0);

        double rad_y = data->eul.y * convert_to_radians;
        val[1].val1 = (int32_t)rad_y;
        val[1].val2 = (int32_t)((rad_y - val[1].val1) * 1000000.0);

        double rad_z = data->eul.z * convert_to_radians;
        val[2].val1 = (int32_t)rad_z;
        val[2].val2 = (int32_t)((rad_z - val[2].val1) * 1000000.0);

        return 0;
        return 0;
    }

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_SYSTEM_STATUS) {
        (val)->val1 = data->sys_status.system_status;
        (val)->val2 = 0;
        (val + 1)->val1 = data->sys_status.self_test_result;
        (val + 1)->val2 = 0;
        (val + 2)->val1 = data->sys_status.system_error;
        (val + 2)->val2 = 0;
        return 0;
    }

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_WXYZ) {
        (val)->val1 = data->qua.w / BNO055_QUATERNION_RESOLUTION;
        (val)->val2 =
            (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.w - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
        (val + 1)->val1 = data->qua.x / BNO055_QUATERNION_RESOLUTION;
        (val + 1)->val2 =
            (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.x - (val + 1)->val1 * BNO055_QUATERNION_RESOLUTION);
        (val + 2)->val1 = data->qua.y / BNO055_QUATERNION_RESOLUTION;
        (val + 2)->val2 =
            (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.y - (val + 2)->val1 * BNO055_QUATERNION_RESOLUTION);
        (val + 3)->val1 = data->qua.z / BNO055_QUATERNION_RESOLUTION;
        (val + 3)->val2 =
            (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.z - (val + 3)->val1 * BNO055_QUATERNION_RESOLUTION);
        return 0;
    }

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_SYS) {
        (val)->val1 = data->calib.sys;
        (val)->val2 = 0;
        return 0;
    }

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_GYR) {
        (val)->val1 = data->calib.gyr;
        (val)->val2 = 0;
        return 0;
    }

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_ACC) {
        (val)->val1 = data->calib.acc;
        (val)->val2 = 0;
        return 0;
    }

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_MAG) {
        (val)->val1 = data->calib.mag;
        (val)->val2 = 0;
        return 0;
    }

    if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_SGAM) {
        (val)->val1 = data->calib.sys;
        (val)->val2 = 0;
        (val + 1)->val1 = data->calib.gyr;
        (val + 1)->val2 = 0;
        (val + 2)->val1 = data->calib.acc;
        (val + 2)->val2 = 0;
        (val + 3)->val1 = data->calib.mag;
        (val + 3)->val2 = 0;
        return 0;
    }

    return -ENOTSUP;
}

static int bno055_attr_set(const struct device* dev, enum sensor_channel chan, enum sensor_attribute attr,
                           const struct sensor_value* val) {
    int err;

    switch (chan) {
        case SENSOR_CHAN_ALL:
            if (attr == SENSOR_ATTR_CONFIGURATION) {
                LOG_INF("SET MODE[%d]", val->val1);
                switch (val->val1) {
                    case OPERATION_MODE_CONFIG:
                        LOG_DBG("MODE OPERATION_MODE_CONFIG");
                        err = bno055_set_config(dev, OPERATION_MODE_CONFIG, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_ACCONLY:
                        LOG_DBG("MODE OPERATION_MODE_ACCONLY");
                        err = bno055_set_config(dev, OPERATION_MODE_ACCONLY, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_MAGONLY:
                        LOG_DBG("MODE OPERATION_MODE_MAGONLY");
                        err = bno055_set_config(dev, OPERATION_MODE_MAGONLY, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_GYRONLY:
                        LOG_DBG("MODE OPERATION_MODE_GYRONLY");
                        err = bno055_set_config(dev, OPERATION_MODE_GYRONLY, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_ACCMAG:
                        LOG_DBG("MODE OPERATION_MODE_ACCMAG");
                        err = bno055_set_config(dev, OPERATION_MODE_ACCMAG, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_ACCGYRO:
                        LOG_DBG("MODE OPERATION_MODE_ACCGYRO");
                        err = bno055_set_config(dev, OPERATION_MODE_ACCGYRO, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_MAGGYRO:
                        LOG_DBG("MODE OPERATION_MODE_MAGGYRO");
                        err = bno055_set_config(dev, OPERATION_MODE_MAGGYRO, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_AMG:
                        LOG_DBG("MODE OPERATION_MODE_AMG");
                        err = bno055_set_config(dev, OPERATION_MODE_AMG, false);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_IMUPLUS:
                        LOG_DBG("MODE OPERATION_MODE_IMUPLUS");
                        err = bno055_set_config(dev, OPERATION_MODE_IMUPLUS, true);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_COMPASS:
                        LOG_DBG("MODE OPERATION_MODE_COMPASS");
                        err = bno055_set_config(dev, OPERATION_MODE_COMPASS, true);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_M4G:
                        LOG_DBG("MODE OPERATION_MODE_M4G");
                        err = bno055_set_config(dev, OPERATION_MODE_M4G, true);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_NDOF_FMC_OFF:
                        LOG_DBG("MODE OPERATION_MODE_NDOF_FMC_OFF");
                        err = bno055_set_config(dev, OPERATION_MODE_NDOF_FMC_OFF, true);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case OPERATION_MODE_NDOF:
                        LOG_DBG("MODE OPERATION_MODE_NDOF");
                        err = bno055_set_config(dev, OPERATION_MODE_NDOF, true);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    default:
                        return -EINVAL;
                }
            } else if (attr == (enum sensor_attribute)BNO055_SENSOR_ATTR_POWER_MODE) {
                LOG_INF("SET POWER[%d]", val->val1);
                switch (val->val1) {
                    case BNO055_POWER_NORMAL:
                        LOG_DBG("POWER BNO055_POWER_NORMAL");
                        err = bno055_set_power(dev, BNO055_POWER_NORMAL);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case BNO055_POWER_LOW_POWER:
                        LOG_DBG("POWER BNO055_POWER_LOW_POWER");
                        err = bno055_set_power(dev, BNO055_POWER_LOW_POWER);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case BNO055_POWER_SUSPEND:
                        LOG_DBG("POWER BNO055_POWER_SUSPEND");
                        err = bno055_set_power(dev, BNO055_POWER_SUSPEND);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    case BNO055_POWER_INVALID:
                        LOG_DBG("POWER BNO055_POWER_INVALID");
                        err = bno055_set_power(dev, BNO055_POWER_INVALID);
                        if (err < 0) {
                            return err;
                        }
                        break;

                    default:
                        return -EINVAL;
                }
            }
            break;

        default:
            return -ENOTSUP;
    }
    return 0;
}

static int bno055_sample_fetch(const struct device* dev, enum sensor_channel chan) {
    struct bno055_data* data = dev->data;
    int err = 0;
    switch (data->mode) {
        case OPERATION_MODE_CONFIG:
            break;
        case OPERATION_MODE_NDOF:
            err = get_system_status(dev);
            err = get_vector(dev, VECTOR_EULER, &data->eul);
            err = get_vector4(dev, VECTOR_QUAT, &data->qua);
            is_fully_calibrated(dev);
            if (err < 0) {
                return err;
            }
            break;

        default:
            LOG_WRN("BNO055 Not in Computation Mode!!");
            return -ENOTSUP;
    }
    return 0;
}

static int bno055_init(const struct device* dev) {
    // Initialize I2C bus
    const struct bno055_config* config = dev->config;
    struct bno055_data* data = dev->data;
    int err = 0;

    if (!i2c_is_ready_dt(&config->i2c_bus)) {
        LOG_ERR("I2C bus not ready!!");
        return -ENODEV;
    }

    LOG_DBG("Initializing BNO055...\n");

    // Setting Page to 0
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_PAGE_ID_ADDR, 0x00);

    // reset the BNO055
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_SYS_TRIGGER_ADDR, 0x20);
    k_sleep(K_MSEC(650));  // Wait for the reset to complete

    // Check if the BNO055 is connected
    uint8_t chip_id;
    err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_CHIP_ID_ADDR, &chip_id);

    if (chip_id != BNO055_ID) {
        LOG_DBG("BNO055 not detected! Chip ID: 0x%02X\n", chip_id);
        return -ENODEV;  // Initialization failed
    }
    LOG_DBG("BNO055 detected! Chip ID: 0x%02X\n", chip_id);

    uint8_t soft[2];
    err = i2c_burst_read_dt(&config->i2c_bus, 0x04, soft, sizeof(soft));
    LOG_INF("SOFTWARE REV [%d][%d]", soft[1], soft[0]);

    // Set the operating mode to CONFIG_MODE for initial setup
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    k_sleep(K_MSEC(30));

    // Perform any necessary sensor configuration (e.g., power mode, unit
    // selection) Example: Set the power mode to normal
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);

    k_sleep(K_MSEC(30));

    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_PAGE_ID_ADDR, 0x00);
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_SYS_TRIGGER_ADDR, 0x00);
    k_sleep(K_MSEC(10));
    // Set the sensor to NDOF mode
    err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    data->mode = OPERATION_MODE_NDOF;
    k_sleep(K_MSEC(30));
    set_ext_crystal_use(dev, config->use_xtal);

    printk("BNO055 initialization complete.\n");
    return 0;  // Initialization successful
}

static const struct sensor_driver_api bno055_driver_api = {
    .attr_set = bno055_attr_set,
    .sample_fetch = bno055_sample_fetch,
    .channel_get = bno055_channel_get,
#if BNO055_USE_IRQ
    .trigger_set = bno055_trigger_set,
#endif
};

#define BNO055_INIT(n)                                                                             \
    static struct bno055_config bno055_config_##n = {                                              \
        .i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                        \
        .use_xtal = DT_INST_PROP(n, use_xtal),                                                     \
    };                                                                                             \
    static struct bno055_data bno055_data_##n;                                                     \
    DEVICE_DT_INST_DEFINE(n, bno055_init, NULL, &bno055_data_##n, &bno055_config_##n, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &bno055_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BNO055_INIT)
