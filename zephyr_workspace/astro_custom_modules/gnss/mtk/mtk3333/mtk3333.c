#define DT_DRV_COMPAT mtk_mtk3333

#include "mtk3333.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mtk3333_pmtk.h"

LOG_MODULE_REGISTER(mtk3333, CONFIG_LOG_DEFAULT_LEVEL);

/* ----------------------------- Driver Data ----------------------------- */

struct mtk3333_config {
    struct i2c_dt_spec i2c_bus;
    uint16_t i2c_addr;
};

static int mtk3333_send_command(const struct device* dev, const char* cmd, size_t len) {
    const struct mtk3333_config* cfg = dev->config;

    int ret = i2c_write_dt(&cfg->i2c_bus, cmd, len);
    if (ret < 0) {
        LOG_ERR("Failed to send command to MTK3333: %d", ret);
        return ret;
    }

    return 0;
}

static void mtk3333_process_char(struct mtk3333_data* data, char c, struct gnss_data* gnss_out, bool* publish) {
    if (c == '\0' || (c == 0x0A && data->last_char != 0x0D)) {
        return;
    }
    data->last_char = c;

    if (data->line_index < MAXLINELENGTH - 1) {
        data->current_line[data->line_index++] = c;
    }

    if (c != '\n') {
        return;
    }

    /* ── Complete sentence ── */
    data->current_line[data->line_index] = '\0';
    data->line_index = 0;

    if (data->current_line == data->buffer_line1) {
        data->last_line = data->buffer_line1;
        data->current_line = data->buffer_line2;
    } else {
        data->last_line = data->buffer_line2;
        data->current_line = data->buffer_line1;
    }

    memset(gnss_out, 0, sizeof(struct gnss_data));
    gnss_nmea_parse(data->last_line, gnss_out);
    // LOG_INF("Received NMEA sentence: %s", data->last_line);
    if (strncmp(data->last_line, "$GNGGA", 6) == 0) {
        *publish = true;
    }
}

/* -------------------------- Poll Work Handler -------------------------- */

static void mtk3333_poll_work(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct mtk3333_data* data = CONTAINER_OF(dwork, struct mtk3333_data, poll_work);
    const struct mtk3333_config* cfg = data->dev->config;

    uint8_t raw[GPS_MAX_I2C_TRANSFER];
    int ret = i2c_read_dt(&cfg->i2c_bus, raw, GPS_MAX_I2C_TRANSFER);
    if (ret < 0) {
        LOG_ERR("I2C read failed: %d", ret);
        goto reschedule;
    }

    for (int i = 0; i < GPS_MAX_I2C_TRANSFER; i++) {
        bool publish = false;
        struct gnss_data gnss_out = {0};

        mtk3333_process_char(data, (char)raw[i], &gnss_out, &publish);

        if (publish) {
            LOG_INF("Satellites: %d HDOP: %d Fix: %d", gnss_out.info.satellites_cnt, gnss_out.info.hdop,
                    gnss_out.info.fix_status);
            gnss_publish_data(data->dev, &gnss_out);
        }
    }

reschedule:
    k_work_schedule(&data->poll_work, K_MSEC(5));
}

/* --------------------------- GNSS API Stubs ---------------------------- */

static int mtk3333_set_fix_rate(const struct device* dev, uint32_t fix_ms) {
    struct mtk3333_data* data = dev->data;
    data->fix_rate_ms = fix_ms;
    return fix_ms;
}

static int mtk3333_get_fix_rate(const struct device* dev, uint32_t* fix_ms) {
    struct mtk3333_data* data = dev->data;
    if (fix_ms) {
        *fix_ms = data->fix_rate_ms;
    } else {
        return -ENOTSUP;
    }
    return 1;
}

static int mtk3333_set_navigation_mode(const struct device* dev, enum gnss_navigation_mode mode) {
    ARG_UNUSED(dev);
    ARG_UNUSED(mode);
    return -ENOTSUP;
}

static int mtk3333_get_navigation_mode(const struct device* dev, enum gnss_navigation_mode* mode) {
    if (mode) {
        *mode = GNSS_NAVIGATION_MODE_ZERO_DYNAMICS;
    }
    return -ENOTSUP;
}

static int mtk3333_set_enabled_systems(const struct device* dev, gnss_systems_t systems) {
    ARG_UNUSED(dev);
    ARG_UNUSED(systems);
    mtk3333_send_command(dev, (const uint8_t*)PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
    return GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU;
}

static int mtk3333_get_enabled_systems(const struct device* dev, gnss_systems_t* systems) {
    if (systems) {
        *systems = 0;
    }
    return GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU;
}

static gnss_systems_t mtk3333_get_supported_systems(const struct device* dev) {
    ARG_UNUSED(dev);
    return GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU;
}

/* ----------------------------- GNSS API Table -------------------------- */

static DEVICE_API(gnss, mtk3333_gnss_api) = {
    .set_fix_rate = mtk3333_set_fix_rate,
    .get_fix_rate = mtk3333_get_fix_rate,
    .set_navigation_mode = mtk3333_set_navigation_mode,
    .get_navigation_mode = mtk3333_get_navigation_mode,
    .set_enabled_systems = mtk3333_set_enabled_systems,
    .get_enabled_systems = mtk3333_get_enabled_systems,
    .get_supported_systems = mtk3333_get_supported_systems,
};

/* --------------------------- Driver Initialization --------------------- */

static int mtk3333_init(const struct device* dev) {
    const struct mtk3333_config* cfg = dev->config;
    struct mtk3333_data* data = dev->data;
    LOG_INF("Initializing MTK3333 simulated GNSS driver...\n");
    if (!i2c_is_ready_dt(&cfg->i2c_bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }
    k_msleep(500);
    data->read_buffer_index = 0;

    data->buff_max = -1;
    data->last_char = 0;
    data->line_index = 0;       ///< our index into filling the current line
    data->current_line = NULL;  ///< Pointer to current line buffer
    data->last_line = NULL;     ///< Pointer to previous line buffer
    data->rcvd_flag = false;    ///< Received flag

    data->last_update = 2000000000L;  ///< millis() when last full sentence successfully parsed
    data->last_fix = 2000000000L;     ///< millis() when last mFix received
    data->last_time = 2000000000L;    ///< millis() when last time received
    data->last_date = 2000000000L;    ///< millis() when last date received
    data->rcvd_time = 2000000000L;    ///< millis() when last full sentence received
    data->sent_time = 2000000000L;    ///< millis() when first character of last
                                      ///< full sentence received
    data->dev = dev;
    data->current_line = data->buffer_line1;
    data->last_line = data->buffer_line2;
    mtk3333_send_command(dev, (const uint8_t*)PMTK_SET_NMEA_OUTPUT_RMCGGA, strlen(PMTK_SET_NMEA_OUTPUT_RMCGGA));
    // uncomment this line to turn on only the "minimum recommended" data
    // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or
    // RMC+GGA since the parser doesn't care about other sentences at this time
    // Set the update rate
    mtk3333_send_command(dev, (const uint8_t*)PMTK_SET_NMEA_UPDATE_1HZ,
                         strlen(PMTK_SET_NMEA_UPDATE_1HZ));  // 1 Hz update rate
    mtk3333_set_fix_rate(dev, 1000);
    // For the parsing code to work nicely and have time to sort thru the data,
    // and print it out we don't suggest using anything higher than 1 Hz

    // Request updates on mAntenna status, comment out to keep quiet
    mtk3333_send_command(dev, (const uint8_t*)PGCMD_ANTENNA, strlen(PGCMD_ANTENNA));

    k_msleep(500);

    /* Initialize delayable work */
    k_work_init_delayable(&data->poll_work, mtk3333_poll_work);

    /* Schedule first poll after 200 ms */
    k_work_schedule(&data->poll_work, K_MSEC(5));

    LOG_INF("MTK3333 simulated GNSS driver initialized");

    return 0;
}

/* ------------------------ Device Instantiation ------------------------- */

#define MTK3333_DEFINE(inst)                                                                                   \
    static struct mtk3333_data mtk3333_data_##inst;                                                            \
    static const struct mtk3333_config mtk3333_config_##inst = {                                               \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                                                 \
        .i2c_addr = DT_INST_REG_ADDR(inst),                                                                    \
    };                                                                                                         \
    DEVICE_DT_INST_DEFINE(inst, mtk3333_init, NULL, &mtk3333_data_##inst, &mtk3333_config_##inst, POST_KERNEL, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mtk3333_gnss_api);

DT_INST_FOREACH_STATUS_OKAY(MTK3333_DEFINE)
