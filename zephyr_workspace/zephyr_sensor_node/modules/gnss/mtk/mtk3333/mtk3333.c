#define DT_DRV_COMPAT mtk_mtk3333

#include "mtk3333.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mtk3333, CONFIG_LOG_DEFAULT_LEVEL);

/* ----------------------------- Driver Data ----------------------------- */

struct mtk3333_config {
    const struct device* i2c;
    uint16_t i2c_addr;
};

struct mtk3333_data {
    struct k_work_delayable poll_work;
    const struct device* dev;

    /* Simulated GNSS data */
    struct gnss_data data;
};

/* -------------------------- Poll Work Handler -------------------------- */

static void mtk3333_poll_work(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct mtk3333_data* data = CONTAINER_OF(dwork, struct mtk3333_data, poll_work);

    /* --- Simulate a GNSS fix --- */
    data->data.info.fix_status = GNSS_FIX_STATUS_GNSS_FIX;
    data->data.info.fix_quality = GNSS_FIX_QUALITY_GNSS_SPS;
    data->data.info.satellites_cnt = 8;
    data->data.info.hdop = 100;

    data->data.nav_data.latitude = 374221234;     // 37.4221234 deg * 1e7
    data->data.nav_data.longitude = -1220845678;  // -122.0845678 deg * 1e7
    data->data.nav_data.altitude = 15000;         // 15 m
    data->data.nav_data.speed = 0;
    data->data.nav_data.bearing = 0;

    /* Publish the GNSS data */
    gnss_publish_data(data->dev, &data->data);

    LOG_INF("MTK3333 simulated fix published");

    /* Reschedule the poll work in 200 ms */
    k_work_schedule(&data->poll_work, K_MSEC(200));
}

/* --------------------------- GNSS API Stubs ---------------------------- */

static int mtk3333_set_fix_rate(const struct device* dev, uint32_t fix_ms) {
    ARG_UNUSED(dev);
    ARG_UNUSED(fix_ms);
    return -ENOTSUP;
}

static int mtk3333_get_fix_rate(const struct device* dev, uint32_t* fix_ms) {
    if (fix_ms) {
        *fix_ms = 0;
    }
    return -ENOTSUP;
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
    return -ENOTSUP;
}

static int mtk3333_get_enabled_systems(const struct device* dev, gnss_systems_t* systems) {
    if (systems) {
        *systems = 0;
    }
    return -ENOTSUP;
}

static gnss_systems_t mtk3333_get_supported_systems(const struct device* dev) {
    ARG_UNUSED(dev);
    return GNSS_SYSTEM_GPS | GNSS_SYSTEM_SBAS;
}

/* ----------------------------- GNSS API Table -------------------------- */

static DEVICE_API(gnss, mtk3333_gnss_api) = {
    .set_fix_rate = mtk3333_set_fix_rate,
    .get_fix_rate = mtk3333_get_fix_rate,
    .set_navigation_mode = mtk3333_get_navigation_mode,
    .get_navigation_mode = mtk3333_get_navigation_mode,
    .set_enabled_systems = mtk3333_set_enabled_systems,
    .get_enabled_systems = mtk3333_get_enabled_systems,
    .get_supported_systems = mtk3333_get_supported_systems,
};

/* --------------------------- Driver Initialization --------------------- */

static int mtk3333_init(const struct device* dev) {
    const struct mtk3333_config* cfg = dev->config;
    struct mtk3333_data* data = dev->data;
    printk("Initializing MTK3333 simulated GNSS driver...\n");
    if (!device_is_ready(cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        // return -ENODEV;
    }

    /* Initialize delayable work */
    k_work_init_delayable(&data->poll_work, mtk3333_poll_work);

    /* Schedule first poll after 200 ms */
    k_work_schedule(&data->poll_work, K_MSEC(200));

    LOG_INF("MTK3333 simulated GNSS driver initialized");

    return 0;
}

/* ------------------------ Device Instantiation ------------------------- */

#define MTK3333_DEFINE(inst)                                                                                   \
    static struct mtk3333_data mtk3333_data_##inst;                                                            \
    static const struct mtk3333_config mtk3333_config_##inst = {                                               \
        .i2c = DEVICE_DT_GET(DT_INST_BUS(inst)),                                                               \
        .i2c_addr = DT_INST_REG_ADDR(inst),                                                                    \
    };                                                                                                         \
    DEVICE_DT_INST_DEFINE(inst, mtk3333_init, NULL, &mtk3333_data_##inst, &mtk3333_config_##inst, POST_KERNEL, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mtk3333_gnss_api);

DT_INST_FOREACH_STATUS_OKAY(MTK3333_DEFINE)
