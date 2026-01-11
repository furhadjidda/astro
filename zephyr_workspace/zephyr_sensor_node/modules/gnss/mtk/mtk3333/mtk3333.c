#define DT_DRV_COMPAT adafruit_mtk3333

#include "mtk3333.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mtk3333, CONFIG_LOG_DEFAULT_LEVEL);

/* -------------------------------------------------------------------------- */
/* GNSS API stubs                                                              */
/* -------------------------------------------------------------------------- */

static int mtk3333_set_fix_rate(const struct device* dev, uint32_t fix_ms) {
    ARG_UNUSED(dev);
    ARG_UNUSED(fix_ms);
    return -ENOTSUP;
}

static int mtk3333_get_fix_rate(const struct device* dev, uint32_t* fix_ms) {
    ARG_UNUSED(dev);

    if (fix_ms != NULL) {
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
    ARG_UNUSED(dev);

    if (mode != NULL) {
        *mode = GNSS_NAVIGATION_MODE_UNKNOWN;
    }

    return -ENOTSUP;
}

static int mtk3333_set_enabled_systems(const struct device* dev, gnss_systems_t systems) {
    ARG_UNUSED(dev);
    ARG_UNUSED(systems);
    return -ENOTSUP;
}

static int mtk3333_get_enabled_systems(const struct device* dev, gnss_systems_t* systems) {
    ARG_UNUSED(dev);

    if (systems != NULL) {
        *systems = 0;
    }

    return -ENOTSUP;
}

static gnss_systems_t mtk3333_get_supported_systems(const struct device* dev) {
    ARG_UNUSED(dev);

    /* MTK3333 supports GPS and SBAS (WAAS/EGNOS) */
    return GNSS_SYSTEM_GPS | GNSS_SYSTEM_SBAS;
}

/* -------------------------------------------------------------------------- */
/* GNSS API table                                                              */
/* -------------------------------------------------------------------------- */

static DEVICE_API(gnss, mtk3333_gnss_api) = {
    .set_fix_rate = mtk3333_set_fix_rate,
    .get_fix_rate = mtk3333_get_fix_rate,
    .set_navigation_mode = mtk3333_set_navigation_mode,
    .get_navigation_mode = mtk3333_get_navigation_mode,
    .set_enabled_systems = mtk3333_set_enabled_systems,
    .get_enabled_systems = mtk3333_get_enabled_systems,
    .get_supported_systems = mtk3333_get_supported_systems,
};

/* -------------------------------------------------------------------------- */
/* Device initialization                                                       */
/* -------------------------------------------------------------------------- */

static int mtk3333_init(const struct device* dev) {
    const struct mtk3333_config* cfg = dev->config;

    if (!device_is_ready(cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    LOG_INF("MTK3333 GNSS initialized (I2C addr 0x%02x)", cfg->i2c_addr);

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Device instantiation                                                        */
/* -------------------------------------------------------------------------- */

#define MTK3333_DEFINE(inst)                                                                                   \
    static struct mtk3333_data mtk3333_data_##inst;                                                            \
                                                                                                               \
    static const struct mtk3333_config mtk3333_config_##inst = {                                               \
        .i2c = DEVICE_DT_GET(DT_INST_BUS(inst)),                                                               \
        .i2c_addr = DT_INST_REG_ADDR(inst),                                                                    \
    };                                                                                                         \
                                                                                                               \
    DEVICE_DT_INST_DEFINE(inst, mtk3333_init, NULL, &mtk3333_data_##inst, &mtk3333_config_##inst, POST_KERNEL, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mtk3333_gnss_api);

DT_INST_FOREACH_STATUS_OKAY(MTK3333_DEFINE)
