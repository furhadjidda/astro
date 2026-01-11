#ifndef ZEPHYR_DRIVERS_GNSS_MTK3333_H_
#define ZEPHYR_DRIVERS_GNSS_MTK3333_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device configuration (from Devicetree)
 */
struct mtk3333_config {
    const struct device* i2c;
    uint16_t i2c_addr;
};

/**
 * Runtime driver data
 */
struct mtk3333_data {
    uint8_t dummy;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_GNSS_MTK3333_H_ */
