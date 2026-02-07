/*
 *   This file is part of astro.
 *
 *   astro is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <iim42652.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_module, LOG_LEVEL_DBG);
static const struct device* const iim42652_dev = DEVICE_DT_GET(DT_NODELABEL(iim42652));

#define IIM42652_TIMING_STARTUP 400  // 400ms
int main(void) {
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(iim42652)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("Deferred init enabled, sleeping for %d ms", IIM42652_TIMING_STARTUP);
    k_sleep(K_MSEC(IIM42652_TIMING_STARTUP));
    device_init(iim42652_dev);
#endif
    if (!device_is_ready(iim42652_dev)) {
        LOG_ERR("Device %s is not ready\n", iim42652_dev->name);
        return -ENODEV;
    }
    LOG_DBG("IIM42652 Device %s is ready\n", iim42652_dev->name);

    while (1) {
        if (NULL != iim42652_dev) {
            sensor_sample_fetch(iim42652_dev);
            struct sensor_value acc[3];
            sensor_channel_get(iim42652_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
            double ax = acc[0].val1 + acc[0].val2 / 1000000.0;
            double ay = acc[1].val1 + acc[1].val2 / 1000000.0;
            double az = acc[2].val1 + acc[2].val2 / 1000000.0;
            LOG_DBG("Accel X: %.3f g, Y: %.3f g, Z: %.3f g", ax, ay, az);
        } else {
            LOG_ERR("IIM42652 device is NULL!\n");
        }
        k_sleep(K_MSEC(500));
    }
}
