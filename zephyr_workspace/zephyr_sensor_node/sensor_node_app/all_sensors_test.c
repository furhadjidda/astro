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

#include <bno055.h>  // Required for custom SENSOR_CHAN_*
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(all_sensors_module, LOG_LEVEL_DBG);

/* =========================================================
 * Thread Configuration
 * ========================================================= */

#define THREAD_STACK_SIZE 4096
#define IMU_THREAD_PRIORITY 5

/* =========================================================
 * Thread objects
 * ========================================================= */

K_THREAD_STACK_DEFINE(imu_stack, THREAD_STACK_SIZE);
static struct k_thread imu_thread;

// Display parameters
#define MAX_FONTS 42
#define SELECTED_FONT_INDEX 0
static uint16_t rows = 0;
static uint8_t ppt = 0;
static uint8_t font_width = 0;
static uint8_t font_height = 0;

// display driver
static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
// imu driver
static const struct device* const bno055_dev = DEVICE_DT_GET(DT_NODELABEL(bno055));
// gnss driver
#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

// IMU configuration
static bool bno055_fusion = true;
#define BNO055_TIMING_STARTUP 400

static void imu_thread_entry(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(bno055)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("Deferred init enabled, sleeping for %d ms", BNO055_TIMING_STARTUP);
    k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
    device_init(bno055_dev);
#endif
    if (!device_is_ready(bno055_dev)) {
        LOG_ERR("Device %s is not ready\n", bno055_dev->name);
        return -ENODEV;
    }
    LOG_DBG("BNO055 Device %s is ready\n", bno055_dev->name);

    while (1) {
        if (NULL != bno055_dev) {
            char buffer[64];
            struct sensor_value eul[3];

            sensor_sample_fetch(bno055_dev);
            sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_EULER_YRP, eul);
            LOG_DBG(
                "EULER: X(rad.s-1)[%d.%06d] Y(rad.s-1)[%d.%06d] "
                "Z(rad.s-1)[%d.%06d]\n",
                eul[0].val1, eul[0].val2, eul[1].val1, eul[1].val2, eul[2].val1, eul[2].val2);
            // Format for the screen

            snprintf(buffer, sizeof(buffer), "X=%d.%02d", eul[0].val1, eul[0].val2);
            cfb_print(display_dev, buffer, 0, 20);

            snprintf(buffer, sizeof(buffer), "Y=%d.%02d", eul[1].val1, eul[1].val2);
            cfb_print(display_dev, buffer, 0, 35);

            snprintf(buffer, sizeof(buffer), "Z=%d.%02d", eul[2].val1, eul[2].val2);
            cfb_print(display_dev, buffer, 0, 50);

            cfb_framebuffer_finalize(display_dev);

            struct sensor_value sys_status[3];
            sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_SYSTEM_STATUS, sys_status);
            LOG_DBG("system status %d , self_test = %d system_error %d \n", sys_status[0].val1, sys_status[1].val1,
                    sys_status[2].val1);

        } else {
            LOG_ERR("BNO055 device is NULL!\n");
        }
        k_sleep(K_MSEC(200));
    }
}

static void gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
    uint64_t timepulse_ns;
    k_ticks_t timepulse;

    if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
        if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
            timepulse_ns = k_ticks_to_ns_near64(timepulse);
        }
    }
    if (data->info.fix_status == GNSS_FIX_STATUS_GNSS_FIX) {
        char buffer[64];
        LOG_DBG("UTC Time: %02d %02d:%02d", data->utc.month, data->utc.hour, data->utc.minute);
        snprintf(buffer, sizeof(buffer), "Time:%02d:%02d", data->utc.hour, data->utc.minute);

        cfb_print(display_dev, buffer, 0, 0);  // Print at
        cfb_framebuffer_finalize(display_dev);
    }
}
GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void gnss_satellites_cb(const struct device* dev, const struct gnss_satellite* satellites, uint16_t size) {
    unsigned int tracked_count = 0;
    unsigned int corrected_count = 0;

    for (unsigned int i = 0; i != size; ++i) {
        tracked_count += satellites[i].is_tracked;
        corrected_count += satellites[i].is_corrected;
    }
    printk("%u satellite%s reported (of which %u tracked, of which %u has RTK corrections)!\n", size,
           size > 1 ? "s" : "", tracked_count, corrected_count);
}
#endif
GNSS_SATELLITES_CALLBACK_DEFINE(GNSS_MODEM, gnss_satellites_cb);

#define GNSS_SYSTEMS_PRINTF(define, supported, enabled)                                                     \
    printk("\t%20s: Supported: %3s Enabled: %3s\n", STRINGIFY(define), (supported & define) ? "Yes" : "No", \
           (enabled & define) ? "Yes" : "No");

int main(void) {
    // Starting display
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready\n");
        return;
    }

    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) != 0) {
        LOG_ERR("Failed to set required pixel format");
        return;
    }

    if (cfb_framebuffer_init(display_dev)) {
        LOG_ERR("Framebuffer init failed\n");
        return;
    }

    cfb_framebuffer_clear(display_dev, true);

    display_blanking_off(display_dev);
    display_set_orientation(display_dev, DISPLAY_ORIENTATION_ROTATED_180);

    rows = cfb_get_display_parameter(display_dev, CFB_DISPLAY_ROWS);
    ppt = cfb_get_display_parameter(display_dev, CFB_DISPLAY_PPT);

    for (int idx = 0; idx < MAX_FONTS; idx++) {
        if (cfb_get_font_size(display_dev, idx, &font_width, &font_height)) {
            break;  // end of font list, so exit loop.
        }

        LOG_DBG("index[%d] font width %d, font height %d", idx, font_width, font_height);
    }

    cfb_framebuffer_set_font(display_dev, SELECTED_FONT_INDEX);

    cfb_framebuffer_invert(display_dev);  // Optional: Invert the display (bright text on dark background)

    LOG_DBG("Starting GNSS test application\n");
    gnss_systems_t supported, enabled;
    uint32_t fix_interval;
    int rc;

    rc = gnss_get_supported_systems(GNSS_MODEM, &supported);
    if (rc < 0) {
        LOG_ERR("Failed to query supported systems (%d)\n", rc);
        return rc;
    }
    rc = gnss_get_enabled_systems(GNSS_MODEM, &enabled);
    if (rc < 0) {
        LOG_ERR("Failed to query enabled systems (%d)\n", rc);
        return rc;
    }

    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_GPS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_GLONASS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_GALILEO, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_BEIDOU, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_QZSS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_IRNSS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_SBAS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_IMES, supported, enabled);

    rc = gnss_get_fix_rate(GNSS_MODEM, &fix_interval);
    if (rc < 0) {
        LOG_ERR("Failed to query fix rate (%d)\n", rc);
        return rc;
    }
    LOG_DBG("Fix rate = %d ms\n", fix_interval);

    /* Start thread */
    k_thread_create(&imu_thread, imu_stack, THREAD_STACK_SIZE, imu_thread_entry, NULL, NULL, NULL, IMU_THREAD_PRIORITY,
                    0, K_NO_WAIT);

    k_thread_name_set(&imu_thread, "imu_thread");
}
