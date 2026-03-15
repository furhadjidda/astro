/*
 * Copyright (c) 2023 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_module, LOG_LEVEL_DBG);

#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(ubloxgnss))

// display driver
static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

// Display parameters
#define MAX_FONTS 42
#define SELECTED_FONT_INDEX 0
static uint16_t rows = 0;
static uint8_t ppt = 0;
static uint8_t font_width = 0;
static uint8_t font_height = 0;

static void gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
    uint64_t timepulse_ns;
    k_ticks_t timepulse;

    if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
        if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
            timepulse_ns = k_ticks_to_ns_near64(timepulse);
            printf("Got a fix (type: %d) @ %lld ns\n", data->info.fix_status, timepulse_ns);
        } else {
            printf("Got a fix (type: %d)\n", data->info.fix_status);
        }
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
    printf("%u satellite%s reported (of which %u tracked, of which %u has RTK corrections)!\n", size,
           size > 1 ? "s" : "", tracked_count, corrected_count);
    char buffer1[64];
    char buffer2[64];
    snprintf(buffer1, sizeof(buffer1), "Satellite Reported:%02d", size);
    snprintf(buffer2, sizeof(buffer2), "Satellite Tracked:%02d", size);
    cfb_print(display_dev, buffer1, 0, 0);   // Print at
    cfb_print(display_dev, buffer2, 0, 35);  // Print at
    cfb_framebuffer_finalize(display_dev);
}
#endif
GNSS_SATELLITES_CALLBACK_DEFINE(GNSS_MODEM, gnss_satellites_cb);

#define GNSS_SYSTEMS_PRINTF(define, supported, enabled)                                                     \
    printf("\t%20s: Supported: %3s Enabled: %3s\n", STRINGIFY(define), (supported & define) ? "Yes" : "No", \
           (enabled & define) ? "Yes" : "No");

int main(void) {
    gnss_systems_t supported, enabled;
    uint32_t fix_interval;
    int rc;

    rc = gnss_get_supported_systems(GNSS_MODEM, &supported);
    if (rc < 0) {
        printf("Failed to query supported systems (%d)\n", rc);
        return rc;
    }
    rc = gnss_get_enabled_systems(GNSS_MODEM, &enabled);
    if (rc < 0) {
        printf("Failed to query enabled systems (%d)\n", rc);
        return rc;
    }

    // Starting display
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready\n");
        return -ENODEV;
    }

    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) != 0) {
        LOG_ERR("Failed to set required pixel format");
        return -ENOTSUP;
    }

    if (cfb_framebuffer_init(display_dev)) {
        LOG_ERR("Framebuffer init failed\n");
        return -EIO;
    }

    cfb_framebuffer_clear(display_dev, true);

    display_blanking_off(display_dev);

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

    printf("GNSS Systems:\n");
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
        printf("Failed to query fix rate (%d)\n", rc);
        return rc;
    }
    printf("Fix rate = %d ms\n", fix_interval);

    return 0;
}
