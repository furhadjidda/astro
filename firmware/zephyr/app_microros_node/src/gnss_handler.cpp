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

#include "gnss_handler.hpp"

#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <string.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_DECLARE(all_sensors_module, LOG_LEVEL_DBG);

/* Gate written by node_main after micro-ROS is fully initialised */
extern atomic_t init_complete[];

/* =========================================================
 * State definitions
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
GnssState mtk3333_state = {};
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
GnssState ublox_state = {};
#endif

/* =========================================================
 * Shared helpers
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)

static void fill_nav_sat_fix(GnssState& state, const struct device* dev, const struct gnss_data* data,
                             const char* tag) {
    k_ticks_t timepulse;
    if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX && gnss_get_latest_timepulse(dev, &timepulse) == 0) {
        (void)k_ticks_to_ns_near64(timepulse);
    }

    k_spinlock_key_t key = k_spin_lock(&state.msg_lock);

    rcl_time_point_value_t now = rmw_uros_epoch_nanos();
    state.msg.header.stamp.sec = now / 1000000000LL;
    state.msg.header.stamp.nanosec = now % 1000000000LL;

    state.msg.latitude = (double)data->nav_data.latitude / 1e9;
    state.msg.longitude = (double)data->nav_data.longitude / 1e9;
    state.msg.altitude = data->nav_data.altitude / 1e3;  // mm → meters

    switch (data->info.fix_status) {
        case GNSS_FIX_STATUS_GNSS_FIX:
            state.msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
            break;
        case GNSS_FIX_STATUS_DGNSS_FIX:
            state.msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX;
            break;
        default:
            state.msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
            break;
    }

    state.msg.status.service =
        sensor_msgs__msg__NavSatStatus__SERVICE_GPS | sensor_msgs__msg__NavSatStatus__SERVICE_GLONASS;

    double hdop = data->info.hdop / 1e3;
    double variance = (hdop * 5.0) * (hdop * 5.0);
    memset(state.msg.position_covariance, 0, sizeof(state.msg.position_covariance));
    state.msg.position_covariance[0] = variance;
    state.msg.position_covariance[4] = variance;
    state.msg.position_covariance[8] = variance * 4.0;
    state.msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;

    LOG_DBG("%s: Lat=%.6f Lon=%.6f Alt=%.2f Fix=%d", tag, state.msg.latitude, state.msg.longitude, state.msg.altitude,
            data->info.fix_status);

    atomic_set_bit(&state.msg_ready, 0);
    k_spin_unlock(&state.msg_lock, key);
}

static void count_satellites(atomic_t* tracked_out, const struct gnss_satellite* satellites, uint16_t size,
                             const char* tag) {
    unsigned int tracked = 0, corrected = 0;
    for (uint16_t i = 0; i < size; ++i) {
        tracked += satellites[i].is_tracked;
        corrected += satellites[i].is_corrected;
    }
    atomic_set(tracked_out, (atomic_val_t)tracked);
    LOG_DBG("[%s] %u satellite%s reported (%u tracked, %u RTK)", tag, size, size > 1 ? "s" : "", tracked, corrected);
}

#endif /* gnss || ubloxgnss */

/* =========================================================
 * MTK3333 callbacks
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)

static void mtk3333_gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
    if (!atomic_test_bit(init_complete, 0) || data == NULL) return;
    LOG_DBG("MTK3333 UTC: %02d:%02d", data->utc.hour, data->utc.minute);
    fill_nav_sat_fix(mtk3333_state, dev, data, "MTK3333");
}
GNSS_DATA_CALLBACK_DEFINE(mtk3333_gnss, mtk3333_gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void mtk3333_satellites_cb(const struct device* dev, const struct gnss_satellite* satellites, uint16_t size) {
    ARG_UNUSED(dev);
    count_satellites(&mtk3333_state.satellites_tracked, satellites, size, "mtk3333");
}
GNSS_SATELLITES_CALLBACK_DEFINE(mtk3333_gnss, mtk3333_satellites_cb);
#endif /* CONFIG_GNSS_SATELLITES */

#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) */

/* =========================================================
 * u-blox callbacks
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)

static void ublox_gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
    if (!atomic_test_bit(init_complete, 0) || data == NULL) return;
    LOG_DBG("uBlox UTC: %02d:%02d", data->utc.hour, data->utc.minute);
    fill_nav_sat_fix(ublox_state, dev, data, "ublox");
}
GNSS_DATA_CALLBACK_DEFINE(ublox_gnss, ublox_gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void ublox_satellites_cb(const struct device* dev, const struct gnss_satellite* satellites, uint16_t size) {
    ARG_UNUSED(dev);
    count_satellites(&ublox_state.satellites_tracked, satellites, size, "ublox");
}
GNSS_SATELLITES_CALLBACK_DEFINE(ublox_gnss, ublox_satellites_cb);
#endif /* CONFIG_GNSS_SATELLITES */

#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay) */
