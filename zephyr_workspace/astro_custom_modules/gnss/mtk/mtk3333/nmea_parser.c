/*
 * gnss_nmea_zephyr.c
 *
 * NMEA validation + parsing for Zephyr GNSS structures
 * Supports:
 *   - GGA: time, lat, lon, altitude, HDOP, satellites, fix quality
 *   - GSA: PDOP, HDOP (optional extension point)
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Zephyr GNSS types */
#include <zephyr/drivers/gnss.h>

/* ==============================
 * Utility Helpers
 * ============================== */

/* Convert ddmm.mmmm to nanodegrees */
static int64_t nmea_degmin_to_nanodeg(double degmin) {
    int deg = (int)(degmin / 100);
    double min = degmin - (deg * 100);
    double dec_deg = deg + (min / 60.0);

    return (int64_t)(dec_deg * 1e9);
}

static int hex_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* ==============================
 * NMEA Validation
 * ============================== */

static int nmea_basic_validate(const char* s) {
    if (!s) return 0;
    if (s[0] != '$') return 0;
    if (!strchr(s, '*')) return 0;
    return 1;
}

static uint8_t nmea_checksum_calc(const char* s) {
    uint8_t csum = 0;

    s++; /* skip '$' */
    while (*s && *s != '*') {
        csum ^= (uint8_t)(*s);
        s++;
    }

    return csum;
}

static int nmea_checksum_validate(const char* s) {
    const char* star = strchr(s, '*');
    if (!star || strlen(star) < 3) {
        return 0;
    }

    int hi = hex_to_int(star[1]);
    int lo = hex_to_int(star[2]);
    if (hi < 0 || lo < 0) {
        return 0;
    }

    uint8_t expected = (hi << 4) | lo;
    uint8_t actual = nmea_checksum_calc(s);

    return expected == actual;
}

static int nmea_validate(const char* s) {
    if (!nmea_basic_validate(s)) return 0;
    if (!nmea_checksum_validate(s)) return 0;
    return 1;
}

/* ==============================
 * Time Parsing
 * ============================== */

/* hhmmss.sss */
static void parse_utc_time(const char* token, struct gnss_time* utc) {
    if (!token || strlen(token) < 6) {
        return;
    }

    utc->hour = (token[0] - '0') * 10 + (token[1] - '0');
    utc->minute = (token[2] - '0') * 10 + (token[3] - '0');

    int sec = (token[4] - '0') * 10 + (token[5] - '0');
    int msec = 0;

    const char* dot = strchr(token, '.');
    if (dot) {
        msec = atoi(dot + 1);
    }

    utc->millisecond = sec * 1000 + msec;
}

/* ==============================
 * GGA Parsing
 * ============================== */

static void parse_gga(char* sentence, struct gnss_data* data) {
    char* token;
    char* rest = sentence;  // Keep track of remaining string
    int field = 0;
    char lat_hemi = 0;
    char lon_hemi = 0;

    while ((token = strsep(&rest, ",")) != NULL) {
        if (*token == '\0') {
            field++;
            continue;  // Skip empty fields but increment counter
        }
        switch (field) {
            case 1: /* UTC time */
                parse_utc_time(token, &data->utc);
                break;

            case 2: /* Latitude */
                if (*token) {
                    data->nav_data.latitude = nmea_degmin_to_nanodeg(atof(token));
                }

                break;

            case 3: /* N/S */
                lat_hemi = token[0];
                break;

            case 4: /* Longitude */
                if (*token) {
                    data->nav_data.longitude = nmea_degmin_to_nanodeg(atof(token));
                }
                break;

            case 5: /* E/W */
                lon_hemi = token[0];
                break;

            case 6: /* Fix quality */
                data->info.fix_quality = atoi(token);
                data->info.fix_status = (atoi(token) > 0) ? GNSS_FIX_STATUS_GNSS_FIX : GNSS_FIX_STATUS_NO_FIX;
                break;

            case 7: /* Satellites */
                data->info.satellites_cnt = atoi(token);
                break;

            case 8: /* HDOP */
                data->info.hdop = (uint32_t)(atof(token) * 1000.0);
                break;

            case 9: /* Altitude (meters → mm) */
                data->nav_data.altitude = (int32_t)(atof(token) * 1000.0);
                break;

            case 11: /* Geoid separation (meters → mm) */
                data->info.geoid_separation = (int32_t)(atof(token) * 1000.0);
                break;
        }
        field++;
    }

    if (lat_hemi == 'S') data->nav_data.latitude = -data->nav_data.latitude;
    if (lon_hemi == 'W') data->nav_data.longitude = -data->nav_data.longitude;
}

/* ==============================
 * Public Parse API
 * ============================== */

int gnss_nmea_parse(const char* nmea, struct gnss_data* data) {
    if (!nmea || !data) {
        return -1;
    }

    if (!nmea_validate(nmea)) {
        return -1;
    }

    /* strtok modifies input */
    char buffer[128];
    strncpy(buffer, nmea, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    /* Reset dynamic fields */
    data->nav_data.bearing = 0;
    data->nav_data.speed = 0;

    if (strstr(buffer, "GGA")) {
        parse_gga(buffer, data);
        return 0;
    }

    /* GSA / RMC can be added here */

    return -1;
}
