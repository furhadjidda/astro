

#ifndef MTK3333_PARSER_H
#define MTK3333_PARSER_H

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mtk3333.h"

int gnss_nmea_parse(const char* nmea, struct gnss_data* data)

#endif  // MTK3333_PARSER_H