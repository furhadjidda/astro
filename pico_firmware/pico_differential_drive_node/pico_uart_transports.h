/*
 *   This file is part of astro_core_ros.
 *
 *   astro_core_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro_core_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro_core_ros.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdio.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

bool pico_serial_transport_open(struct uxrCustomTransport *transport);
bool pico_serial_transport_close(struct uxrCustomTransport *transport);
size_t pico_serial_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t pico_serial_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                                  uint8_t *err);

#ifdef __cplusplus
}
#endif

#endif // MICRO_ROS_PICOSDK
