/*
 * micro-ROS custom transport over USB CDC ACM for Zephyr
 * Compatible with micro-ROS Humble and Zephyr 4.3
 */

#ifndef MICROROS_TRANSPORT_CDC_ACM_H_
#define MICROROS_TRANSPORT_CDC_ACM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/transport.h>

/**
 * @brief Open the custom micro-ROS transport.
 *
 * Called by micro-ROS during transport initialization.
 *
 * @param transport Pointer to the uxrCustomTransport structure.
 * @return true if the transport was opened successfully.
 * @return false on failure.
 */
bool zephyr_transport_open(struct uxrCustomTransport *transport);

/**
 * @brief Close the custom micro-ROS transport.
 *
 * @param transport Pointer to the uxrCustomTransport structure.
 * @return true if the transport was closed successfully.
 */
bool zephyr_transport_close(struct uxrCustomTransport *transport);

/**
 * @brief Write data to the transport.
 *
 * @param transport Pointer to the uxrCustomTransport structure.
 * @param buf Pointer to data buffer to transmit.
 * @param len Number of bytes to transmit.
 * @param err Optional error output (unused for CDC ACM).
 * @return Number of bytes written.
 */
size_t zephyr_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err);

/**
 * @brief Read data from the transport.
 *
 * @param transport Pointer to the uxrCustomTransport structure.
 * @param buf Pointer to destination buffer.
 * @param len Maximum number of bytes to read.
 * @param timeout Timeout in milliseconds.
 * @param err Optional error output (unused for CDC ACM).
 * @return Number of bytes read.
 */
size_t zephyr_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_TRANSPORT_CDC_ACM_H_ */
