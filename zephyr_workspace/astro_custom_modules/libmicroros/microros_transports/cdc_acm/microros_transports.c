/*
 * micro-ROS custom transport over USB CDC ACM for Zephyr 4.3
 * Compatible with micro-ROS Humble
 */

#include <microros_transports.h>
#include <uxr/client/transport.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define RING_BUF_SIZE 2048
#define CDC_ACM_NODE DT_CHOSEN(zephyr_console)

/* -------------------------------------------------------------------------- */
/* Static data                                                                */
/* -------------------------------------------------------------------------- */

static const struct device *cdc_dev;

static uint8_t usb_rx_buffer[RING_BUF_SIZE];
static struct ring_buf rx_ringbuf;

/* -------------------------------------------------------------------------- */
/* CDC ACM interrupt callback */
/* -------------------------------------------------------------------------- */

static void cdc_acm_irq_callback(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);

  while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {

    if (uart_irq_rx_ready(dev)) {
      uint8_t c;

      /* Drain RX FIFO completely */
      while (uart_fifo_read(dev, &c, 1) == 1) {
        ring_buf_put(&rx_ringbuf, &c, 1);
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
/* micro-ROS transport API */
/* -------------------------------------------------------------------------- */

bool zephyr_transport_open(struct uxrCustomTransport *transport) {
  ARG_UNUSED(transport);

  cdc_dev = DEVICE_DT_GET(CDC_ACM_NODE);
  if (!device_is_ready(cdc_dev)) {
    printk("micro-ROS CDC ACM device not ready\n");
    return false;
  }

  ring_buf_init(&rx_ringbuf, sizeof(usb_rx_buffer), usb_rx_buffer);

  uart_irq_callback_set(cdc_dev, cdc_acm_irq_callback);
  uart_irq_rx_enable(cdc_dev);

  return true;
}

bool zephyr_transport_close(struct uxrCustomTransport *transport) {
  ARG_UNUSED(transport);

  if (cdc_dev != NULL) {
    uart_irq_rx_disable(cdc_dev);
  }

  return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err) {
  ARG_UNUSED(transport);
  ARG_UNUSED(err);

  /* CDC ACM uses polling TX */
  for (size_t i = 0; i < len; i++) {
    uart_poll_out(cdc_dev, buf[i]);
  }

  return len;
}

size_t zephyr_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err) {
  ARG_UNUSED(transport);
  ARG_UNUSED(err);

  size_t read = 0;
  int waited_ms = 0;

  /* Timeout is in milliseconds */
  while (ring_buf_is_empty(&rx_ringbuf) && waited_ms < timeout) {
    usleep(1000);
    waited_ms++;
  }

  unsigned int key = irq_lock();
  read = ring_buf_get(&rx_ringbuf, buf, len);
  irq_unlock(key);

  return read;
}
