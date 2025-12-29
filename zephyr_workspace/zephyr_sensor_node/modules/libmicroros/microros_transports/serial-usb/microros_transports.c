#include <microros_transports.h>
#include <uxr/client/transport.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define RING_BUF_SIZE 4096
#define CDC_ACM_NODE DT_CHOSEN(zephyr_console)

/* -------------------------------------------------------------------------- */
/* Static data                                                                */
/* -------------------------------------------------------------------------- */

static const struct device *cdc_dev;

static uint8_t rx_buffer[RING_BUF_SIZE];
static struct ring_buf rx_ringbuf;

static struct k_sem rx_sem;

static atomic_t rx_drops;
static bool transport_open;

/* -------------------------------------------------------------------------- */
/* CDC ACM IRQ callback                                                       */
/* -------------------------------------------------------------------------- */

static void cdc_acm_irq_callback(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);

  if (!uart_irq_update(dev)) {
    return;
  }

  while (uart_irq_rx_ready(dev)) {
    uint8_t c;

    while (uart_fifo_read(dev, &c, 1) == 1) {
      if (ring_buf_put(&rx_ringbuf, &c, 1) == 0) {
        atomic_inc(&rx_drops);
      } else {
        /* Wake reader thread */
        k_sem_give(&rx_sem);
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
/* micro-ROS transport API                                                    */
/* -------------------------------------------------------------------------- */

bool zephyr_transport_open(struct uxrCustomTransport *transport) {
  ARG_UNUSED(transport);

  if (transport_open) {
    return true;
  }

  cdc_dev = DEVICE_DT_GET(CDC_ACM_NODE);
  if (!device_is_ready(cdc_dev)) {
    printk("micro-ROS CDC ACM device not ready\n");
    return false;
  }

  ring_buf_init(&rx_ringbuf, sizeof(rx_buffer), rx_buffer);
  k_sem_init(&rx_sem, 0, K_SEM_MAX_LIMIT);

  atomic_set(&rx_drops, 0);

  int ret = uart_irq_callback_set(cdc_dev, cdc_acm_irq_callback);
  if (ret != 0) {
    printk("Failed to set UART IRQ callback (%d)\n", ret);
    return false;
  }

  uart_irq_rx_enable(cdc_dev);

  transport_open = true;
  return true;
}

bool zephyr_transport_close(struct uxrCustomTransport *transport) {
  ARG_UNUSED(transport);

  if (!transport_open) {
    return true;
  }

  uart_irq_rx_disable(cdc_dev);
  transport_open = false;

  return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err) {
  ARG_UNUSED(transport);

  /* Blocking TX: acceptable for low-rate micro-ROS traffic */
  for (size_t i = 0; i < len; i++) {
    uart_poll_out(cdc_dev, buf[i]);
  }

  if (err) {
    *err = 0;
  }

  return len;
}

size_t zephyr_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err) {
  ARG_UNUSED(transport);

  if (err) {
    *err = 0;
  }

  /* XRCE-DDS tolerates partial reads.
   * Block until data arrives or timeout expires.
   */
  if (ring_buf_is_empty(&rx_ringbuf)) {
    int ret = k_sem_take(&rx_sem, K_MSEC(timeout));
    if (ret != 0) {
      /* Timeout */
      return 0;
    }
  }

  unsigned int key = irq_lock();
  size_t read = ring_buf_get(&rx_ringbuf, buf, len);
  irq_unlock(key);

  return read;
}
