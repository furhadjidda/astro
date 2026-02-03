#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rak14001, LOG_LEVEL_INF);

/* Get LED device from devicetree */
#define LED_DEV_NODE DT_NODELABEL(ncp5623)

static const struct device* led_dev;

int main(void) {
    int ret;

    LOG_INF("===================================");
    LOG_INF("RAK14001 RGB LED Demo for ESP32-S3");
    LOG_INF("Using Zephyr LED API");
    LOG_INF("===================================");
    k_sleep(K_MSEC(4000));
    /* Get LED device */
    led_dev = DEVICE_DT_GET(LED_DEV_NODE);

    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED device %s not ready", led_dev->name);
        return -ENODEV;
    }

    LOG_INF("LED device %s is ready", led_dev->name);

    while (1) {
        /* Red */
        LOG_INF("Turning LED ON (all channels)");
        led_set_color(led_dev, 0, 3, (uint8_t[]){255, 255, 255});
        k_sleep(K_MSEC(1000));
    }

    return 0;
}