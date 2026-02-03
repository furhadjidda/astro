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
    /* Get LED device */
    led_dev = DEVICE_DT_GET(LED_DEV_NODE);

    static uint8_t rgb_colors[3] = {255, 255, 0};

    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED device %s not ready", led_dev->name);
        return -ENODEV;
    }

    LOG_INF("LED device %s is ready", led_dev->name);

    /* CRITICAL: Enable the LED first with brightness */
    ret = led_set_color(led_dev, 0, 3, rgb_colors);
    if (ret) {
        LOG_ERR("Failed to set color: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(100));  // Small delay after enabling
    while (1) {
        /* Red */
        LOG_INF("Turning LED ON (all channels)");
        // ret = led_set_color(led_dev, 0, 3, rgb_colors);

        // if (ret) {
        //     LOG_ERR("Failed to set color: %d", ret);
        //     return ret;
        // }
        k_sleep(K_MSEC(1000));
    }

    return 0;
}