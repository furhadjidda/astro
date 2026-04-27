#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SLEEP_MS 1000

int main(void) {
    const struct device* dev = DEVICE_DT_GET_ANY(st_lps22hb_press);

    if (!device_is_ready(dev)) {
        printk("Error: LPS22HB device not ready\n");
        return -ENODEV;
    }

    printk("LPS22HB sensor found: %s\n", dev->name);

    while (1) {
        struct sensor_value temp, press;

        int rc = sensor_sample_fetch(dev);
        if (rc < 0) {
            printk("sensor_sample_fetch error: %d\n", rc);
            k_msleep(SLEEP_MS);
            continue;
        }

        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

        /* sensor_value has .val1 (integer) and .val2 (micro-fraction) */
        printk("Temperature: %d.%02d °C\n", temp.val1, abs(temp.val2) / 10000);

        printk("Pressure:    %d.%02d kPa\n", press.val1, abs(press.val2) / 10000);

        k_msleep(SLEEP_MS);
    }

    return 0;
}