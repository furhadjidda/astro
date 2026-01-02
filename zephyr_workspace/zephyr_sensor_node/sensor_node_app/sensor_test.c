/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <bno055.h> // Required for custom SENSOR_CHAN_*
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
static const struct device *const bno055_dev =
    DEVICE_DT_GET(DT_NODELABEL(bno055));

static bool bno055_fusion = true;

#define BNO055_TIMING_STARTUP 400 // 400ms
int main(void) {
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(bno055)) & DEVICE_FLAG_INIT_DEFERRED
  k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
  device_init(bno055_dev);
#endif
  // k_sleep(K_MSEC(5000));
  if (!device_is_ready(bno055_dev)) {
    printk("Device %s is not ready\n", bno055_dev->name);
    return -ENODEV;
  }

struct sensor_value config = {
      .val1 = 0,
      .val2 = 0,
  };
  while (1) {
    printk("BNO055 Device %s is ready\n", bno055_dev->name);
    printk("Fetching Euler angles...\n");
    k_sleep(K_MSEC(1000));
    if (NULL != bno055_dev) {

      sensor_sample_fetch(bno055_dev);
      struct sensor_value eul[3];
      sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_EULER_YRP, eul);
    printk("EULER: X(rad.s-1)[%d.%06d] Y(rad.s-1)[%d.%06d] "
        "Z(rad.s-1)[%d.%06d]\n",
        eul[0].val1, eul[0].val2, eul[1].val1, eul[1].val2, eul[2].val1,
        eul[2].val2);
    } else {
      printk("BNO055 device is NULL!\n");
    }
    // sensor_sample_fetch(bno055_dev);
    k_sleep(K_MSEC(500));
  }
}
