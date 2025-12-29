#include "status_class.hpp"

const gpio_dt_spec StatusClass::power_ctrl =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_pwr), enable_gpios);

const device *const StatusClass::strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

StatusClass::StatusClass() { power_on(); }

void StatusClass::display_color(const struct led_rgb &color) {
  if (!device_is_ready(strip)) {
    return;
  }

  memset(&pixels, 0x00, sizeof(pixels));
  memcpy(&pixels[0], &color, sizeof(struct led_rgb));
  led_strip_update_rgb(strip, pixels, num_pixels);
}

void StatusClass::power_on(void) {
  if (!device_is_ready(power_ctrl.port)) {
    return;
  }

  gpio_pin_configure_dt(&power_ctrl, GPIO_OUTPUT_ACTIVE);
}