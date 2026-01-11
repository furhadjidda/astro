#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define I2C_ADDR 0x3C  // or 0x29 depending on COM3 pin
#define REG_CHIP_ID 0x00
#define WIDTH 128
#define HEIGHT 64
#define I2C_INST DT_NODELABEL(i2c0)
static uint8_t fb[WIDTH * HEIGHT / 8];
static const int32_t sleep_time_ms = 1000;

const struct device* const i2c_dev = DEVICE_DT_GET(I2C_INST);

static int ssd1306_cmd(uint8_t cmd) {
    uint8_t buf[2];
    buf[0] = 0x00;  // Control byte: Co = 0, D/C# = 0 (command)
    buf[1] = cmd;   // The command itself

    return i2c_write(i2c_dev, buf, sizeof(buf), I2C_ADDR);
}

static int ssd1306_data(const uint8_t* data, size_t len) {
    /* Control byte 0x40 = data stream */
    uint8_t temp[129];

    while (len > 0) {
        size_t chunk = (len > 128) ? 128 : len;

        temp[0] = 0x40;
        memcpy(&temp[1], data, chunk);

        i2c_write(i2c_dev, temp, chunk + 1, I2C_ADDR);

        data += chunk;
        len -= chunk;
    }
    return 0;
}

/* --- Pixel drawing --- */

static void set_pixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;

    int index = x + (y / 8) * WIDTH;
    fb[index] |= (1 << (y & 7));
}

/* Draw a simple square (outline only) */
static void draw_square(int x0, int y0, int size) {
    int x1 = x0 + size;
    int y1 = y0 + size;

    for (int x = x0; x <= x1; x++) {
        set_pixel(x, y0);
        set_pixel(x, y1);
    }
    for (int y = y0; y <= y1; y++) {
        set_pixel(x0, y);
        set_pixel(x1, y);
    }
}

/* Send full framebuffer to the display */
static void ssd1306_update(void) {
    ssd1306_cmd(0x21);  // Column addr
    ssd1306_cmd(0x00);
    ssd1306_cmd(WIDTH - 1);

    ssd1306_cmd(0x22);  // Page addr
    ssd1306_cmd(0x00);
    ssd1306_cmd((HEIGHT / 8) - 1);

    ssd1306_data(fb, sizeof(fb));
}

void main(void) {
    bool is_ready = false;
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
    } else {
        printk("i2c is ready\n");
        is_ready = true;
    }

    /* --- Minimal Init Sequence --- */
    ssd1306_cmd(0xAE);  // Display OFF
    ssd1306_cmd(0xD5);  // Set display clock divide
    ssd1306_cmd(0x80);

    ssd1306_cmd(0xA8);  // Set multiplex ratio
    ssd1306_cmd(0x3F);  // 0x3F = 64 rows (use 0x1F for 128x32)

    ssd1306_cmd(0xD3);  // Display offset
    ssd1306_cmd(0x00);

    ssd1306_cmd(0x40);  // Start line = 0

    ssd1306_cmd(0x8D);  // Charge pump
    ssd1306_cmd(0x14);  // Enable charge pump

    ssd1306_cmd(0x20);  // Memory mode
    ssd1306_cmd(0x00);  // Horizontal addressing

    ssd1306_cmd(0xA1);  // Segment remap (mirror horizontally)
    ssd1306_cmd(0xC8);  // COM scan direction (flip vertically)

    ssd1306_cmd(0xDA);  // COM pins config
    ssd1306_cmd(0x12);  // For 128x64 (use 0x02 for 128x32)

    ssd1306_cmd(0x81);  // Contrast
    ssd1306_cmd(0x7F);

    ssd1306_cmd(0xD9);  // Pre-charge
    ssd1306_cmd(0xF1);

    ssd1306_cmd(0xDB);  // VCOM detect
    ssd1306_cmd(0x40);

    ssd1306_cmd(0xA4);  // Resume RAM content display
    ssd1306_cmd(0xA6);  // Normal display (not inverted)

    /* --- Turn the display ON --- */
    ssd1306_cmd(0xAF);

    /* Clear framebuffer */
    memset(fb, 0, sizeof(fb));

    /* Draw a 40×40 square starting at (20, 10) */
    draw_square(0, 5, 40);

    /* Update display */
    ssd1306_update();
    while (1) {
        printk("I2C device is ready: %s\n", is_ready ? "true" : "false");
        k_msleep(sleep_time_ms);
    }
}
