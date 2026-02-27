#include <string.h>
#include <zephyr/device.h>
#include <zephyr/fs/zms.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(zms_test, LOG_LEVEL_INF);

/* ZMS IDs for our data */
#define ZMS_ID_BOOT_COUNT 1
#define ZMS_ID_MESSAGE 2
#define ZMS_ID_TIMESTAMP 3

/* ZMS file system */
static struct zms_fs zms;

#define STORAGE_PARTITION storage_partition
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION)

int main(void) {
    int rc;
    uint32_t boot_count = 0;
    char message[64];
    uint32_t timestamp;
    struct flash_pages_info info;

    LOG_INF("=== ZMS Persistent Storage Test ===\n");

    /* Get flash partition info */
    const struct flash_area* fa;
    rc = flash_area_open(STORAGE_PARTITION_ID, &fa);
    if (rc) {
        LOG_ERR("Failed to open flash area: %d", rc);
        return -1;
    }

    LOG_INF("Flash area: %s", fa->fa_dev->name);
    LOG_INF("Flash offset: 0x%x", fa->fa_off);
    LOG_INF("Flash size: 0x%x (%d bytes)", fa->fa_size, fa->fa_size);

    /* Initialize ZMS */
    zms.flash_device = fa->fa_dev;
    zms.offset = fa->fa_off;
    zms.sector_size = 4096;  // Typical SPI NOR erase size
    zms.sector_count = fa->fa_size / zms.sector_size;

    LOG_INF("Initializing ZMS with %u sectors of %u bytes", zms.sector_count, zms.sector_size);

    rc = zms_mount(&zms);
    if (rc) {
        LOG_ERR("ZMS mount failed: %d", rc);
        LOG_INF("Attempting to clear and remount...");

        rc = zms_clear(&zms);
        if (rc) {
            LOG_ERR("ZMS clear failed: %d", rc);
            flash_area_close(fa);
            return -1;
        }

        rc = zms_mount(&zms);
        if (rc) {
            LOG_ERR("ZMS mount failed after clear: %d", rc);
            flash_area_close(fa);
            return -1;
        }
    }

    LOG_INF("ZMS mounted successfully\n");

    /* Read boot count */
    rc = zms_read(&zms, ZMS_ID_BOOT_COUNT, &boot_count, sizeof(boot_count));
    if (rc > 0) {
        LOG_INF("Previous boot count: %u", boot_count);
        boot_count++;
    } else {
        LOG_INF("First boot - initializing boot count");
        boot_count = 1;
    }

    /* Read previous message */
    rc = zms_read(&zms, ZMS_ID_MESSAGE, message, sizeof(message));
    if (rc > 0) {
        message[rc] = '\0';  // Null terminate
        LOG_INF("Previous message: '%s'", message);
    } else {
        LOG_INF("No previous message found");
    }

    /* Read previous timestamp */
    rc = zms_read(&zms, ZMS_ID_TIMESTAMP, &timestamp, sizeof(timestamp));
    if (rc > 0) {
        LOG_INF("Previous timestamp: %u seconds", timestamp);
        uint32_t elapsed = k_uptime_get_32() / 1000 - timestamp;
        LOG_INF("(Approximate time since last write: %u seconds)", elapsed);
    } else {
        LOG_INF("No previous timestamp found");
    }

    LOG_INF("\n=== Writing new data ===");

    /* Write new boot count */
    rc = zms_write(&zms, ZMS_ID_BOOT_COUNT, &boot_count, sizeof(boot_count));
    if (rc < 0) {
        LOG_ERR("Failed to write boot count: %d", rc);
    } else {
        LOG_INF("Boot count written: %u", boot_count);
    }

    /* Write new message */
    snprintf(message, sizeof(message), "Hello from boot #%u!", boot_count);
    rc = zms_write(&zms, ZMS_ID_MESSAGE, message, strlen(message));
    if (rc < 0) {
        LOG_ERR("Failed to write message: %d", rc);
    } else {
        LOG_INF("Message written: '%s'", message);
    }

    /* Write timestamp (uptime in seconds) */
    timestamp = k_uptime_get_32() / 1000;
    rc = zms_write(&zms, ZMS_ID_TIMESTAMP, &timestamp, sizeof(timestamp));
    if (rc < 0) {
        LOG_ERR("Failed to write timestamp: %d", rc);
    } else {
        LOG_INF("Timestamp written: %u", timestamp);
    }

    LOG_INF("\n=== Verifying write ===");

    /* Read back to verify */
    uint32_t verify_count = 0;
    char verify_msg[64] = {0};

    rc = zms_read(&zms, ZMS_ID_BOOT_COUNT, &verify_count, sizeof(verify_count));
    if (rc > 0 && verify_count == boot_count) {
        LOG_INF("✓ Boot count verified: %u", verify_count);
    } else {
        LOG_ERR("✗ Boot count verification failed");
    }

    rc = zms_read(&zms, ZMS_ID_MESSAGE, verify_msg, sizeof(verify_msg));
    if (rc > 0) {
        verify_msg[rc] = '\0';
        if (strcmp(verify_msg, message) == 0) {
            LOG_INF("✓ Message verified: '%s'", verify_msg);
        } else {
            LOG_ERR("✗ Message mismatch");
        }
    } else {
        LOG_ERR("✗ Message read failed");
    }

    LOG_INF("\n=== ZMS Statistics ===");
    ssize_t free_space = zms_calc_free_space(&zms);
    if (free_space >= 0) {
        LOG_INF("Free space: %d bytes", free_space);
    } else {
        LOG_ERR("Failed to calculate free space: %d", free_space);
    }

    flash_area_close(fa);

    LOG_INF("\n=== Test Complete ===");
    LOG_INF("Power cycle the device to see the boot count increment!");

    return 0;
}