/*
 * SD Card FAT32 Boot Counter on RAK3312
 */

#include <ff.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>

// display driver
static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
// Display parameters
#define MAX_FONTS 42
#define SELECTED_FONT_INDEX 0
static uint16_t rows = 0;
static uint8_t ppt = 0;
static uint8_t font_width = 0;
static uint8_t font_height = 0;

LOG_MODULE_REGISTER(fs_main);

#define DISK_DRIVE_NAME "SD"
// For FATFS in Zephyr, the colon is part of the mount name because FAT historically uses drive identifiers:
#define DISK_MOUNT_PT "/SD:"
#define BOOT_COUNT_FILE "/SD:/boot_count.txt"
#define BOOT_LOG_FILE "/SD:/boot_log.txt"

static FATFS fat_fs;

static struct fs_mount_t mp;

static int read_boot_count(int* count) {
    struct fs_file_t file;
    char buf[32] = {0};
    ssize_t bytes_read;
    int res;

    fs_file_t_init(&file);

    LOG_INF("Attempting to read boot count from %s", BOOT_COUNT_FILE);
    res = fs_open(&file, BOOT_COUNT_FILE, FS_O_READ);
    if (res != 0) {
        LOG_WRN("Boot count file does not exist (error %d), will start at 0", res);
        *count = 0;
        return -ENOENT;
    }

    bytes_read = fs_read(&file, buf, sizeof(buf) - 1);
    LOG_INF("Read %d bytes from boot count file", (int)bytes_read);

    if (bytes_read > 0) {
        buf[bytes_read] = '\0';
        *count = atoi(buf);
        LOG_INF("Successfully read boot count: %d (raw: '%s')", *count, buf);
    } else {
        LOG_WRN("Read returned %d bytes, defaulting to 0", (int)bytes_read);
        *count = 0;
    }

    fs_close(&file);
    return 0;
}

static int write_boot_count(int count) {
    struct fs_file_t file;
    char buf[32];
    int res;
    ssize_t written;
    size_t len;

    fs_file_t_init(&file);

    res = fs_open(&file, BOOT_COUNT_FILE, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
    if (res != 0) {
        LOG_ERR("Failed to open boot count file: %d", res);
        return res;
    }

    len = snprintf(buf, sizeof(buf), "%d", count);

    written = fs_write(&file, buf, len);
    if (written != len) {
        LOG_ERR("Write mismatch (%d vs %d)", (int)written, (int)len);
        fs_close(&file);
        return -EIO;
    }

    fs_sync(&file);
    fs_close(&file);

    LOG_INF("Boot count updated to %d", count);
    return 0;
}

static int append_boot_log(int boot_num) {
    struct fs_file_t file;
    char log_entry[128];
    int res;
    ssize_t written;

    fs_file_t_init(&file);

    /* Open in append mode */
    res = fs_open(&file, BOOT_LOG_FILE, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (res != 0) {
        LOG_ERR("Failed to open log file: %d", res);
        return res;
    }

    /* Create log entry with timestamp */
    snprintf(log_entry, sizeof(log_entry), "Boot #%d - Uptime: %llu ms\n", boot_num, k_uptime_get());

    written = fs_write(&file, log_entry, strlen(log_entry));

    /* Sync to disk */
    fs_sync(&file);
    fs_close(&file);

    if (written > 0) {
        LOG_INF("Appended boot log entry");
    } else {
        LOG_ERR("Failed to append to log: %d", (int)written);
    }

    return (written > 0) ? 0 : written;
}

static int read_and_display_file(const char* filepath) {
    struct fs_file_t file;
    char buf[256];
    ssize_t bytes_read;
    int res;

    fs_file_t_init(&file);

    res = fs_open(&file, filepath, FS_O_READ);
    if (res != 0) {
        LOG_WRN("Could not open %s: %d", filepath, res);
        return res;
    }

    LOG_INF("=== Contents of %s ===", filepath);

    while (1) {
        bytes_read = fs_read(&file, buf, sizeof(buf) - 1);
        if (bytes_read <= 0) {
            break;
        }

        buf[bytes_read] = '\0';
        printk("%s", buf);
    }

    printk("\n");
    LOG_INF("=== End of file ===");

    fs_close(&file);
    return 0;
}

static int purge_directory(const char* path) {
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int res;

    fs_dir_t_init(&dir);

    res = fs_opendir(&dir, path);
    if (res != 0) {
        LOG_ERR("Failed to open dir %s (%d)", path, res);
        return res;
    }

    while (1) {
        res = fs_readdir(&dir, &entry);
        if (res != 0 || entry.name[0] == 0) {
            break;
        }

        /* Build full path */
        char full_path[256];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, entry.name);

        if (entry.type == FS_DIR_ENTRY_DIR) {
            LOG_INF("Deleting directory: %s", full_path);

            /* Recursive delete */
            purge_directory(full_path);

            /* Remove empty directory */
            fs_unlink(full_path);
        } else {
            LOG_INF("Deleting file: %s", full_path);
            fs_unlink(full_path);
        }
    }

    fs_closedir(&dir);
    return 0;
}

static int list_directory(const char* path) {
    struct fs_dir_t dirp;
    struct fs_dirent entry;
    int res;
    int count = 0;

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, path);
    if (res != 0) {
        LOG_ERR("Error opening dir %s: %d", path, res);
        return res;
    }

    LOG_INF("=== Directory listing: %s ===", path);

    while (1) {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == 0) {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR) {
            LOG_INF("  [DIR ] %s", entry.name);
        } else {
            LOG_INF("  [FILE] %s (%zu bytes)", entry.name, entry.size);
        }
        count++;
    }

    fs_closedir(&dirp);

    if (count == 0) {
        LOG_INF("  (empty)");
    }
    LOG_INF("=== Total: %d entries ===", count);

    return 0;
}

static void purge_sd_card(void) {
    LOG_WRN("=== PURGING SD CARD CONTENTS ===");

    purge_directory(DISK_MOUNT_PT);

    LOG_WRN("=== SD CARD PURGE COMPLETE ===");
}

int main(void) {
    int res;
    int boot_count = 0;

    // Starting display
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready\n");
        return -ENODEV;
    }

    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) != 0) {
        LOG_ERR("Failed to set required pixel format");
        return -ENOTSUP;
    }

    if (cfb_framebuffer_init(display_dev)) {
        LOG_ERR("Framebuffer init failed\n");
        return -EIO;
    }

    cfb_framebuffer_clear(display_dev, true);

    display_blanking_off(display_dev);
    rows = cfb_get_display_parameter(display_dev, CFB_DISPLAY_ROWS);
    ppt = cfb_get_display_parameter(display_dev, CFB_DISPLAY_PPT);

    for (int idx = 0; idx < MAX_FONTS; idx++) {
        if (cfb_get_font_size(display_dev, idx, &font_width, &font_height)) {
            break;  // end of font list, so exit loop.
        }

        LOG_DBG("index[%d] font width %d, font height %d", idx, font_width, font_height);
    }

    cfb_framebuffer_set_font(display_dev, SELECTED_FONT_INDEX);

    cfb_framebuffer_invert(display_dev);  // Optional: Invert the display (bright text on dark background)

    LOG_INF("=== RAK3312 SD Card Boot Counter ===");

    k_sleep(K_MSEC(500));

    /* Mount filesystem */
    LOG_INF("Mounting FAT filesystem at %s", DISK_MOUNT_PT);
    mp.type = FS_FATFS;
    mp.fs_data = &fat_fs;
    mp.mnt_point = DISK_MOUNT_PT;

    res = fs_mount(&mp);
    if (res != FR_OK) {
        LOG_ERR("Failed to mount SD card: %d", res);
        LOG_INF("Attempting to format SD card...");

        res = fs_mkfs(FS_FATFS, (uintptr_t)DISK_DRIVE_NAME, NULL, 0);
        if (res != 0) {
            LOG_ERR("Format failed: %d", res);
            return res;
        }

        LOG_INF("Format complete, mounting...");
        res = fs_mount(&mp);
        if (res != FR_OK) {
            LOG_ERR("Mount after format failed: %d", res);
            return res;
        }
    }

#if IS_ENABLED(CONFIG_APP_PURGE_SD_ON_BOOT)
    purge_sd_card();
#endif

    LOG_INF("SD card mounted successfully!");

    /* List current contents BEFORE operations */
    LOG_INF("--- BEFORE boot count operations ---");
    list_directory(DISK_MOUNT_PT);

    /* Read current boot count */
    res = read_boot_count(&boot_count);
    LOG_INF("Read returned: %d, boot_count = %d", res, boot_count);

    /* Increment boot count */
    boot_count++;
    LOG_INF("=== BOOT #%d ===", boot_count);

    /* Write updated boot count */
    res = write_boot_count(boot_count);
    if (res != 0) {
        LOG_ERR("Failed to write boot count: %d", res);
    }

    /* Append to boot log */
    res = append_boot_log(boot_count);
    if (res != 0) {
        LOG_ERR("Failed to append boot log: %d", res);
    }

    /* CRITICAL: Give filesystem time to flush buffers */
    LOG_INF("Waiting for filesystem to sync...");
    k_sleep(K_MSEC(500));

    /* List contents AFTER operations */
    LOG_INF("--- AFTER boot count operations ---");
    list_directory(DISK_MOUNT_PT);

    /* Verify the boot count file was created */
    LOG_INF("Verifying boot count file...");
    int verify_count = -1;
    read_boot_count(&verify_count);
    LOG_INF("Verification read: %d (should be %d)", verify_count, boot_count);

    /* Display the boot log */
    read_and_display_file(BOOT_LOG_FILE);

    LOG_INF("Boot counter test complete!");
    LOG_INF("Current boot count: %d", boot_count);
    LOG_INF("Reboot to see the counter increment");

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "System ready. Current boot count: %d", boot_count);
    cfb_print(display_dev, buffer, 0, 0);  // Print at
    cfb_framebuffer_finalize(display_dev);

    while (1) {
        k_sleep(K_MSEC(1000));
    }

    return 0;
}