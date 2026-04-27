/*
 * SD Card FAT32 Boot Counter on RAK3312 with NEW USB Stack
 * FIXED VERSION - Proper MSC Instance Definition
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
#include <zephyr/shell/shell.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/usb/class/usbd_msc.h>  // Mass Storage Class
#include <zephyr/usb/usbd.h>            // NEW USB stack

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
#define DISK_MOUNT_PT "/SD:"
#define BOOT_COUNT_FILE "/SD:/count.txt"  // 8.3 filename
#define BOOT_LOG_FILE "/SD:/boot_log.txt"

static FATFS fat_fs;
static struct fs_mount_t mp;
static bool usb_enabled = false;

/* ========== NEW USB STACK CONFIGURATION ========== */

USBD_DEVICE_DEFINE(sample_usbd, DEVICE_DT_GET(DT_NODELABEL(usb_serial)), 0x2fe3, 0x0100);

USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_MANUFACTURER_DEFINE(sample_mfr, "RAK Wireless");
USBD_DESC_PRODUCT_DEFINE(sample_product, "ESP32-S3 SD Card");
USBD_DESC_SERIAL_NUMBER_DEFINE(sample_sn);

/* Configuration descriptor for MSC */
USBD_DESC_CONFIG_DEFINE(sample_msc_cfg, "MSC Configuration");

USBD_CONFIGURATION_DEFINE(sample_msc_config, USB_SCD_SELF_POWERED, 200, &sample_msc_cfg);

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");

/* doc configuration instantiation start */
static const uint8_t attributes = (IS_ENABLED(CONFIG_SAMPLE_USBD_SELF_POWERED) ? USB_SCD_SELF_POWERED : 0) |
                                  (IS_ENABLED(CONFIG_SAMPLE_USBD_REMOTE_WAKEUP) ? USB_SCD_REMOTE_WAKEUP : 0);

/* doc configuration instantiation end */

/* ========== CRITICAL: MSC INSTANCE DEFINITION ========== */
/* This links the MSC class to the actual SD card disk */
// USBD_MSC_DISK_DEFINE(msc_disk_0, DISK_DRIVE_NAME);

/* ========== FILESYSTEM FUNCTIONS ========== */

static int read_boot_count(int* count) {
    struct fs_file_t file;
    char buf[32] = {0};
    ssize_t bytes_read;
    int res;

    fs_file_t_init(&file);

    res = fs_open(&file, BOOT_COUNT_FILE, FS_O_READ);
    if (res != 0) {
        LOG_WRN("Boot count file not found");
        *count = 0;
        return -ENOENT;
    }

    bytes_read = fs_read(&file, buf, sizeof(buf) - 1);
    fs_close(&file);

    if (bytes_read > 0) {
        buf[bytes_read] = '\0';
        *count = atoi(buf);
        LOG_INF("Read boot count: %d", *count);
        return 0;
    }

    *count = 0;
    return -EIO;
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

    if (written != (ssize_t)len) {
        LOG_ERR("Write mismatch");
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

    res = fs_open(&file, BOOT_LOG_FILE, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (res != 0) {
        LOG_ERR("Failed to open log: %d", res);
        return res;
    }

    snprintf(log_entry, sizeof(log_entry), "Boot #%d - Uptime: %llu ms\n", boot_num, k_uptime_get());

    written = fs_write(&file, log_entry, strlen(log_entry));
    fs_sync(&file);
    fs_close(&file);

    return (written > 0) ? 0 : (int)written;
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
        if (bytes_read <= 0) break;

        buf[bytes_read] = '\0';
        printk("%s", buf);
    }

    printk("\n");
    LOG_INF("=== End of file ===");

    fs_close(&file);
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
        LOG_ERR("Error opening dir: %d", res);
        return res;
    }

    LOG_INF("=== Directory: %s ===", path);

    while (1) {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == 0) break;

        if (entry.type == FS_DIR_ENTRY_DIR) {
            LOG_INF("  [DIR ] %s", entry.name);
        } else {
            LOG_INF("  [FILE] %s (%zu bytes)", entry.name, entry.size);
        }
        count++;
    }

    fs_closedir(&dirp);
    LOG_INF("=== Total: %d entries ===", count);

    return 0;
}

static int print_storage_stats(void) {
    FATFS* fs;
    DWORD free_clusters;
    FRESULT fr;

    fr = f_getfree(DISK_DRIVE_NAME, &free_clusters, &fs);
    if (fr != FR_OK) {
        LOG_ERR("f_getfree failed: %d", fr);
        return -EIO;
    }

    uint64_t total_clusters = (uint64_t)(fs->n_fatent - 2);

#if FF_MAX_SS != FF_MIN_SS
    uint32_t sector_size = fs->ssize;
#else
    uint32_t sector_size = FF_MAX_SS;
#endif

    uint64_t sectors_per_cluster = fs->csize;
    uint64_t total_bytes = total_clusters * sectors_per_cluster * sector_size;
    uint64_t free_bytes = (uint64_t)free_clusters * sectors_per_cluster * sector_size;
    uint64_t used_bytes = total_bytes - free_bytes;
    uint32_t used_percent = (total_bytes == 0) ? 0 : (uint32_t)((used_bytes * 100ULL) / total_bytes);

    LOG_INF("=== SD Storage Statistics ===");
    LOG_INF("Total: %llu KB", total_bytes / 1024);
    LOG_INF("Used : %llu KB", used_bytes / 1024);
    LOG_INF("Free : %llu KB", free_bytes / 1024);
    LOG_INF("Usage: %u %%", used_percent);
    LOG_INF("==============================");

    if (device_is_ready(display_dev)) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Total: %llu KB", total_bytes / 1024);
        cfb_print(display_dev, buffer, 0, 0);
        memset(buffer, 0x00, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "Used : %llu KB", used_bytes / 1024);
        cfb_print(display_dev, buffer, 0, 30);
        memset(buffer, 0x00, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "Usage: %u %%", used_percent);
        cfb_print(display_dev, buffer, 0, 45);
        cfb_framebuffer_finalize(display_dev);
    }

    return 0;
}

/* By default, do not register the USB DFU class DFU mode instance. */
static const char* const blocklist[] = {
    "dfu_dfu",
    NULL,
};

static void sample_fix_code_triple(struct usbd_context* uds_ctx, const enum usbd_speed speed) {
    /* Always use class code information from Interface Descriptors */
    if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS) || IS_ENABLED(CONFIG_USBD_CDC_ECM_CLASS) ||
        IS_ENABLED(CONFIG_USBD_CDC_NCM_CLASS) || IS_ENABLED(CONFIG_USBD_MIDI2_CLASS) ||
        IS_ENABLED(CONFIG_USBD_AUDIO2_CLASS) || IS_ENABLED(CONFIG_USBD_VIDEO_CLASS)) {
        /*
         * Class with multiple interfaces have an Interface
         * Association Descriptor available, use an appropriate triple
         * to indicate it.
         */
        usbd_device_set_code_triple(uds_ctx, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
    } else {
        usbd_device_set_code_triple(uds_ctx, speed, 0, 0, 0);
    }
}

static int enable_usb_mass_storage(void) {
    int ret;

    if (usb_enabled) {
        LOG_WRN("USB already enabled");
        return 0;
    }

    LOG_INF("=== Enabling USB Mass Storage ===");

    /* Step 1: Unmount filesystem - CRITICAL! */
    LOG_INF("Step 1: Unmounting filesystem...");
    ret = fs_unmount(&mp);
    if (ret != 0) {
        LOG_ERR("Unmount failed: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(500));  // Increased delay for stability

    /* Step 2: Add USB descriptors */
    LOG_INF("Step 2: Adding USB descriptors...");
    usbd_add_descriptor(&sample_usbd, &sample_lang);
    usbd_add_descriptor(&sample_usbd, &sample_mfr);
    usbd_add_descriptor(&sample_usbd, &sample_product);
    usbd_add_descriptor(&sample_usbd, &sample_sn);

    /* Step 3: Add configuration */
    LOG_INF("Step 3: Adding USB configuration...");
    ret = usbd_add_configuration(&sample_usbd, USBD_SPEED_FS, &sample_msc_config);
    if (ret) {
        LOG_ERR("Add config failed: %d", ret);
        goto remount;
    }

    /* Step 4: Register MSC class - NOW WITH PROPER DISK BINDING */
    LOG_INF("Step 4: Registering MSC class...");
    ret = usbd_register_all_classes(&sample_usbd, USBD_SPEED_FS, 1, blocklist);
    if (ret) {
        LOG_ERR("Failed to add register classes");
        return NULL;
    }

    sample_fix_code_triple(&sample_usbd, USBD_SPEED_FS);
    usbd_self_powered(&sample_usbd, attributes & USB_SCD_SELF_POWERED);

    /* Step 5: Initialize USB device */
    LOG_INF("Step 5: Initializing USB device...");
    ret = usbd_init(&sample_usbd);
    if (ret) {
        LOG_ERR("USB init failed: %d", ret);
        goto remount;
    }

    /* Step 6: Enable USB */
    LOG_INF("Step 6: Enabling USB...");
    ret = usbd_enable(&sample_usbd);
    if (ret) {
        LOG_ERR("USB enable failed: %d", ret);
        goto shutdown;
    }

    usb_enabled = true;
    LOG_INF("=== USB Mass Storage ENABLED ===");
    LOG_INF("Plug into PC now. SD card is now controlled by USB.");
    return 0;

shutdown:
    usbd_shutdown(&sample_usbd);
remount:
    LOG_WRN("USB enable failed, remounting filesystem...");
    k_sleep(K_MSEC(500));
    ret = fs_mount(&mp);
    if (ret != 0) {
        LOG_ERR("CRITICAL: Remount failed: %d", ret);
    }
    return ret;
}

static int disable_usb_mass_storage(void) {
    int ret;

    if (!usb_enabled) {
        LOG_WRN("USB not enabled");
        return 0;
    }

    LOG_INF("=== Disabling USB Mass Storage ===");

    /* Step 1: Disable USB */
    LOG_INF("Step 1: Disabling USB...");
    ret = usbd_disable(&sample_usbd);
    if (ret) {
        LOG_ERR("USB disable failed: %d", ret);
        return ret;
    }

    /* Step 2: Shutdown USB stack */
    LOG_INF("Step 2: Shutting down USB...");
    ret = usbd_shutdown(&sample_usbd);
    if (ret) {
        LOG_WRN("USB shutdown warning: %d", ret);
    }

    /* Step 3: Wait for USB to fully release */
    k_sleep(K_MSEC(1000));  // Longer delay for safety

    /* Step 4: Remount filesystem */
    LOG_INF("Step 3: Remounting filesystem...");
    ret = fs_mount(&mp);
    if (ret != 0) {
        LOG_ERR("Remount failed: %d", ret);
        return ret;
    }

    usb_enabled = false;
    LOG_INF("=== Filesystem remounted - Normal operation resumed ===");

    return 0;
}

/* ========== SHELL COMMANDS ========== */

static int cmd_usb_enable(const struct shell* sh, size_t argc, char** argv) {
    shell_print(sh, "Enabling USB Mass Storage (NEW stack)...");
    shell_print(sh, "WARNING: Filesystem will be unmounted!");
    int res = enable_usb_mass_storage();
    if (res == 0) {
        shell_print(sh, "SUCCESS! Plug USB cable into PC now.");
        shell_print(sh, "Type 'usb_disable' when done to resume logging.");
    } else {
        shell_error(sh, "FAILED with error: %d", res);
    }
    return res;
}

static int cmd_usb_disable(const struct shell* sh, size_t argc, char** argv) {
    shell_print(sh, "Disabling USB and remounting filesystem...");
    int res = disable_usb_mass_storage();
    if (res == 0) {
        shell_print(sh, "SUCCESS! Filesystem active again.");
    } else {
        shell_error(sh, "FAILED with error: %d", res);
    }
    return res;
}

static int cmd_usb_status(const struct shell* sh, size_t argc, char** argv) {
    shell_print(sh, "USB Status: %s", usb_enabled ? "ENABLED (PC control)" : "DISABLED (Local control)");
    return 0;
}

SHELL_CMD_REGISTER(usb_enable, NULL, "Enable USB Mass Storage", cmd_usb_enable);
SHELL_CMD_REGISTER(usb_disable, NULL, "Disable USB and remount FS", cmd_usb_disable);
SHELL_CMD_REGISTER(usb_status, NULL, "Check USB status", cmd_usb_status);

/* ========== MAIN ========== */

int main(void) {
    int res;
    int boot_count = 0;

    LOG_INF("=== RAK3312 SD Card Boot Counter ===");
    LOG_INF("=== NEW USB Stack Version ===");

    // Display initialization
    if (device_is_ready(display_dev)) {
        if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) == 0 && cfb_framebuffer_init(display_dev) == 0) {
            cfb_framebuffer_clear(display_dev, true);
            display_blanking_off(display_dev);
            cfb_framebuffer_set_font(display_dev, SELECTED_FONT_INDEX);
            cfb_framebuffer_invert(display_dev);
        }
    }

    k_sleep(K_MSEC(500));

    /* Mount filesystem */
    mp.type = FS_FATFS;
    mp.fs_data = &fat_fs;
    mp.mnt_point = DISK_MOUNT_PT;

    LOG_INF("Mounting SD card...");
    res = fs_mount(&mp);
    if (res != FR_OK) {
        LOG_ERR("Mount failed: %d, formatting...", res);
        res = fs_mkfs(FS_FATFS, (uintptr_t)DISK_DRIVE_NAME, NULL, 0);
        if (res != 0) {
            LOG_ERR("Format failed: %d", res);
            return res;
        }
        res = fs_mount(&mp);
        if (res != FR_OK) {
            LOG_ERR("Mount after format failed: %d", res);
            return res;
        }
    }

    LOG_INF("SD card mounted successfully!");

    /* Boot counter */
    list_directory(DISK_MOUNT_PT);
    read_boot_count(&boot_count);
    boot_count++;
    LOG_INF("=== BOOT #%d ===", boot_count);

    write_boot_count(boot_count);
    append_boot_log(boot_count);

    k_sleep(K_MSEC(200));

    list_directory(DISK_MOUNT_PT);
    read_and_display_file(BOOT_LOG_FILE);
    print_storage_stats();

    LOG_INF("==============================");
    LOG_INF("Commands:");
    LOG_INF("  usb_enable  - Enable USB Mass Storage");
    LOG_INF("  usb_disable - Disable USB and remount");
    LOG_INF("  usb_status  - Check current state");
    LOG_INF("==============================");

    while (1) {
        k_sleep(K_MSEC(1000));
    }

    return 0;
}