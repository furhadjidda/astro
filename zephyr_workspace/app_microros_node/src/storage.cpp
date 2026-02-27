/*
 *   This file is part of astro.
 *
 *   astro is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "storage.hpp"

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/SD:"
#define BOOT_COUNT_FILE "/SD:/count.txt"
#define BOOT_LOG_FILE "/SD:/boot_log.txt"

LOG_MODULE_REGISTER(storage);

static FATFS fat_fs;
static struct fs_mount_t mp;

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

/* By default, do not register the USB DFU class DFU mode instance. */
static const char* const blocklist[] = {
    "dfu_dfu",
    NULL,
};

int Storage::init() {
    int res = 0;
    int boot_count = 0;

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

    return 0;
}

int Storage::read_boot_count(int* count) {
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

int Storage::write_boot_count(int count) {
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

int Storage::append_boot_log(int boot_num) {
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

int Storage::read_and_display_file(const char* filepath) {
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

int Storage::list_directory(const char* path) {
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

int Storage::print_storage_stats(void) {
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

    return 0;
}