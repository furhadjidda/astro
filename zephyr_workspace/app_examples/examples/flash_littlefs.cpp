#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(lfs_test, LOG_LEVEL_DBG);

#define STORAGE_PARTITION storage_partition
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION)

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_storage_mnt;

int main(void) {
    int rc;

    LOG_INF("=== LittleFS Flash Test ===\n");

    /* Step 1: Verify flash partition exists */
    const struct flash_area* fa;
    rc = flash_area_open(STORAGE_PARTITION_ID, &fa);
    if (rc) {
        LOG_ERR("Failed to open flash area: %d", rc);
        LOG_ERR("Make sure storage_partition is defined in device tree!");
        return -1;
    }

    LOG_INF("Flash partition found:");
    LOG_INF("  Device: %s", fa->fa_dev->name);
    LOG_INF("  Offset: 0x%x", fa->fa_off);
    LOG_INF("  Size: 0x%x (%u bytes)", fa->fa_size, fa->fa_size);

    /* Verify flash device is ready */
    if (!device_is_ready(fa->fa_dev)) {
        LOG_ERR("Flash device not ready!");
        flash_area_close(fa);
        return -1;
    }
    LOG_INF("  Status: Ready ✓\n");

    flash_area_close(fa);

    /* Step 2: Setup LittleFS mount structure */
    lfs_storage_mnt.type = FS_LITTLEFS;
    lfs_storage_mnt.mnt_point = "/lfs";
    lfs_storage_mnt.storage_dev = (void*)STORAGE_PARTITION_ID;
    lfs_storage_mnt.fs_data = &storage;
    lfs_storage_mnt.flags = 0;

    LOG_INF("Mounting LittleFS...");
    rc = fs_mount(&lfs_storage_mnt);

    if (rc == -ENOTSUP) {
        LOG_ERR("Mount failed: ENOTSUP (-134)");
        LOG_ERR("This usually means:");
        LOG_ERR("  1. Flash partition not properly defined");
        LOG_ERR("  2. Flash device not accessible");
        LOG_ERR("  3. Missing CONFIG_FLASH_MAP or CONFIG_FLASH_PAGE_LAYOUT");
        return -1;
    }

    if (rc < 0) {
        LOG_WRN("Mount failed (%d), formatting...", rc);

        /* Format the partition */
        struct fs_statvfs sbuf;
        rc = fs_statvfs(lfs_storage_mnt.mnt_point, &sbuf);
        if (rc < 0) {
            LOG_INF("Attempting to format partition...");

            /* Erase the partition first */
            rc = flash_area_open(STORAGE_PARTITION_ID, &fa);
            if (rc == 0) {
                LOG_INF("Erasing flash partition...");
                rc = flash_area_erase(fa, 0, fa->fa_size);
                if (rc) {
                    LOG_ERR("Flash erase failed: %d", rc);
                    flash_area_close(fa);
                    return -1;
                }
                flash_area_close(fa);
                LOG_INF("Flash erased successfully");
            }

            /* Try mount again */
            rc = fs_mount(&lfs_storage_mnt);
            if (rc < 0) {
                LOG_ERR("Mount failed after format: %d", rc);
                return -1;
            }
        }
    }

    LOG_INF("LittleFS mounted successfully at %s\n", lfs_storage_mnt.mnt_point);

    /* Step 3: Test filesystem */
    struct fs_file_t file;
    fs_file_t_init(&file);

    /* Read boot count */
    uint32_t boot_count = 0;
    rc = fs_open(&file, "/lfs/boot_count", FS_O_CREATE | FS_O_RDWR);
    if (rc < 0) {
        LOG_ERR("Failed to open boot_count file: %d", rc);
    } else {
        ssize_t bytes_read = fs_read(&file, &boot_count, sizeof(boot_count));
        if (bytes_read == sizeof(boot_count)) {
            LOG_INF("Previous boot count: %u", boot_count);
            boot_count++;
        } else {
            LOG_INF("First boot - initializing counter");
            boot_count = 1;
        }

        /* Write new count */
        fs_seek(&file, 0, FS_SEEK_SET);
        ssize_t written = fs_write(&file, &boot_count, sizeof(boot_count));
        if (written == sizeof(boot_count)) {
            LOG_INF("Boot count updated: %u", boot_count);
        } else {
            LOG_ERR("Failed to write boot count: %d", written);
        }

        fs_close(&file);
    }

    /* Write a message */
    rc = fs_open(&file, "/lfs/message.txt", FS_O_CREATE | FS_O_WRITE);
    if (rc == 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Hello from boot #%u!\n", boot_count);
        fs_write(&file, msg, strlen(msg));
        fs_close(&file);
        LOG_INF("Message written: %s", msg);
    }

    /* Display filesystem stats */
    struct fs_statvfs stat;
    rc = fs_statvfs("/lfs", &stat);
    if (rc == 0) {
        LOG_INF("\nFilesystem stats:");
        LOG_INF("  Block size: %lu bytes", stat.f_bsize);
        LOG_INF("  Total blocks: %lu", stat.f_blocks);
        LOG_INF("  Free blocks: %lu", stat.f_bfree);
        LOG_INF("  Total size: %lu bytes", stat.f_blocks * stat.f_bsize);
        LOG_INF("  Free size: %lu bytes", stat.f_bfree * stat.f_bsize);
    }

    LOG_INF("\n=== Test Complete ===");
    LOG_INF("Power cycle to see boot count increment!");

    return 0;
}