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

#ifndef STORAGE_HPP
#define STORAGE_HPP

class Storage {
   public:
    int init();
    int list_directory(const char* path);
    int read_and_display_file(const char* filepath);
    int print_storage_stats();

   private:
    int read_boot_count(int* count);
    int write_boot_count(int count);
    int append_boot_log(int boot_num);
    void sample_fix_code_triple(struct usbd_context* uds_ctx, const enum usbd_speed speed);
};

#endif /* STORAGE_HPP */