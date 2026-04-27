/*
 *   This file is part of embedded software pico playground project.
 *
 *   embedded software pico playground projec is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   embedded software pico playground project is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with embedded software pico playground project.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "mock_i2c.hpp"

int mock_i2c_read_blocking(mock_i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, uint8_t len, bool nostop) {
    // Mock behavior
    return 0;
}

int mock_i2c_write_blocking(mock_i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, uint8_t len, bool nostop) {
    // Mock behavior
    return 0;
}
