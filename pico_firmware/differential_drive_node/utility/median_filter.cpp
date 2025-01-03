/*
 *   This file is part of the astro project.
 *
 *   Astro project is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Astro project is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro project.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "median_filter.hpp"

median_filter::median_filter() {
    mFilterSize = 1;
    mCurrentFilterSize = 0;
    mBuffer = new float[mFilterSize];
}

median_filter::median_filter(int filter_size) {
    mFilterSize = filter_size;
    mCurrentFilterSize = 0;
    mBuffer = new float[mFilterSize];
}

void median_filter::setSize(int filter_size) {

    float *tmpBuffer = new float[filter_size];
    memcpy(tmpBuffer, mBuffer, mFilterSize * sizeof(float));
    // Saving new filter size
    mFilterSize = filter_size;

    delete[] mBuffer;

    mBuffer = tmpBuffer;
    mCurrentFilterSize = 0;
}

int median_filter::getSize() { return mFilterSize; }

int floatCompare(const void *a, const void *b) {
    if (*(const float *)a < *(const float *)b)
        return -1;
    return *(const float *)a > *(const float *)b;
}

void median_filter::in(float sample) {
    // feed buffer
    if (mCurrentFilterSize < mFilterSize) {
        mBuffer[mCurrentFilterSize] = sample;
        mCurrentFilterSize++;
    } else {
        // circular rotation
        for (int i = 0; i < mFilterSize - 1; i++) {
            mBuffer[i] = mBuffer[i + 1];
        }
        mBuffer[mFilterSize - 1] = sample;
    }
}

float median_filter::out() {
    if (mCurrentFilterSize == 0) {
        // printf("Buffer is empty , cannot calculate median\n");
        return 0.0;
    }

    // Use dynamic memory allocation to store sorted_buffer instead of relying on stack allocation as that could
    // result in stack overflow for large buffers
    float *sorted_buffer = new float[mCurrentFilterSize];
    memcpy(sorted_buffer, mBuffer, sizeof(mBuffer[0]) * mCurrentFilterSize);
    qsort(sorted_buffer, mCurrentFilterSize, sizeof(float), floatCompare);

    float median;
    if (mCurrentFilterSize % 2 == 0) {
        // Even case: average of two middle values
        median = (sorted_buffer[mCurrentFilterSize / 2 - 1] + sorted_buffer[mCurrentFilterSize / 2]) / 2.0f;
    } else {
        // Odd case: middle value
        median = sorted_buffer[mCurrentFilterSize / 2];
    }

    delete[] sorted_buffer; // Clean up dynamic memory
    return median;
}