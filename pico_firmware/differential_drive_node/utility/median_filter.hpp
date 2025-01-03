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

#ifndef MedianFilter_h
#define MedianFilter_h

#include "pico/stdlib.h"
#include <memory>
#include <stdlib.h>
#include <string.h>

class median_filter {
  public:
    // constructor
    median_filter();
    median_filter(int filter_size);

    // method
    void in(float sample);
    float out();
    void reset(float sample);
    void setSize(int filter_size);
    int getSize();

  private:
    int mFilterSize;
    int mCurrentFilterSize;
    float *mBuffer;
};

#endif // MEDIANFILTER_HPP_INCLUDED