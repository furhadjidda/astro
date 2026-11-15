
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>

#define RAD_TO_DEG 57.29578

/*!
    @brief Checks whether a string starts with a specified prefix
    @param str Pointer to a string
    @param prefix Pointer to the prefix
    @return True if str starts with prefix, false otherwise
*/
static bool string_starts_with(const char* str, const char* prefix) {
    while (*prefix) {
        if (*prefix++ != *str++) return false;
    }
    return true;
}

#endif