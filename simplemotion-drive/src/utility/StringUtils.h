/**
 * Project: Simple MotionV2 based drive controller for Carla
 *
 * @file </src/utility/StringUtils.h>
 *
 * @author Prajankya Sonar - <prajankya@gmail.com>
 *
 * MIT License
 * Copyright (c) 2020 Prajankya Sonar
 */

#ifndef __smv2_STRINGUTILS_H__
#define __smv2_STRINGUTILS_H__

typedef unsigned value_type;

#include <vector>

template <typename Iterator>
inline size_t get_length(Iterator p) {
    unsigned char c = static_cast<unsigned char>(*p);
    if (c < 0x80)
        return 1;
    else if (!(c & 0x20))
        return 2;
    else if (!(c & 0x10))
        return 3;
    else if (!(c & 0x08))
        return 4;
    else if (!(c & 0x04))
        return 5;
    else
        return 6;
}

template <typename Iterator>
inline value_type get_value(Iterator p) {
    size_t len = get_length(p);

    if (len == 1) return *p;

    value_type res = static_cast<unsigned char>(*p & (0xff >> (len + 1)))
                     << ((len - 1) * 6);

    for (--len; len; --len)
        res |= (static_cast<unsigned char>(*(++p)) - 0x80) << ((len - 1) * 6);

    return res;
}

inline std::string utf2Latin1(std::string s_utf8) {
    std::vector<char> s_latin1;
    for (std::string::iterator p = s_utf8.begin(); p != s_utf8.end(); ++p) {
        value_type value = get_value<std::string::iterator&>(p);
        if (value > 0xff) throw "Cannot convert string to Latin1!";
        s_latin1.push_back(static_cast<char>(value));
    }

    return std::string(s_latin1.begin(), s_latin1.end());
}

#endif