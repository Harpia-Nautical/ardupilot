/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_MaxBotixSerial.h"

#if AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

#define MAXBOTIX_OUT_OF_RANGE_5M 5000   // 5-meter models report 5000 for no target
#define MAXBOTIX_OUT_OF_RANGE_10M 9999  // 10-meter models report 9999 for no target
#define MAXBOTIX_OUT_OF_RANGE_ADD 1.0f  // meters to add when out of range

extern const AP_HAL::HAL& hal;

// read - return last value measured by sensor
// Protocol: "R####\r" where #### is range in millimeters (4 ASCII digits)
bool AP_RangeFinder_MaxBotixSerial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    int32_t sum = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // Read available data from UART
    for (auto i = 0; i < 8192; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }

        // State machine to parse "R####\r" protocol
        if (linebuf_len == 0) {
            // Looking for 'R' header
            if (c == 'R') {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len >= 1 && linebuf_len <= 4) {
            // Expecting 4 digit characters
            if (isdigit(c)) {
                linebuf[linebuf_len++] = c;
            } else {
                // Invalid character, reset
                linebuf_len = 0;
            }
        } else if (linebuf_len == 5) {
            // Expecting carriage return
            if (c == '\r') {
                // Complete packet received, parse it
                linebuf[linebuf_len] = '\0';

                // Extract the 4-digit range value (skip 'R')
                int32_t range_mm = 0;
                for (uint8_t j = 1; j <= 4; j++) {
                    range_mm = range_mm * 10 + (linebuf[j] - '0');
                }

                // Check for out-of-range values
                if (range_mm == MAXBOTIX_OUT_OF_RANGE_5M ||
                    range_mm == MAXBOTIX_OUT_OF_RANGE_10M) {
                    count_out_of_range++;
                } else {
                    sum += range_mm;
                    count++;
                }
            }
            // Reset buffer for next reading
            linebuf_len = 0;
        } else {
            // Buffer overflow protection
            linebuf_len = 0;
        }
    }

    if (count > 0) {
        // Return average distance of valid readings in meters
        reading_m = (float(sum) / count) * 0.001f;  // mm to meters
        return true;
    }

    if (count_out_of_range > 0) {
        // No target detected - return max range + 1m
        reading_m = max_distance() + MAXBOTIX_OUT_OF_RANGE_ADD;
        return true;
    }

    // No readings available
    return false;
}

#endif  // AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
