#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <time.h>

#include "./logger.h"
#include "./../localsdk/init.h"
#include "./../configs/configs.h"

// Is a message of this level written? (negative levels are always written)
static bool logger_enabled(const int level) {
    return level < 0 || level <= APP_CFG.logger.level;
}

// Write message to log
static int logger_write(const int level, const char *format, ...) {
    int result = 0;
    if(logger_enabled(level)) {
        va_list params, params_copy;
        va_start(params, format);
        va_copy(params_copy, params); // a va_list can only be consumed once
        result = vprintf(format, params);
        // stdout is redirected to a file and fully buffered: flush, or the last
        // lines before a crash (the useful ones) are lost
        fflush(stdout);
        // Write to file
        if(APP_CFG.logger.file && APP_CFG.logger.file[0]) {
            FILE *file;
            if((file = fopen(APP_CFG.logger.file, "a"))) {
                vfprintf(file, format, params_copy);
                fclose(file);
            }
        }
        va_end(params_copy);
        va_end(params);
    }
    return result;
}

// Add message to log
int logger(const char *file, const int line, const char *function, const int level, char *format, ...) {
    // Filtered messages cost nothing: no formatting, no allocation
    if(!logger_enabled(level)) return 0;

    // Trim non-printable symbols from the format (stack buffer: the logger must
    // not allocate, it is called from every thread including under memory pressure)
    char frmt_buffer[512];
    const char *frmt = format;
    size_t frmt_length = strlen(format);
    if(frmt_length < sizeof(frmt_buffer)) {
        size_t j = 0;
        for(size_t i = 0; i < frmt_length; i++) {
            if(isprint((unsigned char) format[i])) frmt_buffer[j++] = format[i];
        }
        frmt_buffer[j] = '\0';
        frmt = frmt_buffer;
    }
    // Date & Time
    time_t rawtime;
    struct tm timeinfo;
    char datetime[18];
    memset(datetime, '\0', sizeof(datetime));
    if(time(&rawtime) != -1 && localtime_r(&rawtime, &timeinfo)) {
        strftime(datetime, sizeof(datetime), "%x %X", &timeinfo);
    }
    // Nanoseconds
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    // Print message (longer than the buffer: truncated)
    char message[1536];
    va_list params;
    va_start(params, format);
    vsnprintf(message, sizeof(message), frmt, params);
    va_end(params);
    return logger_write(level, "[%s.%09ld] [%s:%d][%s]: %s\n", datetime, (long) spec.tv_nsec, file, line, function, message);
}

// Hexdump of memory
int logger_memory(const char *description, const int level, const void *address, const int length, int width) {
    // https://stackoverflow.com/a/7776146
    int result = 0;
    // Check length
    if(length <= 0) {
        result += logger_write(level, "[%s][%s]: incorrect length = %d!\n", "logger", "logger_memory", length);
        return result;
    }
    // Check width
    if(width < 4 || width > 64) width = 16;
    // Output description if given
    if(description != NULL) {
        result += logger_write(level, "%s:\n", description);
    }
    // Process every byte in the data
    int i;
    unsigned char buffer[width+1];
    const unsigned char *chars = (const unsigned char *) address;
    for(i=0;i<length;i++) {
        // Multiple of perLine means new or first line (with line offset)
        if((i % width) == 0) {
            // Only print previous-line ASCII buffer for lines beyond first
            if(i != 0) {
                result += logger_write(level, "  %s\n", buffer);
            }
            // Output the offset of current line
            result += logger_write(level, "  %04x ", i);
        }
        // Now the hex code for the specific character
        result += logger_write(level, " %02x", chars[i]);
        // And buffer a printable ASCII character for later
        if((chars[i] < 0x20) || (chars[i] > 0x7e)) {
            buffer[i % width] = '.';
        } else {
            buffer[i % width] = chars[i];
        }
        buffer[(i % width) + 1] = '\0';
    }
    // Pad out last line if not exactly perLine characters
    while((i % width) != 0) {
        result += logger_write(level, "   ");
        i++;
    }
    // And print the final ASCII buffer
    result += logger_write(level, "  %s\n", buffer);
    return result;
}
