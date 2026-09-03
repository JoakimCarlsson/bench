#include "clock.h"

#ifdef _WIN32
#include <windows.h>

uint64_t now_ns(void) {
    static LARGE_INTEGER freq;
    LARGE_INTEGER t;
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t);
    return (uint64_t)((double)t.QuadPart * 1e9 / (double)freq.QuadPart);
}

#else
#include <time.h>

uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

#endif
