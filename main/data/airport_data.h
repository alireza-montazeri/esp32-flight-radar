#pragma once

#include <stddef.h>

typedef struct
{
    float latitude;
    float longitude;
    char code[5];
} radar_airport_t;

extern const radar_airport_t radar_airports[];
extern const size_t radar_airport_count;
