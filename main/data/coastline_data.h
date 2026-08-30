#pragma once

#include <stddef.h>
#include <stdint.h>

#define RADAR_GEO_COORDINATE_SCALE 10000.0

typedef struct
{
    int32_t latitude;
    int32_t longitude;
} radar_geo_point_t;

typedef struct
{
    uint32_t first_point;
    uint32_t point_count;
} radar_coastline_path_t;

extern const radar_geo_point_t radar_coastline_points[];
extern const size_t radar_coastline_point_count;
extern const radar_coastline_path_t radar_coastline_paths[];
extern const size_t radar_coastline_path_count;
