#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RADAR_MAX_AIRCRAFT 64

typedef enum {
    RADAR_CITY_MELBOURNE = 0,
    RADAR_CITY_TEHRAN,
    RADAR_CITY_COUNT,
} radar_city_t;

typedef struct {
    char icao24[7];
    char callsign[9];
    char origin_country[33];
    char squawk[5];
    double longitude;
    double latitude;
    float altitude_m;
    float velocity_mps;
    float track_deg;
    float vertical_rate_mps;
    uint8_t category;
    bool on_ground;
    int64_t received_us;
} radar_aircraft_t;

typedef struct {
    radar_aircraft_t items[RADAR_MAX_AIRCRAFT];
    size_t count;
} radar_aircraft_list_t;

typedef struct {
    float temperature_c;
    float high_c;
    float low_c;
    float uv_index_max;
    int weather_code;
    int32_t utc_offset_seconds;
    bool valid;
} radar_weather_t;

typedef struct {
    char icao24[7];
    char registration[16];
    char type[40];
    char icao_type[8];
    char manufacturer[32];
    char owner[40];
    char airline[40];
    char origin_icao[5];
    char origin_iata[4];
    char origin_name[48];
    char origin_city[32];
    char destination_icao[5];
    char destination_iata[4];
    char destination_name[48];
    char destination_city[32];
    bool aircraft_found;
    bool route_found;
} radar_aircraft_details_t;
