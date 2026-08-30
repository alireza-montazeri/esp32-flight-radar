#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RADAR_MAX_AIRCRAFT 64

typedef struct {
    char icao24[7];
    char callsign[9];
    double longitude;
    double latitude;
    float altitude_m;
    float velocity_mps;
    float track_deg;
    bool on_ground;
    int64_t received_us;
} radar_aircraft_t;

typedef struct {
    radar_aircraft_t items[RADAR_MAX_AIRCRAFT];
    size_t count;
} radar_aircraft_list_t;
