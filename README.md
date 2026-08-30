# ESP32 Flight Radar

An Internet-connected aircraft radar display for the Waveshare
[ESP32-S3-Knob-Touch-LCD-1.8](https://www.waveshare.com/wiki/ESP32-S3-Knob-Touch-LCD-1.8),
built with ESP-IDF and live data from the
[OpenSky Network REST API](https://openskynetwork.github.io/opensky-api/rest.html).

The application uses the board's 360 x 360 SH8601 QSPI display, capacitive
touch panel, rotary encoder, and DRV2605 vibration motor. It is an Internet
flight tracker, not a 1090 MHz ADS-B radio receiver. A completely local receiver
would also require an SDR or dedicated ADS-B RF front end and antenna.

## Features

- Live OpenSky aircraft positions inside a configurable bounding box
- Nearby scheduled-service airport markers from a bundled offline database
- Simplified global coastlines from a bundled offline vector map
- Aircraft heading, callsign or ICAO address, and altitude
- Position prediction between API updates
- First-boot Wi-Fi, location, display, and OpenSky setup page
- Configuration stored persistently in ESP32 NVS
- Anonymous OpenSky access or automatic OAuth2 client-credential authentication
- Knob-controlled radar radius from 0.05 to 2.5 degrees
- DRV2605 tactile click feedback while turning the knob
- Tap an aircraft for live flight details; tap again to close
- Knob selection between visible aircraft while the detail card is open
- On-demand airline/company and route enrichment
- Animated radar sweep
- Local configuration page at `http://flight-radar.local/`

## Hardware

Required:

- Waveshare ESP32-S3-Knob-Touch-LCD-1.8
- A USB-C data cable
- A 2.4 GHz Wi-Fi network with Internet access
- Windows, Linux, or macOS computer for building and flashing

No external display, touch, encoder, or vibration-motor wiring is required.
The verified onboard connections are:

| Function              | Connection                                           |
| --------------------- | ---------------------------------------------------- |
| LCD QSPI clock        | GPIO 13                                              |
| LCD chip select       | GPIO 14                                              |
| LCD data 0-3          | GPIO 15, 16, 17, 18                                  |
| LCD reset             | GPIO 21                                              |
| LCD backlight PWM     | GPIO 47                                              |
| I2C SDA               | GPIO 11                                              |
| I2C SCL               | GPIO 12                                              |
| Touch controller      | I2C address `0x15`                                   |
| DRV2605 haptic driver | I2C address `0x5A`                                   |
| Encoder A/B           | GPIO 8 and GPIO 7                                    |
| Touch reset/interrupt | GPIO 10 and GPIO 9, reserved by the board definition |

The authoritative project pin definitions are in
[`main/board/user_config.h`](main/board/user_config.h), and the supplied board
schematics are retained under [`Documents/schematics`](Documents/schematics).

## Software prerequisites

This project is tested with **ESP-IDF 6.0.2**. Install and activate that version
before building. Espressif's
[ESP32-S3 getting-started guide](https://docs.espressif.com/projects/esp-idf/en/release-v6.0/esp32s3/get-started/index.html)
describes installation for Windows, Linux, and macOS.

On Windows, open the ESP-IDF terminal installed by Espressif rather than an
ordinary PowerShell window. You can confirm the environment with:

```powershell
idf.py --version
```

The project path and ESP-IDF installation path should not contain spaces.

## Clone and build

```powershell
git clone https://github.com/alireza-montazeri/esp32-flight-radar
cd esp32-flight-radar
idf.py build
```

The committed [`sdkconfig`](sdkconfig) is the exact known-working configuration
for this board. [`sdkconfig.defaults`](sdkconfig.defaults) retains the important
board-specific defaults for regenerating it. If `sdkconfig` is intentionally
removed, regenerate it with:

```powershell
idf.py set-target esp32s3
idf.py build
```

During configuration, ESP-IDF automatically reads
[`main/idf_component.yml`](main/idf_component.yml), resolves
[`dependencies.lock`](dependencies.lock), and downloads LVGL, cJSON, mDNS, and
the SH8601 driver into the ignored `managed_components/` directory. The
[ESP-IDF Component Manager](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/idf-component-manager.html)
performs this automatically; do not copy or commit `managed_components/`.

The board-specific I2C, touch, backlight, and encoder sources under
[`components/`](components) are local components and must remain in the
repository because they describe this board's wiring and external devices.

A successful build produces:

- `build/flight-radar.bin`
- `build/bootloader/bootloader.bin`
- `build/partition_table/partition-table.bin`

The custom [`partitions.csv`](partitions.csv) reserves 6 MB for the factory
application while retaining NVS and PHY partitions.

## Find the ESP32-S3 serial port

On Windows, inspect **Device Manager > Ports (COM & LPT)** before and after
connecting the board. Use the newly appearing ESP32-S3 port, such as `COM6`.

This board can expose a different ESP chip depending on the USB-C plug
orientation at the board. If flashing reports:

```text
This chip is ESP32, not ESP32-S3. Wrong chip argument?
```

unplug the cable, rotate the USB-C plug 180 degrees at the board, reconnect it,
and select the new COM port. A correct ESP32-S3 boot log begins with something
similar to:

```text
ESP-ROM:esp32s3
```

## Flash and monitor

Replace `COM6` with the ESP32-S3 port found on your computer:

```powershell
idf.py -p COM6 flash monitor
```

`flash` builds if necessary, writes the bootloader, partition table, and
application, then resets the board. `monitor` opens the serial log at 115200
baud. Exit the monitor with **Ctrl+]**. These are the standard
[ESP-IDF build, flash, and monitor commands](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/start-project.html).

If the board remains at:

```text
waiting for download
```

release the BOOT button and press RESET, or disconnect and reconnect power
without holding BOOT.

## First-boot Wi-Fi setup

When no Wi-Fi credentials are stored, the screen displays:

```text
Join FlightRadar-Setup - 192.168.4.1
```

1. On a phone or computer, connect to the open Wi-Fi network
   **FlightRadar-Setup**.
2. The network intentionally has no Internet access. If the device tries to
   leave it automatically, choose the option to remain connected.
3. Disable a VPN temporarily if it prevents access to local addresses.
4. Open **`http://192.168.4.1/`** in a browser. Use `http://`, not
   `https://`.
5. Complete the configuration form and select **Save and restart**.

If the page does not open, verify that the phone or computer is actually
connected to `FlightRadar-Setup`. Traffic for `192.168.4.1` must go through
that Wi-Fi connection rather than Ethernet, cellular data, or a VPN.

### Configuration fields

| Field               | What to enter                                                       |
| ------------------- | ------------------------------------------------------------------- |
| Wi-Fi SSID          | Exact name of the 2.4 GHz network                                   |
| Wi-Fi password      | Network password; blank retains an already stored password          |
| Latitude            | Radar centre in signed decimal degrees, from -90 to 90              |
| Longitude           | Radar centre in signed decimal degrees, from -180 to 180            |
| Radius              | 0.05 to 2.5 degrees                                                 |
| Animated sweep      | Enables the rotating, fading green radar beam                       |
| Aircraft labels     | Shows the callsign or ICAO24 address                                |
| Airports            | Shows bundled airport plus markers and codes                        |
| Coastlines          | Shows the bundled Natural Earth coastline layer                     |
| Grounded aircraft   | Includes aircraft currently reported on the ground                  |
| OAuth client ID     | Optional OpenSky API client ID                                      |
| OAuth client secret | Optional OpenSky API client secret; blank retains the stored secret |

The board does not obtain its location automatically. Enter the decimal-degree
coordinates for the centre of the area you want to observe. South and west
coordinates are negative. Do not commit your actual coordinates to the
repository; enter them only through the device's setup page, where they are
stored in NVS flash.

A radius of 0.05 degrees is approximately 5.6 km north/south. East/west distance
varies with latitude and is approximately `5.6 × cos(latitude)` km. The default
radius of 0.75 degrees is approximately 83 km north/south. Degrees remain the
stored source of truth; the on-screen scale converts the radius to kilometres
and rounds it to a whole number. The scale is measured from the radar centre to
the outer ring, so the full edge-to-edge diameter is twice the displayed value.

After saving, the ESP32 restarts and attempts to join the configured Wi-Fi
network. If it cannot connect within approximately 20 seconds, it enables
`FlightRadar-Setup` again so the settings can be corrected.

## OpenSky account and OAuth credentials

OpenSky credentials are optional. Leaving both OAuth fields empty uses anonymous
access.

To enable authenticated access:

1. Create or sign in to an account at
   [OpenSky Network](https://opensky-network.org/).
2. Open the [OpenSky account page](https://opensky-network.org/my-opensky/account).
3. Find the **API Client** section and create an API client.
4. Copy the generated `client_id` into **OAuth client ID** on the radar setup
   page.
5. Copy the generated `client_secret` into **OAuth client secret**.
6. Save the form and allow the radar to restart.

Do not enter the normal OpenSky website username and password. OpenSky's REST
API exclusively uses the OAuth2 client-credentials flow. The firmware exchanges
the client ID and secret for a bearer token and refreshes it automatically; you
do not need to obtain or paste an access token manually. See the official
[OpenSky authentication documentation](https://openskynetwork.github.io/opensky-api/rest.html#authentication).

The firmware's request schedule is designed around OpenSky's documented credit
allowances:

| Mode                        | Firmware interval | OpenSky daily allowance |
| --------------------------- | ----------------: | ----------------------: |
| Anonymous                   |       220 seconds |             400 credits |
| Standard authenticated user |        60 seconds |           4,000 credits |

The maximum configured bounding box is 5 by 5 degrees, or 25 square degrees, so
each `/states/all` request costs one credit under the current
[OpenSky API credit rules](https://openskynetwork.github.io/opensky-api/rest.html#api-credits).
If OAuth authentication fails, the firmware logs a warning and continues using
anonymous access.

## Normal operation

After Wi-Fi connection, the radar fetches nearby aircraft and shows:

- Dim blue coastline vectors clipped to the circular radar range
- White plus airport markers with white IATA/ICAO labels inside the radar circle
- Green aircraft symbols for airborne aircraft
- Optional amber symbols for aircraft reported by OpenSky as on the ground
- GPS-style aircraft symbols rotated to the nearest of 16 heading directions
- Callsign, or ICAO24 address when no callsign is available
- OpenSky-native units: altitude in metres and speed/vertical rate in metres per second
- Icon-based aircraft count, selected range, connection state, and update status

Opening the detail card starts a best-effort lookup through the public
[ADSBDB API](https://github.com/mrjackwills/adsbdb). No additional API key is
required. The card shows the selected aircraft number out of the visible total,
flight/callsign, altitude with vertical speed, speed, and heading with a compass
direction. When available, ADSBDB also adds the airline or owner and separate
aircraft manufacturer and type below it, followed by origin and destination
entries on one line containing each airport code and city name. After a
completed lookup with no company, aircraft information, or route result, the
card displays `No additional detail found`.
The lookup sends only the selected aircraft's public ICAO24 address and
callsign. Results are cached in RAM for the current session.

Aircraft and route metadata may be incomplete or incorrect. Routes are inferred
from callsigns and are commonly unavailable for private, charter, military, or
callsign-changing flights. A failed enrichment lookup does not interrupt live
OpenSky tracking.

Airports are enabled by default. Coastlines and grounded aircraft are disabled
by default. The grounded-aircraft option reflects OpenSky's current
`on_ground` state; it does not display historical flights that have already
disappeared from the live response.

HTTPS requests are serialized to limit ESP32 memory pressure. OpenSky requests
have a 20-second total deadline and failed updates retry after 10 seconds.
Optional ADSBDB enrichment has an 8-second deadline.

### External API calls

The firmware calls only OpenSky and ADSBDB. The exact external requests are:

| Provider | Method | Endpoint | Purpose |
| -------- | ------ | -------- | ------- |
| OpenSky | `POST` | `https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token` | Obtain an OAuth access token when a client ID and secret are configured |
| OpenSky | `GET` | `https://opensky-network.org/api/states/all?lamin={south}&lamax={north}&lomin={west}&lomax={east}&extended=1` | Fetch live aircraft state vectors inside the configured radar area |
| ADSBDB | `GET` | `https://api.adsbdb.com/v0/aircraft/{ICAO24}?callsign={CALLSIGN}` | Look up optional aircraft, airline, origin, and destination metadata for the selected aircraft |

If a callsign is unavailable, the ADSBDB request omits the query string and uses
`https://api.adsbdb.com/v0/aircraft/{ICAO24}`. ADSBDB does not require a key.
OpenSky OAuth is skipped when no credentials are configured; the live states
request is then anonymous. The local setup page and mDNS service are hosted by
the ESP32 and are not external API calls.

### Airport data

Nearby airports do not require an API call. The firmware bundles a compact list
of airports with scheduled service, generated from the public-domain
[OurAirports dataset](https://ourairports.com/data/). Airport coordinates remain
in degrees and use the same circular projection as aircraft. Every scheduled
airport inside the radar circle is drawn with a compact white code; labels are
intentionally allowed to overlap in dense areas.

To refresh the bundled dataset from the latest `airports.csv`, run from the
repository root:

```powershell
python tools/generate_airport_data.py
```

The generated `main/data/airport_data.c` is committed with the project, so this
command is not required for a normal or offline firmware build. You can also
regenerate from a downloaded CSV with
`python tools/generate_airport_data.py --source path/to/airports.csv`.

### Coastline data

Coastlines are bundled vector data and do not add a runtime API request. They
are generated from
[Natural Earth's public-domain 1:50m coastline dataset](https://www.naturalearthdata.com/downloads/50m-physical-vectors/),
stored as quantized degree coordinates, and clipped to the circular radar
range on the device. Projected segments are cached in PSRAM and recalculated
only when the radar centre or radius changes.

To refresh the bundled coastline data, run:

```powershell
python tools/generate_coastline_data.py
```

The generated `main/data/coastline_data.c` is committed, so an ordinary build
does not require Python or internet access. A downloaded Natural Earth GeoJSON
file can be supplied with
`python tools/generate_coastline_data.py --source path/to/coastline.geojson`.

Controls:

| Control                    | Action                                            |
| -------------------------- | ------------------------------------------------- |
| Rotate knob                | Increase or decrease radar radius                 |
| Knob vibration             | Confirms a detected rotation step                 |
| Tap an aircraft            | Open its live detail card                         |
| Rotate knob with card open | Select the previous or next visible aircraft      |
| Tap while card is open     | Close the detail card                             |

The selected knob radius is written to NVS after the knob has been idle for
approximately 1.5 seconds. Opening and closing the detail card does not change
the configured aircraft-label setting.

Between OpenSky responses, the firmware projects aircraft using their reported
velocity and track. A projected position is committed to the visible radar only
when the sweep's leading edge crosses that aircraft's bearing. The icon then
remains fixed until the next sweep, recreating a traditional scanned-radar
display without presenting the projection as a continuous live measurement.

## Reopen or change configuration

While the radar is connected to the normal Wi-Fi network, browse to:

```text
http://flight-radar.local/
```

The phone or computer must be on the same local network. If the `.local`
address is not supported by the router or client, find the radar's IP address in
the router's connected-device list and open:

```text
http://RADAR_IP_ADDRESS/
```

Blank Wi-Fi password and OAuth secret fields retain their existing stored
values. Saving any changes restarts the radar.

Normal firmware flashing does not erase NVS, so saved settings ordinarily
survive a firmware update.

## Reset all stored settings

To erase Wi-Fi, location, OpenSky credentials, and the installed firmware:

```powershell
idf.py -p COM6 erase-flash
```

Then reinstall the application:

```powershell
idf.py -p COM6 flash monitor
```

This is destructive: `erase-flash` clears the entire ESP32-S3 flash, including
NVS.

## Troubleshooting

### Wrong-chip error

```text
This chip is ESP32, not ESP32-S3
```

Use the other USB-C orientation at the board and select the newly appearing
ESP32-S3 COM port.

### Setup page does not open

- Connect specifically to `FlightRadar-Setup`.
- Use `http://192.168.4.1/`, not HTTPS.
- Ignore the phone or computer's “no Internet” warning.
- Temporarily disable VPN, cellular fallback, or another active network route.

### Radar connects but shows no aircraft

- Confirm the configured latitude and longitude signs.
- Increase the radius with the knob.
- Confirm that the Wi-Fi network has Internet access.
- OpenSky coverage varies by region and aircraft altitude.
- Watch the serial monitor for the HTTP status and error name.

### Company or route is unavailable

- Wait briefly after selecting the aircraft; lookup starts after a short knob
  selection debounce.
- Confirm the network permits HTTPS access to `api.adsbdb.com`.
- Some aircraft are not present in the metadata database, and some callsigns do
  not map to a known route.
- Core position, altitude, speed, and heading data still come from OpenSky and
  remain available when enrichment fails.

### OpenSky HTTP 401

The client ID or secret is invalid, or token acquisition failed. Reopen the
configuration page and copy the API-client credentials again. Do not use the
normal account password.

### OpenSky HTTP 429

The OpenSky credit allowance has been exhausted. Wait for the allowance to
refill. Avoid modifying the firmware to poll more frequently than the configured
interval.

### Haptic feedback is absent

Check the monitor for:

```text
radar_haptics: DRV2605 ready (effect 5: sharp click 60%)
```

A “DRV2605 not detected” warning indicates an I2C/device initialization problem;
the rest of the radar continues running without haptics.

### Rebuild downloaded dependencies

```powershell
idf.py fullclean
idf.py build
```

`fullclean` removes generated build state. ESP-IDF then restores managed
components from the manifest and lock file during the next build.

## Configuration and generated files

| Path                       | Version-control policy                      |
| -------------------------- | ------------------------------------------- |
| `sdkconfig`                | Commit: exact known-working configuration   |
| `sdkconfig.defaults`       | Commit: intentional board defaults          |
| `sdkconfig.old`            | Ignore: local backup                        |
| `dependencies.lock`        | Commit: resolved managed-component versions |
| `managed_components/`      | Ignore: downloaded automatically            |
| `build/`                   | Ignore: generated output                    |
| `components/`              | Commit: local Waveshare board support       |
| `Documents/schematics/`    | Commit: hardware reference                  |
| Other `Documents/` content | Ignore: not required to build               |

## Project layout

- `main/main.c` - minimal ESP-IDF entry point
- `main/app` - startup orchestration, knob control, and radar polling
- `main/board` - verified pins and Waveshare SH8601/LVGL display port
- `main/config` - persistent NVS configuration
- `main/data` - generated offline airport and coastline coordinates
- `main/hardware` - DRV2605 haptic control
- `main/model` - shared aircraft data structures
- `main/network` - Wi-Fi, web setup, HTTPS, OAuth, and OpenSky parsing
- `main/ui` - LVGL radar renderer
- `tools` - optional airport-data regeneration utility
- `components` - local I2C, touch, backlight, and encoder components
- `Documents/schematics` - board schematics

## Security and limitations

- `FlightRadar-Setup` is intentionally an open access point.
- The setup page uses plain HTTP on the local network.
- Wi-Fi and OpenSky credentials are stored in ordinary, unencrypted NVS.
- OpenSky traffic uses HTTPS and ESP-IDF's certificate bundle.
- At most 64 aircraft are rendered.
- Aircraft without a reported latitude or longitude are ignored.
- OpenSky coverage and update timing determine what appears on screen.
- Coastlines are simplified visual context and must not be used for navigation.
- This project does not directly receive ADS-B radio transmissions.
