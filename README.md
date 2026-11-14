# SD-CANbus-Logger

An ESP32-based, standalone CAN bus data logger with Wi-Fi connectivity, microSD storage, and a browser-based management interface.

**Project status:** Early development — V02 foundation firmware  
**Hardware target:** WeAct CAN485 V1.1 (ESP32-based)  
**Firmware framework:** Arduino-ESP32  
**Current development environment:** Arduino IDE 1.8.19, ESP32 Arduino Core 3.3.11

---

## 1. Project Overview

SD-CANbus-Logger is intended to provide a compact, self-contained platform for recording CAN bus traffic to a microSD card.

The project is being developed incrementally, with an emphasis on reliable hardware operation, diagnostic visibility, and recoverability before adding CAN logging functionality.

The initial development phase establishes the supporting infrastructure:

- Standalone Wi-Fi access point for configuration and recovery.
- Infrastructure Wi-Fi connectivity (AP + STA).
- Persistent Wi-Fi credentials stored in ESP32 non-volatile storage.
- Browser-based system dashboard.
- Serial command interface.
- microSD card initialization and read/write verification.
- System, memory, network, SD, and HTTP diagnostics.
- Diagnostic event history and boot memory checkpoints.
- mDNS hostname for convenient network access.

**Current limitation:** CAN/TWAI has not yet been initialized in the firmware. CAN reception and data logging are planned features, not currently implemented.

## 2. Hardware

### Target hardware

| Component | Description |
|---|---|
| Development board | WeAct CAN485 V1.1 |
| Microcontroller | ESP32-D0WD-V3 |
| Reported chip revision | 301 |
| CPU | Dual-core Xtensa LX6, 240 MHz |
| Flash | 8 MB reported physical capacity |
| Wireless | 2.4 GHz Wi-Fi |
| Bluetooth | ESP32 hardware capability; not currently used |
| CAN controller | ESP32 native TWAI peripheral |
| CAN transceiver | Onboard CAN hardware |
| Storage | Onboard microSD interface |
| USB | USB-C development/programming connection |

**Flash configuration note:** The current Arduino build configuration has specified a 4 MB flash layout, although the chip reports 8 MB physical flash. The partition configuration should be verified before changing flash allocation.

### Hardware pin assignments

The following assignments are used by the target board:

| Function | ESP32 GPIO |
|---|---:|
| SD chip select | GPIO 13 |
| SD SPI clock (SCK) | GPIO 14 |
| SD SPI MOSI | GPIO 15 |
| SD SPI MISO | GPIO 2 |
| CAN RX | GPIO 26 |
| CAN TX | GPIO 27 |

The SD interface currently uses SPI at 4 MHz.

CAN RX/TX assignments are documented for the planned TWAI implementation. They are not yet active in V02.

### SD card

The firmware uses the Arduino ESP32 `SD` library.

A 256 GB microSD card has been successfully tested. The firmware reported:

- Card capacity: approximately 244,016 MB
- Filesystem capacity: approximately 243,955 MB
- Filesystem usage after initialization: approximately 0.1 MB

The card was successfully mounted and passed repeated write, append, and readback tests.

**Important:** V02 does not provide an SD card formatting function. Use a computer to prepare the card with a compatible FAT filesystem before installation. Formatting support is planned for a future release.

## 3. Software Requirements

### Development environment

The current known-good development configuration is:

| Software | Version |
|---|---|
| Arduino IDE | 1.8.19 |
| ESP32 Arduino Core | 3.3.11 |
| ESP-IDF | 5.5.5 |
| Target board selection | ESP32 Dev Module |
| CPU frequency | 240 MHz |
| Flash mode | QIO |
| Flash frequency | 80 MHz |

The sketch currently uses standard ESP32 Arduino libraries:

- `WiFi.h`
- `WebServer.h`
- `FS.h`
- `SD.h`
- `SPI.h`
- `Preferences.h`
- `ESPmDNS.h`

No additional third-party library installation is currently required for the V02 foundation firmware.

## 4. Getting Started

### Hardware preparation

1. Obtain a compatible WeAct CAN485 V1.1 development board.
2. Prepare a microSD card using a compatible FAT filesystem.
3. Insert the microSD card into the board.
4. Connect the board to a computer using USB-C.
5. Open the Arduino IDE.

### Firmware installation

1. Clone or download this repository.
2. Open `SD-CANbus-Logger.ino` in Arduino IDE.
3. Select the ESP32 Dev Module board configuration.
4. Verify the configuration against the software requirements above.
5. Select the appropriate serial port.
6. Compile and upload the firmware.
7. Open the Serial Monitor at the baud rate configured in the sketch.

The firmware should initialize the access point, mount the SD card, start the HTTP server, and initialize mDNS.

### Initial Wi-Fi connection

The logger starts with its own Wi-Fi access point enabled.

| Setting | Value |
|---|---|
| SSID | `SD-CANbus-Logger` |
| Password | `CANlogger01` |
| Default AP address | `192.168.4.1` |

Connect a phone or computer to the access point and browse to:

`http://192.168.4.1/`

The access point remains enabled when infrastructure Wi-Fi is connected, providing a recovery interface if the router connection becomes unavailable.

### Connecting to infrastructure Wi-Fi

Use the Serial Monitor:

```text
wifi connect SSID|PASSWORD
```

For example:

```text
wifi connect MyNetwork|MyPassword
```

The firmware saves the credentials in ESP32 NVS and attempts to connect without requiring the access point to be disabled.

After a successful connection, the firmware reports the assigned DHCP address.

The browser interface can then be accessed through either:

- The assigned infrastructure IP address, such as `http://192.168.1.160/`
- The mDNS hostname: `http://sd-canbus-logger.local/`

The actual IP address will depend on the network and DHCP configuration.

**Security note:** Wi-Fi passwords are not printed in serial diagnostics. Do not publish credentials, private network information, or diagnostic output containing sensitive details when submitting issues.

## 5. Serial Command Interface

V02 provides the following serial commands:

| Command | Description |
|---|---|
| `help` | Display available commands |
| `status` | Print full system status |
| `events` | Display recent diagnostic event history |
| `checkpoints` | Display boot memory checkpoints |
| `sdtest` | Run SD write/append/readback verification |
| `wifi status` | Show AP and infrastructure Wi-Fi status |
| `wifi connect SSID\|PASSWORD` | Save credentials and connect |
| `wifi disconnect` | Disconnect infrastructure Wi-Fi but retain saved credentials |
| `wifi forget` | Erase saved infrastructure Wi-Fi credentials |

The AP remains enabled as a recovery interface.

### Example

```text
wifi status
status
sdtest
events
checkpoints
```

The `status` command reports system identity, runtime, memory, Wi-Fi, SD storage, HTTP server, main-loop timing, and CAN initialization state.

## 6. Browser Interface

The browser interface provides a convenient way to monitor and exercise the logger without requiring a serial connection.

V02 includes:

- System status dashboard.
- Wi-Fi AP and infrastructure connection information.
- SD card status and capacity.
- SD write/append/readback test control.
- Runtime and memory diagnostics.
- HTTP server statistics.
- Main-loop timing statistics.
- Diagnostic event history.
- Boot memory checkpoints.

The web interface communicates with the ESP32 through HTTP endpoints.

Known V02 endpoints include:

| Endpoint | Purpose |
|---|---|
| `/` | Main browser interface |
| `/api/status` | System status data |
| `/api/test` | SD test operation |
| `/api/events` | Diagnostic event history |

The browser interface periodically refreshes system information. The AP and infrastructure interfaces are intended to provide access to the same web server.

## 7. Diagnostics and Instrumentation

Diagnostic visibility is a core design objective. The project aims to make failures observable before adding more complex functionality.

### System identity

The status report includes:

- Project and firmware version.
- Build timestamp.
- Arduino Core and ESP-IDF versions.
- Chip model and revision.
- CPU frequency.
- Flash capacity.
- Sketch size and available application space.
- Uptime.
- Reset reason.

### Memory

The firmware reports:

- Free heap.
- Minimum free heap.
- Largest free heap block.
- Internal free heap.
- Heap fragmentation indicator.

The fragmentation indicator is calculated as:

`100 × (1 − largest free block / free heap)`

It is a diagnostic indicator, not a complete measure of allocator health.

### Wi-Fi

Reported information includes:

- AP and STA operating mode.
- AP SSID, IP, and client count.
- STA SSID, connection state, IP, and RSSI.
- MAC addresses.
- Connection attempts and disconnect count.
- Last disconnect reason.
- mDNS state and hostname.

### SD storage

Reported information includes:

- Mount status.
- Card type and capacity.
- Filesystem capacity, used space, and free space.
- Mount attempts and failures.
- SD test count and last result.
- Write and read operation counts and failures.
- Last, maximum, and average operation durations.

### HTTP and main loop

The firmware tracks HTTP requests, route counts, errors, 404 responses, and handler timing.

Main-loop instrumentation includes:

- Loop iteration count.
- Last and maximum loop duration.
- Last and maximum loop gap.

These measurements will become increasingly important when CAN reception and SD logging are introduced.

### Diagnostic events and checkpoints

The firmware maintains a bounded diagnostic event history in RAM.

Events record significant state transitions and failures rather than every periodic status update.

Boot memory checkpoints capture memory measurements at important initialization stages, allowing changes in resource consumption to be compared.

## 8. Current Firmware Status

### V02 — Foundation firmware

The following functionality has been implemented and exercised:

- [x] ESP32 startup and system identification.
- [x] Wi-Fi access point.
- [x] Infrastructure Wi-Fi connection.
- [x] Persistent infrastructure credentials using NVS.
- [x] AP + STA operating mode.
- [x] mDNS hostname.
- [x] Browser dashboard.
- [x] SD card initialization.
- [x] SD write/append/readback verification.
- [x] Serial command interface.
- [x] System and memory diagnostics.
- [x] Wi-Fi diagnostics.
- [x] SD operation statistics.
- [x] HTTP request instrumentation.
- [x] Main-loop timing instrumentation.
- [x] Diagnostic event history.
- [x] Boot memory checkpoints.

Not yet implemented:

- [ ] CAN/TWAI initialization.
- [ ] CAN frame reception.
- [ ] CAN bitrate configuration and detection.
- [ ] CAN data logging.
- [ ] Log file management and download.
- [ ] SD card formatting.

## 9. Feature Roadmap

The roadmap is intentionally incremental. Reliability and diagnostic coverage take priority over adding features quickly.

### Phase 1 — Foundation and reliability

- [x] Establish SD card and Wi-Fi operation.
- [x] Implement AP + STA connectivity.
- [x] Implement persistent Wi-Fi credentials.
- [x] Implement browser and serial diagnostics.
- [x] Establish SD write/read verification.
- [ ] Add serial command echo, with password masking.
- [ ] Improve HTTP error diagnostics and request history.
- [ ] Explain system health PASS/WARN/FAIL conditions.
- [ ] Validate long-duration operation and reconnection behavior.

### Phase 2 — SD card management

- [ ] Add SD card formatting capability.
- [ ] Detect unsupported or unformatted cards.
- [ ] Provide card health and filesystem diagnostics.
- [ ] Provide a browser-based file listing.
- [ ] Provide log file download.
- [ ] Provide controlled file deletion.
- [ ] Establish configurable log file naming and rollover.

**Formatting considerations:** Formatting is a destructive operation. It should require explicit confirmation through the browser or serial interface, clearly identify the target card, and report success or failure. It should never occur automatically during normal startup.

### Phase 3 — CAN interface

- [ ] Initialize the ESP32 native TWAI peripheral.
- [ ] Begin in listen-only mode.
- [ ] Verify CAN reception independently of SD logging.
- [ ] Add configurable CAN bitrates.
- [ ] Add bitrate auto-detection where practical.
- [ ] Report CAN bus state and error conditions.
- [ ] Track received frames, error frames, and bus-off events.
- [ ] Add CAN diagnostic information to the browser and serial interfaces.

Initial supported bitrate targets should include 125, 250, 500, and 1000 kbit/s, subject to validation.

### Phase 4 — CAN data logging

- [ ] Implement binary CAN frame logging.
- [ ] Introduce buffered writes to reduce SD access overhead.
- [ ] Track buffer utilization and high-water mark.
- [ ] Track dropped frames and buffer overflow.
- [ ] Track SD write failures and logging interruptions.
- [ ] Establish file rollover and session management.
- [ ] Validate sustained CAN reception and logging under load.

A key architectural requirement is that CAN reception must not block while SD writes or browser requests are being handled. Buffering and explicit overflow accounting should be included from the outset.

### Phase 5 — Operational features

- [ ] Browser-based CAN monitoring.
- [ ] CAN logging start/stop controls.
- [ ] Configurable logging settings.
- [ ] Session statistics and summaries.
- [ ] Exportable diagnostic reports.
- [ ] Improved recovery and fault reporting.
- [ ] Optional time synchronization for timestamped logs.

### Future possibilities

Potential future enhancements include:

- CAN filtering and configurable message selection.
- CAN database (DBC) support.
- Live message decoding.
- Additional network interfaces or protocols.
- Remote log retrieval.
- More advanced system health monitoring.

These are exploratory features and are not commitments for the next firmware release.

## 10. Contributing

Contributions, testing, and technical feedback are welcome.

Useful areas for contribution include:

- ESP32 TWAI implementation and testing.
- SD filesystem reliability and formatting.
- CAN buffering and binary logging.
- Browser interface improvements.
- Long-duration reliability testing.
- Diagnostic instrumentation.
- Documentation and hardware compatibility testing.

### Contribution guidelines

1. Keep changes focused and explain their purpose.
2. Preserve working functionality unless a change is intentional and documented.
3. Avoid blocking CAN reception with SD, network, or diagnostic operations.
4. Use bounded buffers and histories where practical.
5. Record failures and dropped data explicitly; do not silently discard errors.
6. Include hardware and software configuration when reporting test results.
7. Document new serial commands, browser endpoints, and configuration settings.

### Reporting issues

When reporting a problem, include:

- Firmware version and build timestamp.
- Board and microcontroller details.
- Arduino Core version.
- SD card make, capacity, and filesystem.
- Relevant serial output.
- Steps to reproduce the issue.
- Whether the problem occurs through serial, browser, or both.

Diagnostic output is particularly useful for identifying resource, timing, and connectivity problems.

## 11. Development Philosophy

The project follows a few guiding principles:

1. **Establish reliability before complexity.** Verify hardware and supporting infrastructure before introducing CAN logging.
2. **Instrument before optimizing.** Capture useful diagnostic data before making performance changes.
3. **Avoid blocking operations in time-critical paths.** CAN reception must remain independent of SD writes and network servicing.
4. **Make failures visible.** Track errors, timeouts, dropped frames, and resource limitations.
5. **Keep recovery available.** The local access point should remain available even when infrastructure Wi-Fi fails.
6. **Keep the implementation maintainable.** Prefer straightforward, well-documented code over unnecessary architectural complexity.

---

## 12. Repository Structure

The project is currently maintained as a single-sketch Arduino project:

```text
SD-CANbus-Logger/
└── Arduino/
    └── SD-CANbus-Logger/
        └── SD-CANbus-Logger.ino
```

The `.ino` file is the primary firmware source.

Git is used for source control and change history. Firmware version identifiers (V01, V02, etc.) identify development milestones; Git commits provide the authoritative record of source changes.

## License

MIT : MIT License

Copyright (c) 2026 J

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
“Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
