# DAEDALUS — CanSat Avionics Firmware

Embedded flight software for a CanSat (can-sized satellite) running on an **STM32H725RG** microcontroller, built with PlatformIO and the Arduino framework. The system operates under **FreeRTOS**, with all sensor reads, control loops, telemetry, and data logging executed as concurrent tasks.

---

## Hardware

| Component | Interface | Purpose |
|---|---|---|
| STM32H725RG (Cortex-M7) | — | Main MCU, 480 MHz, hardware FPU |
| ISM330DHCX (ISM6HG256X) | SPI | 6-axis IMU (accel + gyro) |
| BMP581 | SPI | Barometric altimeter #1 |
| MS5611 | I2C | Barometric altimeter #2 |
| BNO086 | I2C | AHRS / magnetometer (rotation vector) |
| u-blox M10S | I2C | GNSS (10 Hz, airborne-4g profile) |
| INA236 | I2C | Battery voltage and current monitor |
| XBee (64-bit API mode) | UART | RF telemetry downlink / uplink |
| SD card | SDMMC (4-bit) | Flight data logger |
| Servo A | PWM | Payload deployment mechanism |
| Servo B | PWM | Instrument deployment mechanism |
| Dynamixel servos (x3) | UART (half-duplex, 1 Mbaud) | Paraglider spool / steering |
| NeoPixel x2 | GPIO | Status LED |
| Camera trigger x2 | GPIO | External camera control |

---

## Firmware Architecture

### RTOS Task Map

All concurrent work is implemented as FreeRTOS tasks created in `UserThreads()`. Each task runs in a `hal::rtos::interval_loop` at a fixed period.

| Task | Priority | Interval | Description |
|---|---|---|---|
| `CB_EvalFSM` | Realtime | `RA_INTERVAL_FSM_EVAL` | Kalman predict + flight state machine |
| `CB_INSDeploy` | High | `RA_INTERVAL_FSM_EVAL` | Instrument deployment logic |
| `CB_ReadIMU` | High | `RA_INTERVAL_IMU_READING` | ISM330 accel/gyro → Kalman update |
| `CB_ReadAltimeter` | High | `RA_INTERVAL_ALTIMETER_READING` | BMP581 + MS5611 → Kalman update |
| `CB_ReadMAG` | AboveNormal | `RA_INTERVAL_MAG_READING` | BNO086 rotation vector + calibration |
| `CB_ReadGNSS` | AboveNormal | `RA_INTERVAL_GNSS_READING` | u-blox PVT → GPS Kalman update |
| `CB_ReadINA` | AboveNormal | `RA_INTERVAL_INA_READING` | INA236 battery voltage/current |
| `CB_Control` | Normal | `RA_INTERVAL_Controlling` | Guidance + paraglider servo PID |
| `CB_ConstructData` | Normal | `RA_INTERVAL_CONSTRUCT` | Build telemetry/SD strings |
| `CB_SDLogger` | Realtime+1 | Dynamic (state-dependent) | Write CSV rows to SD card |
| `CB_Transmit` | Normal | `RA_TX_INTERVAL_MS` | XBee RF downlink |
| `CB_ReceiveCommand` | Normal | 10 ms | XBee uplink command parser |
| `CB_RetainDeployment` | Normal | 100 ms | Re-assert servo deployment positions |
| `CB_EEPROMWrite` | Low | `RA_EEPROM_WRITE_INTERVAL` | Persist flight state to backup SRAM |
| `CB_NeoPixelBlink` | Normal | 20 ms | Rainbow status LED |
| `CB_AutoZeroAlt` | High | `RA_INTERVAL_AUTOZERO` | Ground altitude auto-zero |
| `CB_DebugLogger` | Normal | 1000 ms | USB-CDC debug print (compile-time optional) |

### Shared-Resource Mutexes

Fine-grained mutexes prevent concurrent bus access without blocking unrelated tasks:

| Mutex | Guards |
|---|---|
| `mtx_spi` | SPI1 bus (IMU, BMP581) |
| `mtx_i2c` | I2C4 bus (BNO086, GPS, INA236) |
| `mtx_sdio` | SDMMC1 peripheral |
| `mtx_uart` | XBee UART |
| `mtx_cdc` | USB-CDC serial |
| `mtx_buf` | `tx_buf` / `sd_buf` shared strings |
| `mtx_nav` | `NavState` struct (GPS + heading snapshot) |
| `mtx_kf` | All Kalman filters, `alt_agl`, `apogee_raw` |

I2C hung-bus detection is built into the MAG, GNSS, and INA tasks: if a single read takes more than 40 ms, two consecutive timeouts trigger `RecoverI2C4()`, which bit-bangs 9 SCL pulses to release any slave stuck mid-byte.

---

## Flight State Machine (FSM)

States are evaluated every `RA_INTERVAL_FSM_EVAL` ms in `EvalFSM()`, called from `CB_EvalFSM` after the Kalman predict step.

```
STARTUP → IDLE_SAFE → LAUNCH_PAD → ASCENT → APOGEE → DESCENT
                                                          |
                                                   PROBE_RELEASE
                                                          |
                                                PAYLOAD_RELEASE → LANDED
```

| State | Transition Condition |
|---|---|
| `STARTUP` | Immediate |
| `IDLE_SAFE` | Startup countdown (`RA_STARTUP_COUNTDOWN`) |
| `LAUNCH_PAD` | Armed + acceleration OR altitude threshold (sampler vote) |
| `ASCENT` | Filtered vertical velocity drops below `RA_APOGEE_VEL` (+ optional time guard) |
| `APOGEE` | Fixed 1.5 s delay |
| `DESCENT` | Fixed 1.5 s delay; computes main release altitude as 80% of apogee |
| `PROBE_RELEASE` | `alt_agl` falls below `RA_MAIN_ALT_COMPENSATED`; activates servo A |
| `PAYLOAD_RELEASE` | `alt_agl` falls below `RA_LANDED_ALT`; activates paraglider control |
| `LANDED` | Stops Dynamixels, waits 1 min for cameras, then buzzer ON |

All threshold comparisons use a statistical vote (`xcore::sampler_t`) with a configurable over/under ratio to reject transient sensor noise.

---

## Kalman Filters

Four independent Kalman filters run predict in `CB_EvalFSM` and update in the sensor tasks:

| Filter | States | Measurements |
|---|---|---|
| `filter_acc` | Net acceleration, jerk | Total body acceleration |
| `filter_alt` | Altitude AGL, vertical velocity | Barometric altitude |
| `filter_nav_n` | Latitude, north velocity, north accel | GPS latitude + N velocity |
| `filter_nav_e` | Longitude, east velocity, east accel | GPS longitude + E velocity |

The GPS east filter corrects its state-transition matrix on every fix using `cos(lat)` so the longitude coupling stays accurate across latitudes.

---

## Guidance and Control

During `PAYLOAD_RELEASE` (paraglider descent), `CB_Control` runs the full guidance loop at `RA_INTERVAL_Controlling`:

1. **Navigation snapshot** — locks `mtx_nav` briefly to copy the latest `NavState`.
2. **Yaw smoothing** — sin/cos EMA (alpha = 0.15, ~300 ms time constant) removes IMU heading noise before passing to the guidance algorithm. Physical spool offset (`SPOOL_PHYSICAL_OFFSET`) is applied before smoothing.
3. **Guidance** — `controller.guidance.update()` computes target spool angles from current GPS position, target waypoint, altitude AGL, and velocity components.
4. **Servo PID** — `controller.servo_pid_update()` drives three Dynamixel servos via half-duplex UART at 1 Mbaud.

A built-in kinematic simulator (`SimNav` namespace) can replace real GPS/IMU data for ground testing without hardware changes, activated via an uplink command.

---

## Instrument Deployment (INS)

`EvalINSDeploy()` (called from `CB_INSDeploy`) independently decides when to release the instrument package using redundant sensors:

- **Condition 1** — ToF sensor below `RA_INS_TOF_THRESHOLD` **AND** barometric AGL below `RA_INS_NEAR_THRESHOLD` (both sampler-voted).
- **Condition 2 (backup)** — Barometric AGL below `RA_INS_CRIT_THRESHOLD` (sampler-voted).

Either condition activates servo B and logs a dump of the last 50 altitude samples.

---

## Persistent State (EEPROM / Backup SRAM)

Flight state is saved to the STM32H725 **Backup SRAM** (`0x38800000`) via `EEPROM_Write()` at `RA_EEPROM_WRITE_INTERVAL`. The store is validated on each read by a magic number and CRC-8. On power-on or watchdog reset, `EEPROM_Read()` restores:

- FSM state
- Packet count
- UTC time string
- Servo positions (`pos_a`, `pos_b`)
- Altitude reference (`alt_ref`)
- Guidance tuning parameters (bearing EMA alpha, control EMA alpha, heading deadband)

This allows the flight to resume without losing context across brownouts or watchdog resets.

---

## Telemetry

Outbound packets are built by `ConstructString()` into `tx_buf` and transmitted over XBee every `RA_TX_INTERVAL_MS` ms. The SD card CSV header format is:

```
DDL, PACKET_COUNT, UTC, TIMESTAMP_EPOCH, MILLIS, STATE,
TEAM_ID, MISSION_TIME, MODE, ALTITUDE, TEMPERATURE, PRESSURE,
VOLTAGE, CURRENT, GYRO_R/P/Y, ACCEL_R/P/Y,
GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
VELOCITY_E, VELOCITY_N, CMD_ECHO, HEADING, ROLL, PITCH, YAW,
TOF, DEPLOY, ALT_REF, APOGEE_RAW, POS_A, POS_B, CPU_TEMP,
SERVO_1/2/3_ANGLE, SERVO_1/2/3_TARGET
```

Inbound commands arrive via the XBee AP=2 frame parser in `CB_ReceiveCommand` and are dispatched to `HandleCommand()`. Commands can also be sent over USB-CDC when `RA_USB_DEBUG_ENABLED` is set.

---

## Build Environments

Defined in [platformio.ini](platformio.ini). All environments target `genericSTM32H725RGVX`, use C++20, and enable the hardware FPU (`-mfpu=fpv5-d16 -mfloat-abi=hard`).

| Environment | Source directory | Notes |
|---|---|---|
| `MAIN` | `src/main/` | Primary CanSat flight firmware |
| `UAV` | `src/uav/` | UAV variant |
| `duty` | `src/test_duty/` | Duty-cycle / stress test |
| `test_path` | `src/test_path/` | Path / guidance unit test |
| `test_sd` | `src/test_sd/` | SD card write test |
| `test_unit` | `src/test_unit/` | Unit tests |
| `test_servoa` | `src/test_servoa/` | Servo A test |
| `test_servo_eeprom` | `src/test_servo_eeprom/` | Servo + EEPROM test |

---

## Key Source Files

| File | Role |
|---|---|
| [src/main/main.cpp](src/main/main.cpp) | Top-level: hardware init, RTOS tasks, FSM, sensor reads |
| `include/UserFSM.h` | FSM state definitions |
| `include/UserPins.h` | GPIO pin assignments |
| `include/UserSensors.h` | Sensor driver wrappers |
| `include/Controlling.h` | Guidance + servo PID controller |
| `include/custom_kalman.h` | Kalman filter type aliases and init |
| `include/XBeeDriver.h` | XBee frame builder / parser |
| `include/custom_EEPROM.h` | Backup SRAM store struct + CRC |
| `include/BNO086Cal.h` | BNO086 magnetometer calibration |
| `include/config/Main/UserConfig.h` | Mission-specific tuning constants |
