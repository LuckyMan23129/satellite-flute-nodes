# Iridium Satellite Modem — Zephyr Application

* Author: Van-Vu Bui
* Organization: KU Leuven & Quynhon University
* Time: 07/03/2026

Zephyr RTOS firmware for the **CircuitDojo Feather nRF9160** that measures water level with a **LiDAR Lite v3HP** sensor, manages energy with the **AsTAR++ adaptive scheduling algorithm**, and transmits data over the **Iridium 9603 SBD** satellite network.

---

## Project Structure

```
2.lid_sat/
├── src/
│   ├── main.c                      # Entry point: init, scheduling loop, frame build, transmit
│   ├── iridium_uart.c              # UART driver + full Iridium AT command API
│   ├── astar_diurnal.c             # AsTAR++ energy-adaptive task scheduling algorithm
│   ├── i2c_sensor4.c               # LiDAR Lite v3HP I2C driver (water level sensing)
│   ├── lowpower.c                  # Low-power GPIO, UART, and accelerometer management
│   ├── opencircuit.c               # Solar panel open-circuit switching logic
│   ├── read_Vcap.c                 # ADC read for capacitor voltage
│   ├── read_Vpv.c                  # ADC read for solar panel voltage
│   ├── read_Vsupply.c              # ADC read for supply voltage
│   ├── switch_opencircuit.c        # GPIO control for open-circuit switch
│   ├── switch_peripheral_power.c   # GPIO control for peripheral power rail
│   └── switch_Vpv_divider.c        # GPIO control for Vpv voltage divider
├── include/
│   ├── iridium_uart.h              # Public API, constants, result codes, iridium_t struct
│   ├── astar_diurnal.h             # AsTAR++ config struct and public API
│   ├── i2c_sensor4.h               # LiDAR Lite v3HP declarations
│   ├── lowpower.h                  # Low-power function declarations
│   ├── opencircuit.h               # Charging enable/disable declarations
│   ├── read_Vcap.h                 # read_Vcap_mv() declaration
│   ├── read_Vpv.h                  # update_Vpv() declaration
│   ├── read_Vsupply.h              # read_Vsupp_mv() declaration
│   ├── switch_opencircuit.h        # check_gpio_sw_opencircuit() declaration
│   ├── switch_peripheral_power.h   # periph_sw_turn_on/off() declarations
│   ├── switch_Vpv_divider.h        # Vpv divider GPIO declarations
│   └── config.h                    # Global runtime flag (ENABLE_PRINT)
├── child_image/
│   ├── spm.conf                    # Secure Partition Manager config
│   └── circuitdojo_feather_nrf9160.overlay
├── circuitdojo_feather_nrf9160_ns.overlay  # UART2 pin/baud + I2C + ADC configuration
├── prj.conf                        # Zephyr Kconfig (async UART, logging, MCUboot, LTE)
├── CMakeLists.txt
└── Kconfig
```

---

## Hardware

| Item | Detail |
|------|--------|
| MCU | nRF9160 (CircuitDojo Feather) |
| Modem | Iridium 9603 |
| Sensor | LiDAR Lite v3HP (water level, via I2C) |
| UART bus | `uart2` at **19 200 baud** |
| UART pins | Configured in `circuitdojo_feather_nrf9160_ns.overlay` |

---

## Data Frame Format

Each transmission sends **10 bytes** (big-endian, no application checksum — the modem appends its own):

| Bytes | Field | Source |
|-------|-------|--------|
| `[0–1]` | Sleep time | AsTAR++ `schedule()` result (seconds) |
| `[2–3]` | Vcap | Live ADC read (`read_Vcap_mv()` or `read_Vsupp_mv()`) |
| `[4–5]` | Vpv | Live ADC read (`update_Vpv()`) |
| `[6–7]` | Water level | LiDAR Lite v3HP (`lidar_read_water_level()`) in cm |
| `[8–9]` | Signal quality | from `AT+CSQ` (0–31, or 99 = unknown) |

---

## Main Loop

```
main()
  ├── lowpower_setup_gpio()             — disconnect unused GPIOs to save current
  ├── lowpower_setup_accel()            — suspend onboard accelerometer
  ├── astar_init()                      — configure AsTAR++ scheduler
  ├── [GPIO validation checks]          — halt on missing Devicetree nodes
  ├── disable_Vpv_divider()             — disable Vpv divider during modem init
  ├── nrf_modem_lib_init() / lte_lc_init() — init cellular modem in low-power mode
  └── loop:
        [voltage check]                 — read Vcap/Vsupply, check shutoff threshold
        setSuspensionHandler()          — deep sleep if voltage too low (7200 s)
        periph_sw_turn_on()             — power peripheral rail
        lowpower_setup_uart2_ENA()      — resume UART2
        satellite->uart_init()          — register UART callback, enable RX, wait 10 s
        lidar_read_water_level()        — I2C LiDAR read with outlier rejection
        schedule()                      — run AsTAR++ to compute next sleep interval
        build_frame()                   — pack 10-byte frame with live sensor values
        satellite->send_binary()        — AT check → write frame → satellite exchange
        lowpower_setup_uart2_DIS()      — suspend UART2
        periph_sw_turn_off()            — cut peripheral power
        k_sleep(K_SECONDS(sleep_duration)) — deep sleep until next cycle
```

**Background thread** (`over_v_id`): monitors Vcap every 60 s (daytime) or 1800 s (nighttime) and disconnects the solar panel when voltage exceeds `OpenCircuitVoltage`.

---

## AsTAR++ Scheduler (`astar_diurnal.c`)

AsTAR++ is an energy-adaptive algorithm that dynamically adjusts the sleep interval based on harvested energy and time of day.

### Key behaviour

| Condition | Action |
|-----------|--------|
| Vcap ≤ `shutOffVoltage` | Immediate deep sleep for `LowVolt_SleepTime` |
| Daytime (Vpv ≥ `wakeup_Vpv_Threshold`) | Target `daytimeOptimumV`; proportional control |
| Nighttime (Vpv < `sleep_Vpv_Threshold`) | Drain ~90% of charge by estimated sunrise |
| Vcap > `maxVoltage` | Disconnect solar panel; use fastest rate |

### Day/Night detection (hysteresis)

- **Sunset**: Vpv below `sleep_Vpv_Threshold` for ≥ 300 s → nighttime mode.
- **Sunrise**: Vpv above `wakeup_Vpv_Threshold` for ≥ 300 s → daytime mode; updates EWMA night duration estimate.

### Configuration (set in `main.c`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `USE_BOOST` | `false` | No boost converter — Vsupply used instead of Vcap |
| `maxVoltage` | 5100 mV | Overvoltage limit |
| `shutOffVoltage` | 3300 mV | Critical low-voltage threshold |
| `daytimeOptimumV` | 5000 mV | Daytime target voltage |
| `OpenCircuitVoltage` | 5400 mV | Disconnect solar panel above this |
| `wakeup_Vpv_Threshold` | 4000 mV | Vpv needed to confirm sunrise |
| `sleep_Vpv_Threshold` | 3800 mV | Vpv below which sunset is detected |
| `maxRate` | 300 s | Fastest transmission interval |
| `minRate` | 7200 s | Slowest transmission interval |
| `nighttimeMaxRate` | 120 s | Fastest allowed at night |
| `LowVolt_SleepTime` | 7200 s | Sleep time when critically low |
| `nightDurationRollingEstimate` | 48000 s | Initial EWMA night-length estimate |

---

## LiDAR Water Level Sensor (`i2c_sensor4.c`)

Drives a **LiDAR Lite v3HP** over I2C using mode 7 (custom, low-sensitivity, low-error):

- Takes `RAW_READING_NUMBERS` samples, averages them, applies a −10 cm calibration offset.
- **Outlier rejection**: if the result differs from the previous reading by ≥ 15 cm, the measurement is repeated.
- Returns water level in **cm**.

---

## UART Driver Design (`iridium_uart.c`)

The driver is **fully event-driven** — no polling loops.

### Key mechanisms

| Mechanism | Purpose |
|-----------|---------|
| `response_sem` | Thread blocks here; `uart_cb()` signals it when response detected |
| `tx_done_sem` | Thread blocks after `uart_tx()`; signaled on `UART_TX_DONE` |
| `expected_response` | Volatile pointer set by thread; read by callback to match response |
| `resp_buf` + `resp_len` | Accumulates raw UART RX bytes; written by callback only |
| Double RX buffers (`rx_buf[2]`) | Zero-copy handshake via `UART_RX_BUF_REQUEST` |
| `irq_lock()` / `irq_unlock()` | Guards `resp_buf` clears and snapshots |

### Thread-safety rules

- **Clear** `resp_buf`: always under `irq_lock()` via `resp_buf_reset()`.
- **Read** `resp_buf` in thread context: always snapshot under `irq_lock()` into a local buffer first, then parse the local copy.
- **Write** `resp_buf`: only from `uart_cb()` (callback context), no lock needed there.

---

## Public API (`iridium_t` function table)

Obtain the singleton with `iridium_get()`, then call members via `->`:

```c
const iridium_t *satellite = iridium_get();
satellite->uart_init();
satellite->send_binary(frame, FRAME_SIZE);
```

| Member | Signature | Description |
|--------|-----------|-------------|
| `uart_init` | `int (void)` | Init UART, enable RX, wait 10 s for modem boot |
| `send_cmd` | `iridium_result_t (cmd, expected_resp, timeout_s)` | Send AT command, block on semaphore for response |
| `sbd_write` | `iridium_result_t (data, len, timeout_s)` | Load binary payload via `AT+SBDWB` |
| `sbd_initiate` | `iridium_result_t (timeout_s)` | Satellite exchange via `AT+SBDIX` |
| `get_rssi` | `int (void)` | Query signal quality via `AT+CSQ`; returns 0–31 or 99 |
| `get_imei` | `int (buf, buf_len)` | Retrieve 15-digit IMEI via `AT+CGSN` |
| `result_str` | `const char *(result)` | Convert `iridium_result_t` to human-readable string |
| `send_text` | `iridium_result_t (msg, timeout_s)` | Load text via `AT+SBDWT` + initiate exchange |
| `send_binary` | `void (frame, len)` | AT check → write frame → initiate exchange |

### Result codes

| Code | Meaning |
|------|---------|
| `IRIDIUM_OK` | Success |
| `IRIDIUM_ERROR` | Modem returned `ERROR` |
| `IRIDIUM_TIMEOUT` | No response within timeout |
| `IRIDIUM_TX_FAIL` | UART transmit failed |
| `IRIDIUM_INIT_ERR` | Driver init failure |
| `IRIDIUM_BAD_ARG` | NULL or out-of-range argument |
| `IRIDIUM_CHKSUM` | Modem rejected binary checksum |
| `IRIDIUM_NO_NETWORK` | SBDIX MO status: no network service |

---

## Key Timeout Configuration (`iridium_uart.h`)

| Constant | Default | Description |
|----------|---------|-------------|
| `CYCLE_INTERVAL_MS` | 300 000 ms | Fallback delay between cycles (superseded by AsTAR++) |
| `TIMEOUT_AT_S` | 60 s | Timeout for basic AT commands |
| `TIMEOUT_SBDWB_S` | 10 s | Timeout for `AT+SBDWB` |
| `TIMEOUT_SBDIX_S` | 120 s | Timeout for `AT+SBDIX` (satellite acquisition) |
| `FRAME_SIZE` | 10 bytes | Total frame size |
| `IRIDIUM_SBD_MAX_BINARY_LEN` | 340 bytes | Max SBD binary payload |

---

## Build & Flash

```bash
# Configure environment (adjust path to your Zephyr installation)
source ~/zephyrproject/zephyr/zephyr-env.sh

# Build
west build -b circuitdojo_feather_nrf9160_ns .

# Flash
west flash
```

> MCUboot is enabled (`CONFIG_BOOTLOADER_MCUBOOT=y`). Ensure the bootloader is already flashed on the target.

---

## Known Limitations

- The SBD binary result parsing in `iridium_sbd_write_binary()` reads `resp_buf` directly without an `irq_lock` snapshot — a minor race that can be addressed if concurrent RX is observed in practice.
- `RAW_READING_NUMBERS` (LiDAR sample count) is defined in `i2c_sensor4.h`; increase it for better averaging at the cost of longer sensor warm-up time.
- The cellular modem (`nrf_modem_lib` / `lte_lc`) is initialized in low-power mode and is not used for data transmission — it is present for future LTE fallback capability.
