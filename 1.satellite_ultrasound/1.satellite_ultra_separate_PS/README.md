# Iridium Satellite Modem — Zephyr Application

* Author: Van-Vu Bui
* Organization: KU Leuven & Quynhon University
* Time: 07/03/2026

Zephyr RTOS firmware for the **CircuitDojo Feather nRF9160** that measures water level with an **ultrasonic distance sensor (UART3)**, manages energy with the **AsTAR++ adaptive scheduling algorithm**, and transmits data over the **Iridium 9603 SBD** satellite network.

---

## Project Structure

```
2.ultra_sat/
├── src/
│   ├── main.c                      # Entry point: init, scheduling loop, frame build, transmit
│   ├── iridium_uart.c              # UART driver + full Iridium AT command API
│   ├── astar_diurnal.c             # AsTAR++ energy-adaptive task scheduling algorithm
│   ├── uart_sensor.c               # Ultrasonic sensor UART3 driver (water level sensing)
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
│   ├── uart_sensor.h               # Ultrasonic sensor public API declarations
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
│   └── circuitdojo_feather_nrf9160.overlay  # MCUboot overlay
├── circuitdojo_feather_nrf9160_ns.overlay   # UART2/UART3 pins + ADC configuration
├── CMakeLists.txt
└── Kconfig
```

---

## Hardware

| Item | Detail |
|------|--------|
| MCU | nRF9160 (CircuitDojo Feather) |
| Modem | Iridium 9603 |
| Sensor | Ultrasonic distance sensor (water level, via UART3 at 9600 baud) |
| UART2 | Iridium modem at **19 200 baud** (TX=P0.23, RX=P0.24) |
| UART3 | Ultrasonic sensor at **9 600 baud** (TX=P0.19, RX=P0.31) |
| Energy | Solar panel → supercapacitor (max 5.4 V, cut-off 3.3 V) |

---

## Data Frame Format

Each transmission sends **10 bytes** (big-endian, no application checksum — the modem appends its own):

| Bytes | Field | Source |
|-------|-------|--------|
| `[0-1]` | Sleep time | AsTAR++ `schedule()` result (seconds) |
| `[2-3]` | Vcap | Live ADC read (`read_Vcap_mv()` or `read_Vsupp_mv()`) |
| `[4-5]` | Vpv | Live ADC read (`update_Vpv()`) |
| `[6-7]` | Water level | Ultrasonic sensor (`get_safe_distance_cm()`) in cm |
| `[8-9]` | Signal quality | from `AT+CSQ` (0-31, or 99 = unknown) |

---

## Main Loop

```
main()
  ├── lowpower_setup_gpio()              — disconnect unused GPIOs to save current
  ├── lowpower_setup_accel()             — suspend onboard accelerometer
  ├── astar_init()                       — configure AsTAR++ scheduler
  ├── [GPIO validation checks]           — halt on missing Devicetree nodes
  ├── disable_Vpv_divider()              — disable Vpv divider during modem init
  ├── nrf_modem_lib_init() / lte_lc_init() — init cellular modem in low-power mode
  └── loop:
        [voltage check]                  — read Vcap/Vsupply, check shutoff threshold
        setSuspensionHandler()           — deep sleep if voltage too low (7200 s)
        periph_sw_turn_on()              — power peripheral rail (sensor power)
        k_sleep(2000 ms)                 — wait for sensor hardware boot (~2 s)
        get_safe_distance_cm()           — read ultrasonic sensor via UART3
        lowpower_setup_uart2_ENA()       — resume UART2 for satellite modem
        satellite->uart_init()           — register UART callback, enable RX, wait for modem
        schedule()                       — run AsTAR++ to compute next sleep interval
        build_frame()                    — pack 10-byte frame with live sensor values
        satellite->send_binary()         — AT check → write frame → satellite exchange
        satellite->uart_deinit()         — stop async RX cleanly before suspend
        lowpower_setup_uart2_DIS()       — suspend UART2
        periph_sw_turn_off()             — cut peripheral power
        k_sleep(K_SECONDS(sleep_duration)) — deep sleep until next cycle
```

**Background thread** (`over_v_id`): monitors Vcap every 60 s (daytime) or 1800 s (nighttime) and disconnects the solar panel when voltage exceeds `OpenCircuitVoltage`.

---

## Ultrasonic Water Level Sensor (`uart_sensor.c`)

Drives an ultrasonic distance sensor over **UART3** at 9600 baud using Zephyr async UART API.

### Frame format

```
[0xFF] [H] [L] [CS]   (4 bytes)
  CS = (0xFF + H + L) & 0xFF
  Distance = (H << 8) | L  in mm
```

### Semaphore design (two dedicated semaphores)

| Semaphore | Given in | Used by |
|-----------|----------|---------|
| `uart_data_ready_sem` | `UART_RX_RDY` when valid 0xFF frame found | `check_data_available()` only |
| `uart_rx_disabled_sem` | `UART_RX_DISABLED` (natural or forced) | Retry path + cleanup path only |

This split ensures `check_data_available()` returns `true` only when real sensor data is in `uart_value[]`, never on a forced disable from the retry or cleanup paths.

### Key timing constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `RECEIVE_TIMEOUT` | 10 000 us | Idle gap after last byte before `UART_RX_DISABLED` fires |
| `ENABLE_READ_TIMEOUT_MS` | 200 ms | Wait window for `uart_data_ready_sem`; covers 2x sensor output period (~100 ms each) |
| `UART_MAX_RETRIES` | 5 | Max retry attempts — safety net for broken/disconnected sensor |

### Read sequence in `distance_cm()`

1. `lowpower_setup_uart3_ENA()` — resume UART3 peripheral
2. `initialize_uart_sensor()` — register callback, reset both semaphores, `uart_rx_enable()`, 60 ms sleep
3. `check_data_available()` — wait up to 200 ms for `uart_data_ready_sem`
4. Retry up to 5x if no data (safety net for sensor failure): disable RX, wait on `uart_rx_disabled_sem`, re-init
5. `uart_rx_disable()` + wait on `uart_rx_disabled_sem` — clean shutdown before PM suspend
6. `lowpower_setup_uart3_DIS()` — suspend UART3
7. `read_distance()` — validate checksum, convert mm -> cm

### Outlier rejection in `get_safe_distance_cm()`

If the new reading differs from the previous by >= 15 cm, the measurement is repeated once.

---

## AsTAR++ Scheduler (`astar_diurnal.c`)

AsTAR++ is an energy-adaptive algorithm that dynamically adjusts the sleep interval based on harvested energy and time of day.

### Key behaviour

| Condition | Action |
|-----------|--------|
| Vcap <= `shutOffVoltage` | Immediate deep sleep for `LowVolt_SleepTime` |
| Daytime (Vpv >= `wakeup_Vpv_Threshold`) | Target `daytimeOptimumV`; proportional control |
| Nighttime (Vpv < `sleep_Vpv_Threshold`) | Drain ~90% of charge by estimated sunrise |
| Vcap > `maxVoltage` | Disconnect solar panel; use fastest rate |

### Day/Night detection (hysteresis)

- **Sunset**: Vpv below `sleep_Vpv_Threshold` -> nighttime mode.
- **Sunrise**: Vpv above `wakeup_Vpv_Threshold` -> daytime mode; updates EWMA night duration estimate.

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
| `maxRate` | 120 s | Fastest transmission interval |
| `minRate` | 7200 s | Slowest transmission interval |
| `nighttimeMaxRate` | 120 s | Fastest allowed at night |
| `LowVolt_SleepTime` | 7200 s | Sleep time when critically low |
| `nightDurationRollingEstimate` | 48000 s | Initial EWMA night-length estimate |

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
- **Read** `resp_buf` in thread context: snapshot under `irq_lock()` into a local buffer first, then parse the local copy.
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
| `uart_init` | `int (void)` | Init UART, enable RX, wait for modem boot |
| `uart_deinit` | `void (void)` | Stop async RX cleanly before PM suspend |
| `send_cmd` | `iridium_result_t (cmd, expected_resp, timeout_s)` | Send AT command, block on semaphore for response |
| `sbd_write` | `iridium_result_t (data, len, timeout_s)` | Load binary payload via `AT+SBDWB` |
| `sbd_initiate` | `iridium_result_t (timeout_s)` | Satellite exchange via `AT+SBDIX` |
| `get_rssi` | `int (void)` | Query signal quality via `AT+CSQ`; returns 0-31 or 99 |
| `get_imei` | `int (buf, buf_len)` | Retrieve 15-digit IMEI via `AT+CGSN` |
| `result_str` | `const char *(result)` | Convert `iridium_result_t` to human-readable string |
| `send_text` | `iridium_result_t (msg, timeout_s)` | Load text via `AT+SBDWT` + initiate exchange |
| `send_binary` | `void (frame, len)` | AT check -> write frame -> initiate exchange |

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
- The cellular modem (`nrf_modem_lib` / `lte_lc`) is initialized in low-power mode and is not used for data transmission — it is present for future LTE fallback capability.
