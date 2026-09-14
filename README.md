# obd_session_rs

The OBD-II session engine behind [RevOBD](https://greyhorsesoftware.com) (macOS / iPadOS) and
[omatach](https://github.com/greyhorsesoftware/omatach-releases) (Linux terminal). It owns the adapter
link, identifies the vehicle, and polls PIDs from named subscriptions on a background thread, handing
every parsed reply to the host as JSON. Use it from Rust directly, or from any language through the
C FFI and a cbindgen-generated header.

## What it does

- **Adapters** — ELM327 and STN (OBDLink) text dialect, and the OBDX Pro DVI binary protocol, behind one
  `LinkHandler` seam. Adapter facts (pipe / chunk capability, STN periodic, DVI) come from an
  `adapters.json` catalog the host hands in.
- **Identify** — VIN, calibration ids, ECU roster and names, supported-PID map, negotiated protocol;
  cached per VIN so reconnects skip the walk.
- **11-bit and 29-bit CAN** (ISO 15765-4) — the addressing is sniffed from the first response header and
  every layer above de-framing is header-independent. See [CAN Addressing](#can-addressing-11-bit--29-bit).
- **Subscriptions** — named PID sets with Fast / Medium / Slow tiers, deduplicated across subscriptions,
  token-bucket rate limited, with run-once PIDs and dynamic add / remove.
- **Fast tiers** — multi-PID pipelining on capable adapters, STN periodic messaging, and UDS
  `2C` dynamic-DID streaming where the vehicle supports it (`acquisition`, `stn_periodic`,
  `periodic_stream`).
- **Transports** — the host can feed bytes from its own stack (`external_platform`), or the crate drives
  the adapter itself: TCP (WiFi adapters), serial (USB), and, behind features, BLE and Bluetooth Classic
  on Linux / Windows / macOS.
- **Observability** — a session monitor (per-command timing, mismatch detection), session health
  windows, and a wire log of every TX / RX / DROP.

## Building

```bash
cargo build --release                      # core: TCP + serial transports, C FFI
cargo build --release --features ble       # + BLE (btleplug)
cargo build --release --features full-bluetooth   # + BLE + Bluetooth Classic
cargo test --all-features
```

**Linux prerequisites for the Bluetooth features:** BlueZ 5.50+ with development headers
(`bluez libbluetooth-dev libdbus-1-dev pkg-config` on Debian / Ubuntu, `bluez bluez-utils` on Arch), and
the `bluetooth` service running.

`build_universal.sh` builds the Apple static libraries (`debug`, `release`, `macos`, `ios`) plus the
cbindgen header and stages them into `../lib` by default, or wherever `OBD_LIB_DIR` points:

```bash
OBD_LIB_DIR=./lib ./build_universal.sh release
```

The toolchain is pinned in `rust-toolchain.toml` so the static library stays link-compatible with
[obd_equation_rs](https://github.com/greyhorsesoftware/obd_equation_rs) when both are linked into one
binary. The GitHub Actions workflow tests with all features on Linux and builds Linux, Windows, macOS,
and iOS artifacts.

## Using from Rust

The engine is an `OBDSessionManager` built from a platform (the transport), a config, and a JSON
callback. `api_handle()` returns the handle every call goes through.

```rust
use obd_session_rs::{OBDSessionManager, OBDSessionConfig};
use obd_session_rs::external_platform::create_mock_external_platform;

// The mock platform answers from mock_data/ — the same code path as production.
let (platform, _ctx) = create_mock_external_platform();
let session = OBDSessionManager::new(platform, OBDSessionConfig::default(), |json| {
    println!("{json}");
})?;

let api = session.api_handle();
let dash = api.create_subscription(Some("dash".into()), vec!["010C".into(), "010D".into()], None)?;
api.start_subscription(dash)?;
```

For real hardware on the Rust route, build an `ExternalPlatform`, hand the adapter catalog to
`set_adapter_catalog`, and call `connect_to_controller`. Discovery results arrive as
`connection_progress` messages (`connector_list`, `waiting_for_selection`); answer with
`select_connector`, and the session identifies the vehicle and reports `connected` followed by
`vehicle_info`. Connector ids carry their transport: `wifi:host:port`, `usb:/dev/tty…`, `ble:…`,
`rfcomm:…`.

Every reply reaches the callback as an `obd_data` message with the raw response and a `parsed` block
keyed by responding ECU (`raw_hex`, `data_bytes`, `is_valid`). See the module docs (`cargo doc --open`)
for the full API and message shapes.

## Using from C

`cargo build --release` produces `libobd_session_rs.a`; `cbindgen --config cbindgen.toml --crate
obd_session_rs --output obd_session_rs.h` produces the header (`build_universal.sh` does both). On this
route the host owns the transport and feeds bytes to Rust; the flow and the function reference follow
below.

## Architecture

```
Host Application
    |
    |  C FFI calls
    v
obd_session_rs
    |-- Session Manager      (orchestration, identify, connection lifecycle)
    |-- Link Handlers         (ELM327 / STN text dialect, OBDX Pro DVI binary protocol)
    |-- Command Processor     (serial execution, rate limiting, timing)
    |-- Subscription Manager  (PID scheduling, tiered refresh, deduplication)
    |-- Addressing            (11-bit / 29-bit CAN detection + header read/generate)
    |-- Response Parser       (OBD hex parsing, multi-controller, ISO-TP)
    |-- Session Monitor       (real-time command/response tracing, mismatch detection)
    |-- PID Registry          (global deduplication, caching)
    |-- Cache                 (vehicle info persistence across sessions)
    |-- Transports            (TCP, serial; BLE + Bluetooth Classic behind features)
    |
    v
Adapter — via the host's transport (C FFI route) or the crate's own (Rust route)
```

### CAN Addressing (11-bit / 29-bit)

The library speaks both ISO 15765-4 CAN variants and picks the right one automatically:

- **Detection** — at connect, the negotiated protocol is sniffed from the first response header
  (`7Ex` → 11-bit; `18 DA F1 xx` → 29-bit, with the tester address learned from the wire), with `ATDPN`
  as a fallback. A non-CAN protocol (J1850, ISO 9141, KWP) fails the connection with a clear error rather
  than misreading the bus.
- **Uniform identity** — controllers are keyed by a protocol-neutral ECU byte (`"00"`, `"10"`), not the raw
  header. The `addressing` module (`Addressing` / `Controller` / `Bus`) is the single place that knows wire
  formats: it reads response headers into a `Controller` and renders `Controller` → `ATSH` (`7E0` on 11-bit,
  `18DA10F1` on 29-bit; functional `7DF` / `18DB33F1`). Everything downstream of de-framing is
  header-independent.
- **29-bit specifics** — ECUs are enumerated passively from the multi-ECU functional-broadcast responses
  (source addresses like `0x10`, `0x28`), supported PIDs are unioned across all responding ECUs, and an
  imported 11-bit target (e.g. a Torque `preferredController` of `7E0`) that has no 29-bit equivalent falls
  back to functional addressing, with responses attributed by their source header.
- The negotiated protocol is reported to the host as a display string on `VehicleInfo.protocol`
  (e.g. `"ISO 15765-4 CAN 29-bit (500k)"`) and cached per-VIN.

### Adapter Communication Flow (host-fed route)

On the C FFI route the host owns the transport and feeds bytes to Rust:

```
                    HOST APPLICATION                         obd_session_rs
                   (Transport Layer)                         (Rust Library)

  OBD Adapter
  (ELM327/STN)     BLE / WiFi / USB
       |                  |
       |    ============================================    SETUP
       |    |             |                            |
       |    |  obd_create_session_with_platform() ---->|  Create session
       |    |    - send_callback        (Rust->Host)   |  with callbacks
       |    |    - response_callback    (Rust->Host)   |
       |    |             |                            |
       |    ============================================
       |                  |
       |    ============================================    START SUBSCRIPTION
       |    |             |                            |
       |    |  obd_start_subscription() -------------->|  App starts polling
       |    |             |                            |
       |    |             |  1. Queue "AT\r"           |  Flush adapter first
       |    |             |                            |
       |    |             |  2. send_callback("AT")    |  Rust calls host
       |    |  <--------- |     via C FFI callback     |
       |    |             |                            |
  <----|----| 3. Host writes "AT\r" to adapter         |
  -----|---->    Adapter responds "OK"                  |
       |    |             |                            |
       |    |  4. obd_external_receive_response() ---->|  "OK" — adapter clean
       |    |     ("AT", "OK")                         |  (suppressed from app)
       |    |             |                            |
       |    |             |  5. get_next_command()     |  Now start real polling
       |    |             |     picks first PID        |
       |    |             |                            |
       |    ============================================
       |                  |
       |    ============================================    COMMAND CYCLE
       |    |             |                            |
       |    |             |  1. send_callback("010C")  |  Rust calls host
       |    |  <--------- |     via C FFI callback     |  to send command
       |    |             |                            |
  <----|----| 2. Host writes "010C\r" to adapter       |
       |    |    via BLE/WiFi/USB                      |
       |    |             |                            |
  -----|----> 3. Adapter responds                      |
       |    |    "7E8 04 41 0C 1A F8"                  |
       |    |             |                            |
       |    |  4. obd_external_receive_response() ---->|  Host sends
       |    |     (command, response)                  |  response to Rust
       |    |             |                            |
       |    |             |  5. Parse response         |  Validate PID match,
       |    |             |     command_completed()    |  update subscription,
       |    |             |     obd_data -> app        |  deliver to app
       |    |             |                            |
       |    |             |  6. get_next_command()     |  Pick next PID,
       |    |             |     send_callback(next)    |  continue polling
       |    |             |                            |
       |    ============================================
       |                  |
       |    ============================================    ERROR HANDLING
       |    |             |                            |
       |    |  Host detects timeout or error            |
       |    |             |                            |
       |    |  obd_external_receive_error() ---------->|  Host reports error
       |    |     (command, error)                     |
       |    |             |                            |
       |    |             |  1. obd_data with error    |  App sees error JSON
       |    |             |     -> app                 |
       |    |             |                            |
       |    |             |  2. Pause all active       |  Stop sending commands
       |    |             |     subscriptions          |
       |    |             |                            |
       |    |             |  3. Clear in-flight PIDs   |  Reset tracking
       |    |             |                            |
       |    |             |  Pipeline STOPPED          |  No more commands
       |    |             |                            |
       |    ============================================
       |                  |
       |    ============================================    RECOVERY
       |    |             |                            |
       |    |  App shows error UI                      |
       |    |  User clicks "Resume"                    |
       |    |             |                            |
       |    |  obd_start_subscription() -------------->|  Resume polling
       |    |             |                            |
       |    |             |  1. Queue "AT\r"           |  Flush adapter
       |    |             |  2. "OK" received          |  Adapter is clean
       |    |             |  3. get_next_command()     |  Resume real polling
       |    |             |                            |
       |    ============================================
```

### Data Flow Summary

| Step | Direction | Function | Purpose |
|------|-----------|----------|---------|
| Setup | Host -> Rust | `obd_create_session_with_platform()` | Create session with callbacks |
| Start | Host -> Rust | `obd_start_subscription()` | Begin polling (sends AT\r flush first) |
| Send | Rust -> Host | `send_callback(command)` | Tell host to send command to adapter |
| Transport | Host -> Adapter | BLE write / TCP send | Transport-specific send |
| Receive | Adapter -> Host | BLE notification / TCP read | Transport-specific receive |
| Deliver | Host -> Rust | `obd_external_receive_response()` | Deliver response to Rust |
| Data | Rust -> Host | `response_callback(json)` | Deliver parsed `obd_data` to app |
| Error | Host -> Rust | `obd_external_receive_error()` | Report timeout/error — pauses all subscriptions |
| Resume | Host -> Rust | `obd_start_subscription()` | Resume after error (AT\r flush + polling) |

## Error Handling

When the host reports an error via `obd_external_receive_error()`:

1. An `obd_data` message with the error is sent to the app via `response_callback`
2. All active subscriptions are automatically paused
3. In-flight PID tracking is cleared
4. The command pipeline stops — no more commands are sent

To resume after an error:
1. App shows error UI to the user
2. User clicks "Resume"
3. App calls `obd_start_subscription(handle, subscription_id)` for each subscription
4. Rust sends `AT\r` to flush the adapter, then resumes normal PID polling

The `AT\r` flush ensures the adapter is in a clean state before sending real commands. It does not reset adapter settings (unlike `ATZ`) — it's just a ping that clears any stale buffer.

## FFI API Reference

All functions use C calling convention. Strings returned by `obd_get_*` / `obd_create_*` must be freed with `obd_free_string()`.

### Session Lifecycle

| Function | Description |
|----------|-------------|
| `obd_create_session_with_platform(send_cb, response_cb, ctx, ...)` | Create session with external transport |
| `obd_get_external_api_handle(session)` | Get API handle for operations |
| `obd_destroy_external_session(session)` | Destroy session and free resources |
| `obd_external_receive_response(session, command, response)` | Deliver adapter response to Rust |
| `obd_external_receive_error(session, command, error)` | Deliver adapter error — pauses all subscriptions |
| `obd_external_update_connection(session, status, reason)` | Update connection status |

### Connection Management

| Function | Description |
|----------|-------------|
| `obd_connect_to_controller(session, connector_id)` | Connect to a specific adapter |
| `obd_select_connector(session, connector_id)` | Select connector after discovery |
| `obd_disconnect(session)` | Disconnect from adapter |
| `obd_update_discovered_connectors(session, json)` | Update available connectors |

### Commands

| Function | Description |
|----------|-------------|
| `obd_send_command(handle, command, timeout_ms)` | Send a single OBD command |

### Subscriptions

| Function | Description |
|----------|-------------|
| `obd_create_subscription(handle, name, pids, count, controller)` | Create named subscription (continuous) |
| `obd_create_subscription_with_run_once(handle, name, pids, count, controller, run_once_pids, run_once_count)` | Create subscription with run-once PIDs |
| `obd_start_subscription(handle, subscription_id)` | Start polling (sends AT\r flush first) |
| `obd_pause_subscription(handle, subscription_id)` | Pause polling |
| `obd_cancel_subscription(handle, subscription_id)` | Cancel and remove |
| `obd_add_pids_to_subscription(handle, subscription_id, pids, count, controller)` | Add PIDs dynamically |
| `obd_remove_pids_from_subscription(handle, subscription_id, pids, count)` | Remove PIDs |
| `obd_set_subscription_timeout(handle, subscription_id, timeout_ms)` | Override timeout |
| `obd_set_subscription_tiers(handle, subscription_id, tiers_json)` | Set PID refresh tiers |

### Subscription Introspection

| Function | Returns | Description |
|----------|---------|-------------|
| `obd_get_subscriptions(handle)` | JSON `char*` | All subscriptions with PIDs, tiers, state, in-flight flag |
| `obd_get_subscription_audit_log(handle, count)` | JSON `char*` | Last N PID add/remove operations with timestamps |
| `obd_install_subscription_change_callback(handle, cb)` | `OBDError` | Real-time callback on PID add/remove |
| `obd_remove_subscription_change_callback(handle)` | `OBDError` | Remove callback |

### Session Monitor

| Function | Returns | Description |
|----------|---------|-------------|
| `obd_install_session_monitor(handle, callback)` | `OBDError` | Real-time callback per command/response with timing |
| `obd_remove_session_monitor(handle)` | `OBDError` | Remove callback |
| `obd_get_session_history(handle, count)` | JSON `char*` | Last N monitor entries from ring buffer |
| `obd_clear_session_history(handle)` | `OBDError` | Clear ring buffer |

### Statistics

| Function | Returns | Description |
|----------|---------|-------------|
| `obd_get_stats(handle)` | JSON `char*` | Command stats, throughput, mismatch rate |
| `obd_reset_stats(handle)` | `OBDError` | Reset counters |
| `obd_set_rate_limit(handle, cmds_per_sec)` | `OBDError` | Update rate limiter |

### Vehicle Info & Cache

| Function | Returns | Description |
|----------|---------|-------------|
| `obd_probe_pids_cached(handle, cache_path)` | JSON `char*` | Discover vehicle with cache |
| `obd_get_cached_vehicle_info(handle)` | JSON `char*` | Get cached vehicle info |
| `obd_save_probe_results(handle, cache_path, json)` | `OBDError` | Save probe results to disk |
| `obd_clear_cache(handle, cache_path)` | `OBDError` | Delete cache file |

### Utilities

| Function | Description |
|----------|-------------|
| `obd_get_version()` | Library version string |
| `obd_free_string(ptr)` | Free a Rust-allocated string |
| `obd_free_error(error)` | Free an OBDError struct |

## Key JSON Structures

### Session Monitor Entry

Delivered via `obd_install_session_monitor` callback or `obd_get_session_history`:

```json
{
  "command": "010C",
  "subscription_id": "uuid",
  "subscription_name": "Dashboard",
  "target_controller": "7E0",
  "response": "7E8 04 41 0C 1A F8",
  "queue_time_ms": 2.1,
  "response_time_ms": 48.3,
  "total_time_ms": 50.4,
  "success": true,
  "pid_mismatch": false,
  "response_pid": "010C",
  "timestamp": 1711036845.123,
  "is_at_command": false,
  "tier": "Fast"
}
```

### Subscription Snapshot

Returned by `obd_get_subscriptions`:

```json
[{
  "id": "uuid",
  "subscription_name": "Dashboard",
  "state": "Active",
  "target_controller": "7E0",
  "created_at": 1711036800.0,
  "last_active": 1711036850.0,
  "is_run_once_only": false,
  "pid_count": 3,
  "pids": [
    {"pid": "010C", "tier": "Fast", "run_count": null, "execution_count": 142, "in_flight": true},
    {"pid": "010D", "tier": "Fast", "run_count": null, "execution_count": 140, "in_flight": false},
    {"pid": "0105", "tier": "Slow", "run_count": null, "execution_count": 14, "in_flight": false}
  ]
}]
```

### Subscription Audit Entry

Delivered via `obd_install_subscription_change_callback` or `obd_get_subscription_audit_log`:

```json
{
  "action": "add",
  "subscription_id": "uuid",
  "subscription_name": "Dashboard",
  "pids": ["0104"],
  "target_controller": "7E0",
  "timestamp": 1711036845.0
}
```

### obd_data (normal response)

```json
{
  "type": "obd_data",
  "command": "010C",
  "data": "7E8 04 41 0C 1A F8",
  "error": null,
  "parsed": { ... }
}
```

### obd_data (error response — subscriptions paused)

```json
{
  "type": "obd_data",
  "command": "010C",
  "data": "",
  "error": "NoResponse { command: \"010C\" }"
}
```

When the app receives an `obd_data` message with `error` set, all subscriptions have been paused. Call `obd_start_subscription` to resume.

### Stats

Returned by `obd_get_stats`:

```json
{
  "total_commands": 500,
  "completed_commands": 498,
  "failed_commands": 2,
  "pids_per_second": 18.5,
  "average_completion_time_ms": 52,
  "in_progress_commands": 1,
  "response_mismatch_count": 3,
  "response_total_count": 498,
  "response_mismatch_rate": 0.006024
}
```

## PID Refresh Tiers

Subscriptions support weighted polling. Fast PIDs are polled every cycle, Medium every 3rd, Slow every 10th:

| Tier | Divisor | Typical PIDs |
|------|---------|--------------|
| Fast | 1 | RPM, Speed, Throttle, MAF, Engine Load |
| Medium | 3 | Fuel trims, O2 sensors, timing advance |
| Slow | 10 | Coolant temp, fuel level, runtime, barometric pressure |

Set tiers via `obd_set_subscription_tiers` with a JSON map: `{"010C": 1, "0105": 10}`.

## Response Mismatch Detection

The library detects when the PID in the OBD response bytes (`41 XX` / `62 XX YY`) doesn't match the PID that was sent. This catches adapter buffering issues. Mismatches are:

- Counted in `obd_get_stats` (`response_mismatch_count` / `response_mismatch_rate`)
- Flagged per-entry in session monitor (`pid_mismatch: true`)
- Automatically re-routed to the correct PID based on the response bytes

Supports Mode 01 (`41 XX`), Mode 09 (`49 XX`), Mode 22 (`62 XX YY`), and Mode 24 (`64 XX YY`).

## Mock Data

`mock_data/` holds adapter transcripts captured from real vehicles, one JSON file per car, keyed by
mode and PID with the exact frames the adapter returned (multi-frame ISO-TP, interleaved ECUs and all):

| File | Vehicle | Bus |
|------|---------|-----|
| `default.json` | Chevrolet | 11-bit |
| `bmw428i.json` | BMW 428i | 11-bit, two ECUs |
| `gladiator.json` | Jeep Gladiator | 11-bit, two ECUs |
| `mustang50.json` | Ford Mustang GT | 11-bit |
| `rangerover.json` | Range Rover | 11-bit, three ECUs |
| `29bitJeep.json` | Jeep (Global-B) | 29-bit, `18 DA F1 xx` headers |

Every VIN and calibration id in them is synthetic (`tools/synthesize_fixtures.py` rewrites the
`0902` / `0904` frames byte for byte, keeping the WMI and model-year character). Do not add a fixture
with a real VIN — run the script on it first.

The mock platform (`external_platform::create_mock_external_platform`) replays these files, and the
golden tests in `session_manager/golden.rs` pin the identify result for each one under
`tests/fixtures/golden/`. A deliberate behaviour change regenerates them with
`UPDATE_GOLDEN=1 cargo test golden_`; read the diff.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT)
at your option. Copyright (c) 2025-2026 [Grey Horse Software](https://greyhorsesoftware.com).

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this
crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
