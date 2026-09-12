# Host simulation (no Arduino hardware)

This tree is a PC test environment for the Loco-CAN protocol defined in `src/can_protocol.h`. It does not talk to MCP2515, ESP32 TWAI, or any other CAN transceiver. Virtual modules share an in-memory bus, so firmware-facing message IDs, 29-bit packing, filters, and SerialCAN USB framing can be exercised on a laptop or in CI.

## What it covers

- **Protocol codecs** for drive, vehicle status, lights, signals, loco setup, and u16 telemetry, using the same `#define`s as the Arduino library.
- **Extended ID packing** used by the firmware CAN driver: `(id << 18) | (uuid & 0x3FFFF)`.
- **Acceptance filters** with the firmware rule `(id & mask) == filter` and `CAN_ID_MASK 0x770`.
- **Virtual modules**: controller, vehicle/motor, lights.
- **Scenarios**: drive, heartbeat loss / emergency stop, second controller lockout, lights, setup request, horn/signal.
- **SerialCAN framing** compatible with `python_monitor` (`0xFF 0xFF … XOR … LOCO\\r\\n`).

## Build and run

From the repository root (CMake 3.16+, a C++17 compiler):

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Or `make test`.

Binaries:

| Binary | Purpose |
| --- | --- |
| `build/host/loco_host_tests` | unit tests |
| `build/host/loco-sim` | scenarios and an interactive REPL |

```bash
build/host/loco-sim --list
build/host/loco-sim --scenario drive
build/host/loco-sim --scenario all
build/host/loco-sim                 # REPL
```

Example REPL session:

```
sim> mains on
sim> drive on
sim> throttle 400
sim> tick 250
sim> status
sim> lights train,cab
sim> tick 200
sim> drop
sim> tick 1100
sim> status
```

`drop` disconnects the virtual controller without sending mains-off, which is how heartbeat-loss / emergency-stop is tested.

## Limits

This is a protocol and state-machine harness, not a physics engine and not a drop-in replacement for `loco-can-software`. GPIO, PWM, MCP2515 timing, and EEPROM setup dumps still need hardware or the firmware repo. Telemetry payloads (speed/current/voltage) use a host convention of big-endian uint16 values because the Arduino library does not yet document those byte layouts.

When a USB SerialCAN adapter is present, the same `encode_serialcan` / `decode_serialcan` helpers can be wired to a real adapter later; they are tested here with in-memory buffers only.
