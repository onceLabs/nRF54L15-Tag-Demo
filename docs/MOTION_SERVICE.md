# nRF54L15 Tag — Motion Service BLE Protocol

Self-contained specification for the custom BLE **Motion Service** that streams
high-rate accelerometer and gyroscope data from the nRF54L15 Tag. This document
is everything a client needs to discover, connect, subscribe, and decode the
data — no access to the firmware source is required.

Audience: developers (or an AI assistant) building a companion UI — e.g. a
Python (bleak) host or a TypeScript VS Code extension.

---

## 1. Overview

- The Tag is a **BLE peripheral**. The companion app is the **central** (it
  scans, connects, and subscribes).
- **Find the device by name:** it advertises the complete local name
  **`nRF54L15 Tag`**. (It also advertises the 16-bit Environmental Sensing
  Service UUID `0x181A`, which is unrelated to motion.)
- The **Motion Service UUID is 128-bit and is NOT in the advertising data.**
  You must connect first, then perform GATT service discovery to find it.
- Once subscribed, the device streams **accelerometer** and/or **gyroscope**
  samples as GATT **notifications**. Streaming starts automatically on
  subscribe and stops when you unsubscribe.

Typical flow:

1. Scan → match device by local name `nRF54L15 Tag`.
2. Connect.
3. (Recommended) Negotiate a large ATT MTU and 2M PHY (see §2).
4. Discover the Motion Service and its characteristics by UUID (§3).
5. Enable notifications (CCCD) on the accel and/or gyro characteristic.
6. Decode incoming notification packets (§4, §5).

---

## 2. Connection setup (recommended)

The Tag samples the IMU at up to **1600 Hz**, so throughput matters. The device
supports and prefers:

- **ATT MTU up to 498 bytes** — request an MTU exchange right after connecting.
  Larger MTU = more samples per notification (see the table in §4).
- **LE 2M PHY** — request a PHY update to 2M for higher throughput.
- **Data Length Extension** (up to 251-byte PDUs) — supported; most stacks
  negotiate this automatically.
- **Short connection interval** — on the first subscribe the Tag itself
  requests a short interval (default 7.5 ms; 4 s supervision timeout). The
  central ultimately controls the interval. A shorter interval means lower
  latency and fewer samples buffered per notification.

None of these are mandatory to receive data — they only affect throughput and
latency. With the default ATT MTU (23) you still get 1 sample per notification.

---

## 3. Service & characteristics

All UUIDs are 128-bit with the base suffix `-9b0f-4a3e-8b1a-2f9c0d5e7a10`.

| Role | UUID | Properties |
|------|------|------------|
| **Motion Service** | `f0de1b00-9b0f-4a3e-8b1a-2f9c0d5e7a10` | Primary service |
| **Accelerometer Stream** | `f0de1b01-9b0f-4a3e-8b1a-2f9c0d5e7a10` | Notify |
| **Gyroscope Stream** | `f0de1b02-9b0f-4a3e-8b1a-2f9c0d5e7a10` | Notify |
| **Configuration** | `f0de1b03-9b0f-4a3e-8b1a-2f9c0d5e7a10` | Read, Write |

The accel and gyro characteristics are **notify-only** (no read/write) and each
has a Client Characteristic Configuration Descriptor (CCCD, `0x2902`). The
**Configuration** characteristic (§6) is read/write and sets the sample rates —
it has no CCCD.

### Start / stop streaming

- **Start:** enable notifications on the accel characteristic and/or the gyro
  characteristic (your BLE stack writes the CCCD for you). Sampling
  **auto-starts** as soon as the first of the two is subscribed.
- **Stop:** disable notifications on both. Sampling stops when neither is
  subscribed.
- The two are **independent** — subscribe to just accel, just gyro, or both.
  Both use the identical packet format described below.

---

## 4. Notification packet format

Each notification on either characteristic is a **batch of 1..N samples**. The
number of samples per notification varies with the connection interval and
sample rate (roughly `interval × rate`; it can be as low as 1 at very short
intervals). Do not assume a fixed count — always read the `count` byte.

**All fields are little-endian.**

```
byte 0        : uint8   count          (number of samples in this packet)
byte 1..      : count × Sample (16 bytes each)

Sample (16 bytes):
  offset 0 : int32   x
  offset 4 : int32   y
  offset 8 : int32   z
  offset 12: uint32  t_us             (device uptime, microseconds)
```

- Total packet length = `1 + count * 16` bytes.
- For the **accel** characteristic, `x/y/z` are acceleration; for the **gyro**
  characteristic they are angular rate. Units and conversions are in §5.
- `count` upper bound depends on the negotiated ATT MTU:
  `count_max = min(30, (ATT_MTU - 4) / 16)` (the firmware hard-caps at 30).

| Negotiated ATT MTU | Max samples / notification |
|--------------------|----------------------------|
| 23 (default)       | 1                          |
| 247                | 15                         |
| 498 (device max)   | 30                         |

---

## 5. Units & conversions

| Field | Characteristic | Raw unit | To SI | To common |
|-------|----------------|----------|-------|-----------|
| x, y, z | Accel | micro-m/s² (int32) | `÷ 1e6` → m/s² | `÷ 9.80665e6` → g |
| x, y, z | Gyro  | micro-rad/s (int32) | `÷ 1e6` → rad/s | `× (180/π)/1e6` → °/s |
| t_us | both | microseconds (uint32) | device uptime | — |

Notes:

- **`t_us` is a `uint32` and wraps** every 2³² µs ≈ **4294.97 s (~71.6 min)**.
  Compute time deltas with unsigned 32-bit subtraction to handle wrap.
- **Accel and gyro are sampled together.** A gyro sample and an accel sample
  that share the same `t_us` came from the same instant — use `t_us` to align
  the two streams.
- Samples within/across notifications are (approximately) evenly spaced at the
  configured rate. **Derive the actual rate from `t_us` deltas** rather than
  assuming a nominal value.
- Example: accel `x = 9,806,650` → `9.80665 m/s²` → `1.0 g`.

---

## 6. Configuration characteristic (poll rate / ODR / range)

The **Configuration** characteristic (`f0de1b03-…`, Read + Write) sets the IMU
sampling parameters at runtime — the same knobs available on the device's debug
shell. Changes take effect immediately (and while streaming).

### Write — one command at a time

Write exactly **5 bytes, little-endian**: `[uint8 field_id][uint32 value]`.

| field_id | parameter | value / units | valid values |
|----------|-----------|---------------|--------------|
| `0` | stream poll rate | Hz (`0` = follow the accel ODR) | any; a value above the ODR just repeats samples |
| `1` | imu-accel ODR | Hz | 1 – 1600 |
| `2` | imu-gyro ODR | Hz | 25 – 3200 |
| `3` | imu-accel full-scale range | g | 2, 4, 8, 16 |
| `4` | imu-gyro full-scale range | dps | 125, 250, 500, 1000, 2000 |

Notes:
- **Poll rate vs ODR:** the poll rate is how often the firmware reads a sample;
  it is independent of the sensor ODR. Set it ≤ the ODR (a higher poll rate just
  re-reads the same sample).
- The BMI270 quantizes ODR to its supported steps (…/400/800/1600 for accel;
  the gyro adds 3200). A request is accepted and snapped to the nearest step; a
  subsequent read (below) returns the **requested** value, which may differ from
  the exact hardware step.
- **Errors:** a malformed length returns ATT error *Invalid Attribute Length*;
  an unknown `field_id` or an out-of-range value returns ATT error *Value Not
  Allowed* (`0x13`) and nothing changes.

### Read — current settings

Reading the characteristic returns a **20-byte little-endian struct** of the
current (last-requested) settings:

```
offset 0  : uint32 poll_rate_hz     (0 = following ODR)
offset 4  : uint32 accel_odr_hz
offset 8  : uint32 gyro_odr_hz
offset 12 : uint32 accel_range_g
offset 16 : uint32 gyro_range_dps
```

### Examples

Python (bleak):

```python
import struct
CFG_UUID = "f0de1b03-9b0f-4a3e-8b1a-2f9c0d5e7a10"

# Set imu-accel ODR to 1600 Hz, then the poll rate to 1600 Hz.
await client.write_gatt_char(CFG_UUID, struct.pack("<BI", 1, 1600), response=True)
await client.write_gatt_char(CFG_UUID, struct.pack("<BI", 0, 1600), response=True)

# Read back the current config.
raw = await client.read_gatt_char(CFG_UUID)
poll, a_odr, g_odr, a_rng, g_rng = struct.unpack("<IIIII", raw)
print(poll, a_odr, g_odr, a_rng, g_rng)
```

TypeScript (Web Bluetooth):

```ts
const CFG = "f0de1b03-9b0f-4a3e-8b1a-2f9c0d5e7a10";
const cfg = await svc.getCharacteristic(CFG);

function cfgCmd(fieldId: number, value: number): Uint8Array {
  const b = new DataView(new ArrayBuffer(5));
  b.setUint8(0, fieldId);
  b.setUint32(1, value, true);           // little-endian
  return new Uint8Array(b.buffer);
}

await cfg.writeValue(cfgCmd(1, 1600));   // imu-accel ODR = 1600 Hz
await cfg.writeValue(cfgCmd(0, 1600));   // poll rate = 1600 Hz

const dv = await cfg.readValue();
const current = {
  pollRateHz:  dv.getUint32(0, true),
  accelOdrHz:  dv.getUint32(4, true),
  gyroOdrHz:   dv.getUint32(8, true),
  accelRangeG: dv.getUint32(12, true),
  gyroRangeDps:dv.getUint32(16, true),
};
```

---

## 7. Parsing examples

### Python (bleak)

```python
import asyncio
import struct
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "nRF54L15 Tag"
ACCEL_UUID  = "f0de1b01-9b0f-4a3e-8b1a-2f9c0d5e7a10"
GYRO_UUID   = "f0de1b02-9b0f-4a3e-8b1a-2f9c0d5e7a10"

def parse_packet(data: bytes):
    """Yield (x, y, z, t_us) with x/y/z in raw micro-units."""
    count = data[0]
    off = 1
    for _ in range(count):
        x, y, z, t_us = struct.unpack_from("<iiiI", data, off)
        off += 16
        yield x, y, z, t_us

def on_accel(_char, data: bytearray):
    for x, y, z, t_us in parse_packet(bytes(data)):
        ax, ay, az = x / 1e6, y / 1e6, z / 1e6            # m/s^2
        print(f"accel t={t_us} us  {ax:.3f} {ay:.3f} {az:.3f} m/s^2")

def on_gyro(_char, data: bytearray):
    for x, y, z, t_us in parse_packet(bytes(data)):
        gx, gy, gz = x / 1e6, y / 1e6, z / 1e6            # rad/s
        print(f"gyro  t={t_us} us  {gx:.3f} {gy:.3f} {gz:.3f} rad/s")

async def main():
    dev = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
    if dev is None:
        raise RuntimeError("Tag not found")
    async with BleakClient(dev) as client:
        # bleak negotiates MTU automatically on most backends.
        await client.start_notify(ACCEL_UUID, on_accel)
        await client.start_notify(GYRO_UUID, on_gyro)
        await asyncio.sleep(30)
        await client.stop_notify(ACCEL_UUID)
        await client.stop_notify(GYRO_UUID)

asyncio.run(main())
```

### TypeScript (Web Bluetooth)

```ts
const SERVICE = "f0de1b00-9b0f-4a3e-8b1a-2f9c0d5e7a10";
const ACCEL   = "f0de1b01-9b0f-4a3e-8b1a-2f9c0d5e7a10";
const GYRO    = "f0de1b02-9b0f-4a3e-8b1a-2f9c0d5e7a10";

interface Sample { x: number; y: number; z: number; tUs: number; }

function parsePacket(dv: DataView): Sample[] {
  const count = dv.getUint8(0);
  const out: Sample[] = [];
  let off = 1;
  for (let i = 0; i < count; i++) {
    out.push({
      x:  dv.getInt32(off, true),        // little-endian
      y:  dv.getInt32(off + 4, true),
      z:  dv.getInt32(off + 8, true),
      tUs: dv.getUint32(off + 12, true),
    });
    off += 16;
  }
  return out;
}

async function connect() {
  const device = await navigator.bluetooth.requestDevice({
    filters: [{ name: "nRF54L15 Tag" }],
    optionalServices: [SERVICE],
  });
  const gatt = await device.gatt!.connect();
  const svc = await gatt.getPrimaryService(SERVICE);

  const accel = await svc.getCharacteristic(ACCEL);
  accel.addEventListener("characteristicvaluechanged", (e) => {
    const dv = (e.target as BluetoothRemoteGATTCharacteristic).value!;
    for (const s of parsePacket(dv)) {
      // raw micro-m/s^2 -> m/s^2
      console.log("accel", s.tUs, s.x / 1e6, s.y / 1e6, s.z / 1e6);
    }
  });
  await accel.startNotifications();

  const gyro = await svc.getCharacteristic(GYRO);
  gyro.addEventListener("characteristicvaluechanged", (e) => {
    const dv = (e.target as BluetoothRemoteGATTCharacteristic).value!;
    for (const s of parsePacket(dv)) {
      // raw micro-rad/s -> rad/s
      console.log("gyro", s.tUs, s.x / 1e6, s.y / 1e6, s.z / 1e6);
    }
  });
  await gyro.startNotifications();
}
```

---

## 8. Behavior & edge cases

- **Variable batch size:** always read byte 0 (`count`); never hard-code
  samples-per-notification. Expect 1 at very short connection intervals, more
  at longer ones.
- **Rate vs interval:** at 1600 Hz and a 7.5 ms interval you'd see ~12
  samples/notification; at sub-millisecond intervals ~1. Compute the effective
  rate from `t_us` deltas.
- **Dropped samples:** if the central cannot keep up (link too slow for the
  sample rate), the device drops samples at the source — `t_us` deltas will
  show a gap larger than the nominal sample period. There is no sequence
  number; use `t_us` to detect gaps.
- **Timestamp wrap:** `t_us` wraps ~every 71.6 min (see §5); use unsigned
  32-bit deltas.
- **Aligning accel & gyro:** match samples across the two characteristics by
  equal `t_us`.
- **Reconnection:** CCCD subscriptions are not persisted (no bonding by
  default); re-subscribe after every reconnect. Streaming auto-stops on
  disconnect.
- **One or both:** subscribing to only accel or only gyro streams just that
  characteristic; the other stays silent.

---

## 9. Quick reference

```
Device name        : nRF54L15 Tag         (128-bit motion UUID not advertised)
Service            : f0de1b00-9b0f-4a3e-8b1a-2f9c0d5e7a10
  Accel  (Notify)  : f0de1b01-9b0f-4a3e-8b1a-2f9c0d5e7a10
  Gyro   (Notify)  : f0de1b02-9b0f-4a3e-8b1a-2f9c0d5e7a10
  Config (R/W)     : f0de1b03-9b0f-4a3e-8b1a-2f9c0d5e7a10

Subscribe (CCCD) to accel/gyro to start; unsubscribe both to stop.

Stream packet (little-endian):
  [uint8 count][ count × { int32 x, int32 y, int32 z, uint32 t_us } ]
  length = 1 + count*16 ; count ≤ min(30, (ATT_MTU-4)/16)

Units:
  accel x/y/z : micro-m/s²  (÷1e6 → m/s², ÷9.80665e6 → g)
  gyro  x/y/z : micro-rad/s (÷1e6 → rad/s, ×57.29578/1e6 → °/s)
  t_us        : uint32 µs uptime (wraps ~71.6 min; use unsigned deltas)

Config write (5 bytes LE): [uint8 field_id][uint32 value]
  0=poll_rate Hz(0=track ODR)  1=accel_odr Hz  2=gyro_odr Hz
  3=accel_range g              4=gyro_range dps
Config read (20 bytes LE): uint32 poll, accel_odr, gyro_odr, accel_range, gyro_range
```
