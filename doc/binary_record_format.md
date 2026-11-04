# Binary record format

The recorder writes `mobili_<number>.record` in this format; `common/reader/binary_record_reader.h`
reads it. The older text format is read by
`common/reader/txt_record_reader.h`.

## File layout

A file is a 9-byte header followed by framed records. All multibyte values are little-endian and
written field by field, without C struct padding.

| Offset | Size | Value |
|---|---:|---|
| 0 | 8 | `4D 4F 4B 55 52 45 43 00` (ASCII `MOKUREC` plus NUL) |
| 8 | 1 | Format version `01` |

## Records

Every record is: sync bytes `A5 5A`, a 1-byte type, a signed 64-bit boot timestamp in microseconds
(same meaning as in the text format), the payload, and a CRC-16.

| Type | Payload, in order | Record size |
|---|---|---:|
| `01` IMU | `ax`, `ay`, `az` (m/s²), `gx`, `gy`, `gz` (rad/s): six binary32 | 37 bytes |
| `02` Speed | wheel speed (km/h): int32, `255` = unavailable | 17 bytes |
| `03` GPS | fix age (ms): uint32; latitude, longitude (deg): binary64; altitude (m), speed (m/s): binary32; satellite count: uint8 | 42 bytes |
| `04` Long IMU | the six `01` values, then `mx`, `my`, `mz` (milligauss), temperature (°C), pressure (Pa): eleven binary32 | 57 bytes |

Recorders without a magnetometer and barometer write IMU records (`01`); those with them write long
IMU records (`04`).

| Type | Offset | Field |
|---|---:|---|
| All | 0–1 | Sync bytes `A5 5A` |
| All | 2 | Type |
| All | 3–10 | Boot timestamp (µs), int64 |
| IMU | 11–34 | `ax`, `ay`, `az`, `gx`, `gy`, `gz` |
| IMU | 35–36 | CRC-16 |
| Speed | 11–14 | Speed, int32 |
| Speed | 15–16 | CRC-16 |
| GPS | 11–14 | Fix age, uint32 |
| GPS | 15–22 | Latitude, binary64 |
| GPS | 23–30 | Longitude, binary64 |
| GPS | 31–34 | Altitude, binary32 |
| GPS | 35–38 | Speed, binary32 |
| GPS | 39 | Satellite count, uint8 |
| GPS | 40–41 | CRC-16 |
| Long IMU | 11–34 | `ax`, `ay`, `az`, `gx`, `gy`, `gz`, as in IMU |
| Long IMU | 35–46 | `mx`, `my`, `mz` |
| Long IMU | 47–50 | Temperature |
| Long IMU | 51–54 | Pressure |
| Long IMU | 55–56 | CRC-16 |

## CRC-16

CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value `0xFFFF`, no reflection, final XOR `0x0000`.
It covers offsets 2 up to the CRC (type, timestamp, payload; not the sync bytes) and is stored
little-endian.

## Writer conventions

- IMU values are binary32 because the writer receives C `float`.
- GPS altitude and speed are rounded from `double` to binary32; satellite counts above 255 are
  clamped to 255.
- Records keep their write order; several may share a timestamp.

## Reader behaviour

`ReadBinaryRecord` returns `nullopt` if the file cannot be opened or the header (magic or version)
does not match. On a bad sync, unknown type, CRC mismatch or truncated tail it advances one byte and
searches for the next `A5 5A`, counting the CRC mismatches and skipped bytes. Timestamps become
nanoseconds, wheel speed m/s; speed `255` is dropped. A long IMU record fills
`sensor::ImuData::magnetic_field_mgauss` and adds a `sensor::BarometerData` (temperature, pressure).
