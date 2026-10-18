# MMDVM Serial Interface

This document describes the binary serial protocol spoken between the MMDVM
firmware (the modem) and a host application (for example `MMDVMHost`,
`MMDVM_CM` or a custom controller). The implementation lives in
`SerialPort.cpp` / `SerialPort.h`; opcode constants are defined at the top of
`SerialPort.cpp` and the mode enumeration in `Globals.h`.

## Physical layer

| Property | Value |
|----------|-------|
| Transport | UART, 8 data bits, no parity, 1 stop bit (8N1) |
| Baud rate | `SERIAL_SPEED` from `Config.h` (default `460800`; `SerialPort.h` falls back to `115200`) |
| Flow control | None; the host is expected to poll `GET_STATUS` before sending mode data |

Some builds expose additional UARTs: `SERIAL_REPEATER` (a transparent serial
bridge, normally 9600 baud) and `I2C_REPEATER` (an I2C bridge). Both carry data
wrapped in the same frame format described below.

## Frame format

Every frame begins with the start byte `0xE0`. Two length encodings are used.

**Short form** (total frame length 3..255 bytes):

```
+------+--------+--------+-----------------+
| 0xE0 | length | type   | payload ...     |
+------+--------+--------+-----------------+
         total    opcode   length - 3 bytes
```

**Long form** (total frame length 258..512 bytes):

```
+------+------+---------------+--------+-----------------+
| 0xE0 | 0x00 | length - 255  | type   | payload ...     |
+------+------+---------------+--------+-----------------+
                low byte        opcode   length - 4 bytes
```

- `length` is the total number of bytes in the frame, including the `0xE0`
  start byte and the length byte(s).
- Long form is selected when the frame would exceed 255 bytes; the third byte
  carries `total_length - 255`. The maximum payload is 508 bytes.
- Bytes received while no valid start/length header is present are discarded.
- If the host stops communicating, the firmware resets the parser after the
  watchdog window (`io.getWatchdog() >= 48000U`, roughly one second).

## Error codes

`NAK` responses carry an error code:

| Code | Meaning |
|------|---------|
| 0 | Success (never sent in a NAK) |
| 1 | Unknown/unsupported opcode (default case) |
| 2 | Command not valid in the current modem state, or unhandled |
| 4 | Invalid length or invalid parameter value |
| 5 | Command requires the modem to be idle (e.g. CW ID) |
| 9 | Rejected because DMR user mode is receive-only (fork extension) |

Data `write` routines may return additional non-zero codes for short buffers;
any non-zero result causes a NAK.

## Host to modem commands

| Opcode | Name | Payload |
|--------|------|---------|
| `0x00` | `MMDVM_GET_VERSION` | none |
| `0x01` | `MMDVM_GET_STATUS` | none |
| `0x02` | `MMDVM_SET_CONFIG` | configuration block (see below) |
| `0x03` | `MMDVM_SET_MODE` | 1 byte modem state |
| `0x04` | `MMDVM_SET_FREQ` | frequency data (acknowledged, otherwise ignored) |
| `0x08` | `MMDVM_CAL_DATA` | calibration data for the active calibration state |
| `0x0A` | `MMDVM_SEND_CWID` | CW ID text (only valid in `STATE_IDLE`) |
| `0x10` | `MMDVM_DSTAR_HEADER` | D-Star header |
| `0x11` | `MMDVM_DSTAR_DATA` | D-Star data |
| `0x13` | `MMDVM_DSTAR_EOT` | none |
| `0x18` | `MMDVM_DMR_DATA1` | DMR slot 1 burst |
| `0x1A` | `MMDVM_DMR_DATA2` | DMR slot 2 / DMO burst |
| `0x1C` | `MMDVM_DMR_SHORTLC` | DMR short LC |
| `0x1D` | `MMDVM_DMR_START` | 1 byte: `0x01` start, `0x00` stop |
| `0x1E` | `MMDVM_DMR_ABORT` | DMR abort data |
| `0x20` | `MMDVM_YSF_DATA` | System Fusion data |
| `0x30` | `MMDVM_P25_HDR` | P25 header |
| `0x31` | `MMDVM_P25_LDU` | P25 LDU |
| `0x40` | `MMDVM_NXDN_DATA` | NXDN data |
| `0x45` | `MMDVM_M17_LINK_SETUP` | M17 link setup frame |
| `0x46` | `MMDVM_M17_STREAM` | M17 stream frame |
| `0x49` | `MMDVM_M17_EOT` | none |
| `0x50` | `MMDVM_POCSAG_DATA` | POCSAG data |
| `0x55` | `MMDVM_AX25_DATA` | AX.25 frame |
| `0x60` | `MMDVM_FM_PARAMS1` | FM callsign/ID parameters |
| `0x61` | `MMDVM_FM_PARAMS2` | FM acknowledgement parameters |
| `0x62` | `MMDVM_FM_PARAMS3` | FM miscellaneous parameters |
| `0x63` | `MMDVM_FM_PARAMS4` | FM external audio parameters |
| `0x65` | `MMDVM_FM_DATA` | FM audio sample pairs |
| `0x80` | `MMDVM_SERIAL_DATA` | transparent serial bridge (requires `SERIAL_REPEATER`) |
| `0x81` | `MMDVM_I2C_DATA` | transparent I2C bridge (requires `I2C_REPEATER`) |
| `0x90` | `MMDVM_TRANSPARENT` | ignored by the modem |
| `0x91` | `MMDVM_QSO_INFO` | ignored by the modem |

`SET_CONFIG`, `SET_MODE`, the FM parameter commands and `CAL_DATA` are
acknowledged with `ACK`/`NAK`. For most mode-data opcodes a NAK is sent on
failure but nothing is sent on success; the text/data opcodes auto-switch the
modem into the matching state when it is currently idle.

### Transparent serial / I2C bridges

Data received in `SERIAL_DATA` / `I2C_DATA` frames is queued and emitted on the
secondary UART/I2C bus. Bytes arriving on that bus are batched and returned to
the host in `SERIAL_DATA` / `I2C_DATA` frames. The serial bridge uses a small
de-bounce (`MAX_SERIAL_COUNT`) so that bursts settle before being forwarded, and
one frame never exceeds `MAX_SERIAL_DATA` (250) bytes.

## Modem to host messages

| Opcode | Name | Payload |
|--------|------|---------|
| `0x00` | `MMDVM_GET_VERSION` | version block (see below) |
| `0x01` | `MMDVM_GET_STATUS` | status block (see below) |
| `0x08` | `MMDVM_CAL_DATA` | calibration data |
| `0x09` | `MMDVM_RSSI_DATA` | RSSI values |
| `0x10` | `MMDVM_DSTAR_HEADER` | D-Star header |
| `0x11` | `MMDVM_DSTAR_DATA` | D-Star data |
| `0x12` | `MMDVM_DSTAR_LOST` | none |
| `0x13` | `MMDVM_DSTAR_EOT` | none |
| `0x18` | `MMDVM_DMR_DATA1` | DMR slot 1 burst |
| `0x19` | `MMDVM_DMR_LOST1` | none |
| `0x1A` | `MMDVM_DMR_DATA2` | DMR slot 2 burst |
| `0x1B` | `MMDVM_DMR_LOST2` | none |
| `0x20` | `MMDVM_YSF_DATA` | System Fusion data |
| `0x21` | `MMDVM_YSF_LOST` | none |
| `0x30` | `MMDVM_P25_HDR` | P25 header |
| `0x31` | `MMDVM_P25_LDU` | P25 LDU |
| `0x32` | `MMDVM_P25_LOST` | none |
| `0x40` | `MMDVM_NXDN_DATA` | NXDN data |
| `0x41` | `MMDVM_NXDN_LOST` | none |
| `0x45` | `MMDVM_M17_LINK_SETUP` | M17 link setup frame |
| `0x46` | `MMDVM_M17_STREAM` | M17 stream frame |
| `0x48` | `MMDVM_M17_LOST` | none |
| `0x49` | `MMDVM_M17_EOT` | none |
| `0x50` | `MMDVM_POCSAG_DATA` | POCSAG data |
| `0x55` | `MMDVM_AX25_DATA` | AX.25 frame |
| `0x65` | `MMDVM_FM_DATA` | FM audio sample pairs |
| `0x66` | `MMDVM_FM_STATUS` | 1 byte FM state |
| `0x67` | `MMDVM_FM_EOT` | none |
| `0x70` | `MMDVM_ACK` | 1 byte: opcode being acknowledged |
| `0x7F` | `MMDVM_NAK` | 2 bytes: opcode, error code |
| `0x80` | `MMDVM_SERIAL_DATA` | transparent serial bridge data |
| `0x81` | `MMDVM_I2C_DATA` | transparent I2C bridge data |
| `0xF1`..`0xF5` | `MMDVM_DEBUG1`..`DEBUG5` | null-terminated text plus 0..4 big-endian `int16_t` values |
| `0xFA` | `MMDVM_DEBUG_DUMP` | raw debug bytes |

### ACK / NAK

```
ACK : 0xE0 0x04 0x70 <opcode>
NAK : 0xE0 0x05 0x7F <opcode> <error>
```

### GET_STATUS response

The status frame is always 20 bytes long (17-byte payload). Field offsets are
relative to the start of the payload (the byte immediately after the opcode):

| Offset | Field |
|--------|-------|
| 0 | Current modem state (see mode table) |
| 1 | Status bits: `0x01` TX active, `0x02` ADC overflow, `0x04` RX overflow, `0x08` TX overflow, `0x10` lockout, `0x20` DAC overflow, `0x40` DCD (carrier detect) |
| 2 | Reserved (`0x00`) |
| 3 | D-Star TX buffer space |
| 4 | DMR slot 1 TX buffer space |
| 5 | DMR slot 2 TX buffer space (or DMO space) |
| 6 | YSF TX buffer space |
| 7 | P25 TX buffer space |
| 8 | NXDN TX buffer space |
| 9 | M17 TX buffer space |
| 10 | FM TX buffer space |
| 11 | POCSAG TX buffer space |
| 12 | AX.25 TX buffer space |
| 13..16 | Reserved (`0x00`) |

Buffer-space values report how many frames may be queued for transmission. A
value of `0` for a disabled mode.

### GET_VERSION response

The version frame has a variable length of `23 + strlen(HARDWARE)` bytes.
Field offsets are relative to the payload (byte after the opcode):

| Offset | Field |
|--------|-------|
| 0 | Protocol version (currently `2`) |
| 1 | Mode capability bits: `0x01` D-Star, `0x02` DMR, `0x04` YSF, `0x08` P25, `0x10` NXDN, `0x20` M17, `0x40` FM |
| 2 | Mode capability bits (continued): `0x01` POCSAG, `0x02` AX.25 |
| 3 | CPU type: `0` Atmel ARM, `1` NXP ARM, `2` ST-Micro ARM |
| 4..19 | 16-byte UDID |
| 20.. | Hardware description string (for example `MMDVM <version> <TCXO> <build>`) |

## SET_CONFIG payload

Minimum length is 37 bytes. All multi-bit fields are single bytes.

| Offset | Field |
|--------|-------|
| 0 | Flags: `0x01` RX invert, `0x02` TX invert, `0x04` PTT invert, `0x08` YSF low deviation, `0x10` debug enabled, `0x20` use COS as lockout, `0x40` DMR user mode (fork extension), `0x80` simplex (clears duplex) |
| 1 | Mode enable: `0x01` D-Star, `0x02` DMR, `0x04` YSF, `0x08` P25, `0x10` NXDN, `0x20` FM, `0x40` M17 |
| 2 | Mode enable: `0x01` POCSAG, `0x02` AX.25 |
| 3 | TX delay (0..50, rejected above) |
| 4 | Initial modem state |
| 5 | TX DC offset (value + 128) |
| 6 | RX DC offset (value + 128) |
| 7 | RX level |
| 8 | CW ID TX level |
| 9 | D-Star TX level |
| 10 | DMR TX level |
| 11 | YSF TX level |
| 12 | P25 TX level |
| 13 | NXDN TX level |
| 14 | M17 TX level |
| 15 | POCSAG TX level |
| 16 | FM TX level |
| 17 | AX.25 TX level |
| 18..19 | Reserved |
| 20 | YSF TX hang time |
| 21 | P25 TX hang time |
| 22 | NXDN TX hang time |
| 23 | M17 TX hang time |
| 24..25 | Reserved |
| 26 | DMR colour code (0..15, rejected above) |
| 27 | DMR delay |
| 28 | AX.25 RX twist (value + 128, range -4..10, rejected outside) |
| 29 | AX.25 TX delay |
| 30 | AX.25 slot time |
| 31 | AX.25 p-persist |
| 32..36 | Reserved |

Configuration is rejected (NAK code 4) if the requested `modemState` is unknown,
if it selects a mode that was not enabled in the same frame, if a field is out
of range, or (fork extension) if DMR user mode is requested without simplex DMR.

### FM parameter payloads

| Opcode | Layout |
|--------|--------|
| `0x60` `FM_PARAMS1` | speed, frequency/10, time, holdoff, high level, low level, flags (`0x01` call at start, `0x02` call at end, `0x04` call at latch), then callsign text (min 8 bytes) |
| `0x61` `FM_PARAMS2` | speed, frequency/10, min time, delay/10, level, then acknowledgement text (min 6 bytes) |
| `0x62` `FM_PARAMS3` | timeout/5, timeout level, CTCSS frequency, CTCSS high threshold, CTCSS low threshold, CTCSS level, kerchunk time, hang time, flags (`&0x0F` access mode, `0x20` link mode, `0x40` noise squelch, `0x80` COS invert), RF audio boost, max deviation, RX level, squelch high threshold, squelch low threshold (min 14 bytes) |
| `0x63` `FM_PARAMS4` | audio boost, speed, frequency/10, level, then acknowledgement text (min 4 bytes) |

## Modem states

`SET_MODE` and `SET_CONFIG` carry one of the following `MMDVM_STATE` values
(`Globals.h`):

| Value | State |
|-------|-------|
| 0 | `STATE_IDLE` |
| 1 | `STATE_DSTAR` |
| 2 | `STATE_DMR` |
| 3 | `STATE_YSF` |
| 4 | `STATE_P25` |
| 5 | `STATE_NXDN` |
| 6 | `STATE_POCSAG` |
| 7 | `STATE_M17` |
| 10 | `STATE_FM` |
| 11 | `STATE_AX25` (informational; not settable via `SET_MODE`) |
| 91 | `STATE_NXDNCAL1K` |
| 92 | `STATE_DMRDMO1K` |
| 93 | `STATE_P25CAL1K` |
| 94 | `STATE_DMRCAL1K` |
| 95 | `STATE_LFCAL` |
| 96 | `STATE_RSSICAL` |
| 97 | `STATE_CWID` |
| 98 | `STATE_DMRCAL` |
| 99 | `STATE_DSTARCAL` |
| 100 | `STATE_INTCAL` |
| 101 | `STATE_POCSAGCAL` |
| 102..107 | `STATE_FMCAL10K` .. `STATE_FMCAL30K` |
| 108 | `STATE_M17CAL` |

Calibration states consume `CAL_DATA` frames and produce `CAL_DATA`/`RSSI_DATA`
responses. `STATE_RSSICAL` reports via `RSSI_DATA`, all other calibration states
via `CAL_DATA`.

## Fork extension: DMR user mode

This fork adds a receive-only "DMR user mode" that lets a dual-slot repeater be
monitored for DMR data and audio.

- Enabled by setting bit `0x40` of the `SET_CONFIG` flags byte, together with
  simplex DMR (`simplex` set in flags bit `0x80` and DMR enabled).
- The host keeps the modem in `STATE_DMR`. Because the mode is receive-only,
  `MMDVM_DMR_DATA1` and `MMDVM_DMR_DATA2` frames from the host are rejected
  with NAK error code `9`, and the DMR transmit stack is not processed.
- Received traffic is emitted through the normal `MMDVM_DMR_DATA1/2` and
  `MMDVM_DMR_LOST1/2` messages, tagged with the configured colour code.
- The capability/status framing is otherwise unchanged, so existing hosts can
  discover the modem as a normal DMR device.

## Timing and housekeeping

- The host should send `GET_STATUS` regularly. Each status request resets the
  firmware watchdog; the parser is reset if the host is silent for about a
  second.
- TX buffer-space values returned by `GET_STATUS` should be checked before
  queueing new data frames so that the host does not overrun the modem.
- Debug output (`0xF1`..`0xFA`) is only produced when the debug flag is set in
  `SET_CONFIG`.
