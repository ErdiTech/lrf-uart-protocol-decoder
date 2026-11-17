# Protocol reference

Every profile in `docs/decoder.js` and `python/lrf_decoder/core.py` is built from a public
manual or vendor page. This file lists the frame layout, checksum, units and the source for
each one. Test vectors live in [`tests/vectors.json`](../tests/vectors.json); each vector is
marked `manual` (bytes printed in the source) or `constructed` (assembled from the source's
field table and checksum rule).

Sources were checked on 2026-11-17. If your module disagrees with a table here, open an issue
with a hex capture and the page of your manual.

## Benewake TF series: 9-byte frame

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|---|
| Field | `0x59` | `0x59` | Dist_L | Dist_H | Strength_L | Strength_H | Temp_L | Temp_H | Checksum |

- Checksum: low 8 bits of the sum of bytes 0-7.
- Distance: cm by default, little-endian. The TFmini-S "Standard 9 bytes (mm)" output format
  (command `5A 05 05 06 6A`) uses the same frame in mm: profile `benewake-tf-9-mm`.
- Temperature: `Temp / 8 - 256` degrees C (TFmini-S, TFmini Plus, TF-Luna, TF02-Pro).
- Default baud: 115200.
- Commands: `5A Len ID Data.. Checksum`, Len = whole frame length, checksum = low byte of the
  sum of the preceding bytes. Recognised and labelled, not interpreted.

Invalid-reading rules differ per model, which is why there is one profile per model:

| Profile | Model | Rule from the manual | Source |
|---|---|---|---|
| `benewake-tfmini-s` | TFmini-S | Dist 65535 (-1): strength < 100; 65534 (-2): signal strength saturation; 65532 (-4): ambient light saturation | [Product Manual of TFmini-S, SJ-PM-TFmini-S A01 (SparkFun mirror)](https://cdn.sparkfun.com/assets/8/a/f/a/c/16977-TFMini-S_-_Micro_LiDAR_Module-Product_Manual.pdf), tables 7 and 8 |
| `benewake-tfmini-plus` | TFmini Plus | "When the signal strength is lower than 100 or equal to 65535 (overexposed), the detection is unreliable, TFmini Plus will set distance value to 0." | [SJ-PM-TFmini Plus A05](https://m.media-amazon.com/images/I/A1UADgRvNzL.pdf) |
| `benewake-tf-luna` | TF-Luna | "Dist value is unreliable when Amp < 100 or Amp = 65535 (Overexposure)"; Dist set to dummy_dist (default 0) | [SJ-PM-TF-Luna A05](https://cdn.compacttool.ru/downloads/TF-Luna%20Datasheet.pdf) |
| `benewake-tf02-pro` | TF02-Pro | "when the signal strength is insufficient and lower than 60, the distance value will become the maximum value of 4500" | [TF02-Pro User Manual BP-UM-26 A02](https://evelta.com/content/datasheets/BP-UM-26%20A02%20TF02-Pro%20User%20Manual.pdf) |
| `benewake-tf03` | TF03 | bytes 4-7 reserved (no strength or temperature) | [TF03 user manual, table 4 (reseller copy)](https://www.worldronemarket.com/benewake-tf03-rangefinder-180-m-user-manual/) |
| `benewake-tf-9` | any / unknown, incl. original TFmini | frame and checksum only | [Product Manual of TFmini (52Pi mirror)](https://wiki.52pi.com/images/a/a9/TFmini_Lidar_Module_Product_Mannual_EN.pdf) |

The TF03 page prints the checksum as "Byte0 + Byte2 + ... + Byte7"; every other TF manual sums
bytes 0-7. The decoder sums bytes 0-7. If a real TF03 capture fails every checksum by exactly
`0x59`, please open an issue.

Benewake mirrors: Benewake's own download centre needs a login, so the links above are vendor
manuals republished by distributors. Document numbers are quoted so you can match them against
your copy.

## ASCII metres per line

Profile `ascii-metres`. One decimal number in metres per line, terminated by `\r\n`; anything
after the number is kept as a note.

| Device | Example | Source |
|---|---|---|
| LightWare SF10 / SF11 serial output | `22.48 m 1.045 V\r\n` (altitude in metres, two decimals, then the analog voltage) | [SF11 Laser Altimeter Manual Rev 0](https://acroname.com/sites/default/files/assets/sf11_-_laser_altimeter_manual_rev_0.pdf) |
| Benewake TFmini-S character-string format | `1.21\r\n` | TFmini-S manual above, "Character string data format" |

On lost signal the SF11 outputs a configurable value (130 m by default), so a reading equal to
that setting means "no target", not a real distance.

## LightWare binary protocol

Profile `lightware-binary`. From the [SF30/D microLiDAR Product Guide v3.3](https://lightwarelidar.com/wp-content/uploads/2026/03/SF30D-Product-Guide-v3.3-1.pdf):

```
AA  flags_lo flags_hi  ID  data...  crc_lo crc_hi
```

- Start byte `0xAA`.
- Flags (16-bit, little-endian): bits 15-6 payload length (1-1023, ID byte included), bits 5-1
  reserved, bit 0 write.
- CRC: "CRC-16-CCITT 0x1021" over every byte except the CRC, initial value 0, low byte first.
  This equals CRC-16/XMODEM (check value `0x31C3` for `"123456789"`; asserted in the tests).
- Default baud 115200.
- ID 44 "Distance data cm": a list of int16 values whose order depends on command (29)
  "Distance output" (first return raw / filtered / strength, last return raw / filtered /
  strength, background noise, temperature). The decoder reports the first value as the distance
  and lists all values in the note. Other IDs are validated (CRC) and shown with their number.

The guide prints no example packet, so the two LightWare vectors are constructed with the
guide's own CRC routine.

## Chengdu JRT laser distance sensors (register protocol)

Profile `jrt-aa`. The vendor's product page for the 10 m FPC sensor (U85 / C85 class) prints a
real capture ([jrt-measure.com](https://www.jrt-measure.com/serial-laser-distance-sensor/60034347.html)):

```
T  AA 00 00 20 00 01 00 06 27                    start continuous fast measurement
R  AA 00 00 22 00 03 00 00 05 F6 00 45 65        1526 mm, signal quality 0x45
R  AA 00 00 22 00 03 00 00 05 F6 00 32 52
R  AA 00 00 22 00 03 00 00 05 F5 00 28 47
T  58                                            stop continuous mode
```

```
AA  addr  reg_hi reg_lo  cnt_hi cnt_lo  payload (cnt x 2 bytes)  checksum
```

- Checksum: low 8 bits of the sum of every byte after `0xAA` up to the checksum. All three
  printed replies and the request verify.
- Register `0x0022` with `cnt = 3`: distance as 32-bit big-endian millimetres, then a 16-bit
  big-endian signal quality.
- Baud: the same page states 19200 for that model; other JRT parts ship at other rates, so the
  scanner tries all common rates.
- Field names and the "count of 16-bit words" reading of `cnt` match the independent open-source
  drivers [cmsedore/jrt-laser-distance-sensor](https://github.com/cmsedore/jrt-laser-distance-sensor)
  (tested on a U81B) and [williamkapke/jrt](https://github.com/williamkapke/jrt).
- Not covered: JRT's long-range 905 nm / 1535 nm pulse rangefinders, which use different
  protocols that JRT does not publish.

## Nooploop TOFSense / TOFSense-F

Profile `nooploop-tofsense`. NLink_TOFSense_Frame0, 16 bytes, little-endian, default 921600 baud
([Nooploop protocol page](https://support.nooploop.com/tofsense/protocol/),
[TOFSense User Manual V2.0](https://files.seeedstudio.com/products/101990702/TOFSense_User_Manual_V2.0_EN.pdf)):

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | header `0x57` |
| 1 | 1 | function mark `0x00` |
| 2 | 1 | reserved |
| 3 | 1 | id |
| 4 | 4 | system_time, ms |
| 8 | 3 | dis x 1000 (signed 24-bit; `ad 08 00` = 2.221 m) |
| 11 | 1 | dis_status (meaning is model specific; see the model datasheet) |
| 12 | 2 | signal_strength |
| 14 | 1 | reserved |
| 15 | 1 | sum of bytes 0-14, low 8 bits |

Both printed frames (`... 03 00 06 41` and `... 03 00 ff 3a`) are test vectors. The query frame
`57 10 FF FF id FF FF sum` (UART query mode) is recognised as a request. The 400-byte
TOFSense-M multi-zone frame (`57 01`) is not decoded.

## DFRobot SEN0366 / 80 m laser module

Profile `dfrobot-sen0366`. From DFRobot's
[SEN0366 communication protocol](https://dfimg.dfrobot.com/nobody/wiki/068db268ba37a41067c1b17607932139.pdf):
9600 baud 8N1, default address `0x80`.

```
request   ADDR 06 02 CS            single measurement      (80 06 02 78)
          ADDR 06 03 CS            continuous measurement  (80 06 03 77)
          ADDR 06 07 CS            read cache              (80 06 07 73)
reply     ADDR 06 82|83 "123.456" CS     metres as ASCII, 7 chars (1 mm) or 8 chars (0.1 mm)
error     ADDR 06 82|83 "ERR--15" CS
```

CS = two's complement of the sum of all preceding bytes ("sum ... reverse and add 1").
The three printed request frames verify with this rule.

## ERDI

Built from the transcribed ERDI manuals in
[ErdiTech/erdi-lrf-drivers](https://github.com/ErdiTech/erdi-lrf-drivers/tree/main/docs). The
decoder agrees with those documents field for field.

### Family B: `5C` stream (`erdi-b-*`, `erdi-lrf22vb`, `erdi-lrf50vb`)

```
5C lo hi chk            LRF50H, LRF100H, LRF200H, LRF300H, LRF600H (cm), LRF22VB, LRF50VB (mm)
5C lo mid hi chk        LRF1500H (cm)
```

- chk = bitwise NOT of the sum of the distance bytes. Default 460800 baud, little-endian.
- Out of range: 65535 (H series), 16777215 (LRF1500H), 20000 (LRF22VB), 50000 (LRF50VB).
  Use the model profile, because 20000 is a real 20 m reading on an LRF50VB.
- `5A cmd len data.. chk` command frames and their replies (cmd | 0x80) are recognised. The
  LRF50VB manual prints a length byte of `02` with four data bytes, so the decoder accepts the
  printed form.
- The LRF1500H manual's example `5C 02 11 03 EC` does not satisfy its own checksum
  (`E9` does); the decoder follows the checksum function.
- Source: [protocol-B-5C.md](https://github.com/ErdiTech/erdi-lrf-drivers/blob/main/docs/protocol-B-5C.md).

### Family A: `55 AA` command/response (`erdi-a`)

```
55 AA fn STA FF DIS_H DIS_L SUM      reply, SUM = low byte of sum of bytes 0-6
55 AA fn FF FF FF FF SUM             request, SUM = low byte of sum of bytes 2-6
```

- Distance big-endian, value x 10 (metres); STA = 1 means success.
- Requests and replies use different checksum scopes (they differ by `0x55 + 0xAA = 0xFF`);
  the decoder accepts both and labels the direction.
- Source: [protocol-A-55AA.md](https://github.com/ErdiTech/erdi-lrf-drivers/blob/main/docs/protocol-A-55AA.md).

## Not included, and why

| Module | Reason |
|---|---|
| Garmin LIDAR-Lite v3/v4 | I2C / PWM only, no UART frame |
| TOFSense-M (multi-zone) | 400-byte image frame; out of scope for a single-point decoder |
| JRT 905 / 1535 nm pulse rangefinders | protocol not published |

Want another module? Open an issue with a hex capture and a link to the manufacturer's manual.
