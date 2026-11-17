"""Stream decoder and protocol auto-detection for laser rangefinder / single-point LiDAR UART frames.

Python port of docs/decoder.js; both are tested against tests/vectors.json.
Every profile cites the public manual it was built from (see docs/PROTOCOLS.md).
"""
import re

__all__ = ["PROFILES", "COMMANDS", "BAUD_RATES", "Decoder", "decode", "detect", "parse_hex", "crc16ccitt"]


def sum8(b, s=0, e=None):
    return sum(b[s:len(b) if e is None else e]) & 0xFF


def inv8(b, s, e):
    return (~sum8(b, s, e)) & 0xFF


def neg8(b, s, e):
    return (0x100 - sum8(b, s, e)) & 0xFF


def u16le(f, o):
    return f[o] | (f[o + 1] << 8)


def u16be(f, o):
    return (f[o] << 8) | f[o + 1]


def h2(n):
    return "0x%02X" % n


def hexs(b):
    return " ".join("%02X" % x for x in b)


def crc16ccitt(data, s=0, e=None):
    """LightWare binary protocol CRC (CRC-16-CCITT 0x1021, init 0), per the SF30/D Product Guide sample code."""
    crc = 0
    for x in data[s:len(data) if e is None else e]:
        code = (crc >> 8) & 0xFF
        code ^= x
        code ^= code >> 4
        crc = (crc << 8) & 0xFFFF
        crc ^= code
        code = (code << 5) & 0xFFFF
        crc ^= code
        code = (code << 7) & 0xFFFF
        crc ^= code
    return crc & 0xFFFF


# Parser contract: parse(buf, i, p) -> None (need more bytes) | False (no frame) | (len, ok, kind, info)

ERDI_B_CMDS = {0x0D: "read serial no.", 0x16: "read software version", 0x06: "set baud rate", 0x0B: "set frequency",
               0x1B: "read frequency", 0x1F: "switch to I2C", 0x0A: "start/stop ranging"}


def parse_erdi_b(buf, i, p):
    n = len(buf) - i
    if buf[i] == 0x5C:
        L = p["len"]
        if n < L:
            return None
        ok = inv8(buf, i + 1, i + L - 1) == buf[i + L - 1]
        raw = int.from_bytes(bytes(buf[i + 1:i + L - 1]), "little")
        oor = p["oor"] is not None and raw >= p["oor"]
        return L, ok, "measurement", dict(raw=raw, distance_m=None if oor else raw * p["unit"], out_of_range=oor,
                                         note="out-of-range marker %d" % p["oor"] if oor else "")
    if buf[i] == 0x5A:
        if n < 3:
            return None
        cmd = buf[i + 1]
        tried = list(dict.fromkeys([buf[i + 2], 4, 2]))
        for dl in tried:
            L = 4 + dl
            if dl > 8 or n < L:
                continue
            if inv8(buf, i + 1, i + L - 1) == buf[i + L - 1]:
                reply = bool(cmd & 0x80)
                name = ERDI_B_CMDS.get(cmd & 0x7F, "command " + h2(cmd & 0x7F))
                return L, True, "reply" if reply else "request", dict(note=("reply: " if reply else "request: ") + name)
        return None if n < 4 + max(d for d in tried if d <= 8) else False
    return False


ERDI_A_FN = {0x88: "single-shot ranging", 0x89: "continuous ranging", 0x8E: "stop ranging", 0x8A: "angle measurement",
             0x80: "self-test", 0x86: "LD constant-on", 0x70: "external circuit enable", 0x71: "external circuit disable"}


def parse_erdi_a(buf, i, p):
    if buf[i] != 0x55:
        return False
    if len(buf) - i < 2:
        return None
    if buf[i + 1] != 0xAA:
        return False
    if len(buf) - i < 8:
        return None
    f = buf[i:i + 8]
    fn = f[2]
    name = ERDI_A_FN.get(fn, "function " + h2(fn))
    if sum8(f, 0, 7) == f[7]:
        info = dict(function=fn, status=f[3], note="reply: " + name)
        if fn in (0x88, 0x89):
            raw = u16be(f, 5)
            good = f[3] == 1
            info.update(raw=raw, distance_m=raw * p["unit"] if good else None, out_of_range=not good,
                        note="" if good else "STA=%d (measurement failed / no echo)" % f[3])
            return 8, True, "measurement", info
        if fn == 0x8A:
            info.update(raw=u16be(f, 5), note="angle %.1f deg" % (u16be(f, 5) * 0.1))
        if fn == 0x80:
            info["note"] = "self-test, error code candidates D3=%s D4=%s" % (h2(f[5]), h2(f[6]))
        return 8, True, "reply", info
    if sum8(f, 2, 7) == f[7]:
        return 8, True, "request", dict(function=fn, note="request: " + name)
    return 8, False, "measurement", dict(note="reply checksum must be SUM(55..D4); request checksum SUM(fn..D4)")


def benewake_info(p, dist, strength, temp):
    info = dict(raw=dist, distance_m=dist * p["unit"], strength=strength, out_of_range=False, note="")
    if p["temp"]:
        info["temp_c"] = temp / 8 - 256
    if not p["strength"]:
        del info["strength"]
    m = p["model"]
    if m == "tfmini-s":
        codes = {65535: "-1: strength < 100", 65534: "-2: signal strength saturation", 65532: "-4: ambient light saturation"}
        if dist in codes:
            info.update(distance_m=None, out_of_range=True, note=codes[dist])
    elif m in ("tfmini-plus", "tf-luna"):
        if strength < 100 or strength == 65535:
            info.update(distance_m=None, out_of_range=True,
                        note="strength 65535: overexposed, distance unreliable" if strength == 65535
                        else "strength < 100: distance unreliable (dummy distance %d)" % dist)
    elif m == "tf02-pro":
        if strength < 60:
            info.update(distance_m=None, out_of_range=True, note="strength < 60: module outputs 4500 cm (out of range)")
    return info


def parse_benewake(buf, i, p):
    n = len(buf) - i
    if buf[i] == 0x59:
        if n < 2:
            return None
        if buf[i + 1] != 0x59:
            return False
        if n < 9:
            return None
        ok = sum8(buf, i, i + 8) == buf[i + 8]
        return 9, ok, "measurement", benewake_info(p, u16le(buf, i + 2), u16le(buf, i + 4), u16le(buf, i + 6))
    if buf[i] == 0x5A:
        if n < 2:
            return None
        L = buf[i + 1]
        if L < 4 or L > 16:
            return False
        if n < L:
            return None
        if sum8(buf, i, i + L - 1) != buf[i + L - 1]:
            return False
        return L, True, "request", dict(note="command/response ID " + h2(buf[i + 2]))
    return False


_ASCII_RE = re.compile(r"^\s*([-+]?\d+(?:\.\d+)?)(?:\s*m\b)?(.*)$")


def parse_ascii_metres(buf, i, p):
    c = buf[i]
    if not (0x30 <= c <= 0x39 or c in (0x2D, 0x20, 0x2B)):
        return False
    if i > 0 and buf[i - 1] != 0x0A:
        return False
    j = i
    while j < len(buf) and j - i < 48 and buf[j] != 0x0A:
        b = buf[j]
        if b != 0x0D and (b < 0x20 or b > 0x7E):
            return False
        j += 1
    if j >= len(buf):
        return None if j - i < 48 else False
    if buf[j] != 0x0A:
        return False
    line = bytes(buf[i:j]).decode("ascii").rstrip("\r")
    m = _ASCII_RE.match(line)
    if not m:
        return False
    v = float(m.group(1))
    return j - i + 1, True, "measurement", dict(raw=m.group(1), distance_m=v, out_of_range=v < 0, note=m.group(2).strip())


def parse_lightware(buf, i, p):
    n = len(buf) - i
    if buf[i] != 0xAA:
        return False
    if n < 3:
        return None
    flags = u16le(buf, i + 1)
    if flags & 0x3E:
        return False
    plen = flags >> 6
    if plen < 1 or plen > 1023:
        return False
    L = 3 + plen + 2
    if n < L:
        return None
    ok = crc16ccitt(buf, i, i + L - 2) == u16le(buf, i + L - 2)
    cid = buf[i + 3]
    info = dict(command=cid, note="command %d%s, %d-byte payload" % (cid, " (write)" if flags & 1 else "", plen))
    kind = "reply"
    if cid == 44 and plen >= 3:
        vals = []
        k = i + 4
        while k + 1 < i + 3 + plen:
            v = u16le(buf, k)
            vals.append(v - 0x10000 if v & 0x8000 else v)
            k += 2
        info.update(raw=vals[0], distance_m=vals[0] / 100, out_of_range=False, values=vals,
                    note="(44) distance data, int16 fields %s - order set by command (29) distance output" % ", ".join(map(str, vals)))
        kind = "measurement"
    return L, ok, kind, info


def parse_jrt(buf, i, p):
    n = len(buf) - i
    if buf[i] != 0xAA:
        return False
    if n < 6:
        return None
    reg = u16be(buf, i + 2)
    cnt = u16be(buf, i + 4)
    if cnt < 1 or cnt > 16:
        return False
    L = 7 + 2 * cnt
    if n < L:
        return None
    ok = sum8(buf, i + 1, i + L - 1) == buf[i + L - 1]
    meas = reg == 0x22 and cnt == 3
    if not ok and not meas:
        return False
    if meas:
        mm = int.from_bytes(bytes(buf[i + 6:i + 10]), "big")
        sq = u16be(buf, i + 10)
        return L, ok, "measurement", dict(raw=mm, distance_m=mm / 1000, strength=sq, out_of_range=False, note="strength = JRT signal quality")
    if reg == 0x20:
        mode = buf[i + 7]
        return L, ok, "request", dict(note="request: start measurement, mode %s%s" % (h2(mode), " (continuous fast)" if mode == 6 else ""))
    return L, ok, "reply", dict(note="register %s, %d word(s)" % (h2(reg), cnt))


def parse_tofsense(buf, i, p):
    n = len(buf) - i
    if buf[i] != 0x57:
        return False
    if n < 2:
        return None
    if buf[i + 1] == 0x10:
        if n < 8:
            return None
        if sum8(buf, i, i + 7) != buf[i + 7]:
            return False
        return 8, True, "request", dict(note="query frame for module id %d" % buf[i + 4])
    if buf[i + 1] != 0x00:
        return False
    if n < 16:
        return None
    ok = sum8(buf, i, i + 15) == buf[i + 15]
    dis = buf[i + 8] | (buf[i + 9] << 8) | (buf[i + 10] << 16)
    if dis & 0x800000:
        dis -= 0x1000000
    status = buf[i + 11]
    strength = u16le(buf, i + 12)
    t = int.from_bytes(bytes(buf[i + 4:i + 8]), "little")
    return 16, ok, "measurement", dict(id=buf[i + 3], raw=dis, distance_m=dis / 1000, strength=strength, status=status,
                                      out_of_range=False, note="id %d, dis_status %d, system_time %d ms" % (buf[i + 3], status, t))


_DFR_RE = re.compile(r"^[0-9.ER-]+$")


def parse_dfr80m(buf, i, p):
    n = len(buf) - i
    if n < 3:
        return None
    if buf[i + 1] != 0x06:
        return False
    fn = buf[i + 2]
    if fn in (0x02, 0x03, 0x07):
        if n < 4:
            return None
        if neg8(buf, i, i + 3) != buf[i + 3]:
            return False
        return 4, True, "request", dict(note="request: " + {2: "single measurement", 3: "continuous measurement", 7: "read cache"}[fn])
    if fn not in (0x82, 0x83):
        return False
    for dl in (7, 8):
        L = 4 + dl
        if n < L:
            return None
        text = bytes(buf[i + 3:i + 3 + dl]).decode("latin-1")
        if not _DFR_RE.match(text) or neg8(buf, i, i + L - 1) != buf[i + L - 1]:
            continue
        if text.startswith("ER"):
            return L, True, "measurement", dict(raw=text, distance_m=None, out_of_range=True, note="error " + text)
        return L, True, "measurement", dict(raw=text, distance_m=float(text), out_of_range=False, note="0.1 mm resolution" if dl == 8 else "")
    return (11, False, "measurement", dict(note="CS = (~sum + 1) of all preceding bytes")) if n >= 12 else None


BW_SRC = ["https://cdn.sparkfun.com/assets/8/a/f/a/c/16977-TFMini-S_-_Micro_LiDAR_Module-Product_Manual.pdf"]
ERDI_B_SRC = ["https://github.com/ErdiTech/erdi-lrf-drivers/blob/main/docs/protocol-B-5C.md"]


def _bw(model, label, **extra):
    d = dict(vendor="Benewake", family="benewake-9", label=label, baud=[115200], unit=0.01, strength=True, temp=True,
             model=model, strong=True, parse=parse_benewake, source=BW_SRC)
    d.update(extra)
    return d


def _erdi_b(label, length, unit, oor):
    return dict(vendor="ERDI", family="erdi-5c-%d" % length, label=label, baud=[460800, 115200], len=length, unit=unit,
                oor=oor, strong=False, parse=parse_erdi_b, source=ERDI_B_SRC)


PROFILES = {
    "erdi-b-4-cm": _erdi_b("ERDI H series LRF50H/100H/200H/300H/600H (5C, cm)", 4, 0.01, 65535),
    "erdi-b-4-mm": _erdi_b("ERDI VB series, generic (5C, mm)", 4, 0.001, None),
    "erdi-lrf22vb": _erdi_b("ERDI LRF22VB (5C, mm, 20000 = out of range)", 4, 0.001, 20000),
    "erdi-lrf50vb": _erdi_b("ERDI LRF50VB (5C, mm, 50000 = out of range)", 4, 0.001, 50000),
    "erdi-b-5-cm": _erdi_b("ERDI LRF1500H (5C, 5-byte, cm)", 5, 0.01, 16777215),
    "erdi-a": dict(vendor="ERDI", family="erdi-55aa", label="ERDI LR / LRF-A1 / SPD (55 AA, metres x10)", baud=[115200],
                   unit=0.1, strong=True, parse=parse_erdi_a,
                   source=["https://github.com/ErdiTech/erdi-lrf-drivers/blob/main/docs/protocol-A-55AA.md"]),
    "benewake-tf-9": _bw("generic", "Benewake 9-byte, model unknown (59 59, cm)"),
    "benewake-tfmini-s": _bw("tfmini-s", "Benewake TFmini-S (59 59, cm, -1/-2/-4 codes)"),
    "benewake-tfmini-plus": _bw("tfmini-plus", "Benewake TFmini Plus (59 59, cm)", source=["https://m.media-amazon.com/images/I/A1UADgRvNzL.pdf"]),
    "benewake-tf-luna": _bw("tf-luna", "Benewake TF-Luna (59 59, cm)", source=["https://cdn.compacttool.ru/downloads/TF-Luna%20Datasheet.pdf"]),
    "benewake-tf02-pro": _bw("tf02-pro", "Benewake TF02-Pro (59 59, cm)",
                             source=["https://evelta.com/content/datasheets/BP-UM-26%20A02%20TF02-Pro%20User%20Manual.pdf"]),
    "benewake-tf03": _bw("tf03", "Benewake TF03 (59 59, cm, bytes 4-7 reserved)", strength=False, temp=False,
                         source=["https://www.worldronemarket.com/benewake-tf03-rangefinder-180-m-user-manual/"]),
    "benewake-tf-9-mm": _bw("generic-mm", "Benewake 9-byte mm output mode (59 59, mm)", unit=0.001),
    "ascii-metres": dict(vendor="ASCII", family="ascii", label="ASCII metres per line: LightWare SF10/SF11 serial, Benewake string/Pixhawk mode",
                         baud=[115200, 9600, 19200, 38400, 57600], strong=False, parse=parse_ascii_metres,
                         source=["https://acroname.com/sites/default/files/assets/sf11_-_laser_altimeter_manual_rev_0.pdf"] + BW_SRC),
    "lightware-binary": dict(vendor="LightWare", family="lightware-binary", label="LightWare binary protocol (AA, CRC-16; SF30/D distance ID 44)",
                             baud=[115200, 921600], strong=False, parse=parse_lightware,
                             source=["https://lightwarelidar.com/wp-content/uploads/2026/03/SF30D-Product-Guide-v3.3-1.pdf"]),
    "jrt-aa": dict(vendor="JRT", family="jrt-aa", label="Chengdu JRT laser distance sensor (AA 00 00 22, mm)", baud=[19200, 115200, 9600, 38400],
                   strong=False, parse=parse_jrt, source=["https://www.jrt-measure.com/serial-laser-distance-sensor/60034347.html"]),
    "nooploop-tofsense": dict(vendor="Nooploop", family="nooploop-tofsense", label="Nooploop TOFSense / TOFSense-F (57 00, 16-byte)",
                              baud=[921600, 115200], strong=True, parse=parse_tofsense,
                              source=["https://support.nooploop.com/tofsense/protocol/",
                                      "https://files.seeedstudio.com/products/101990702/TOFSense_User_Manual_V2.0_EN.pdf"]),
    "dfrobot-sen0366": dict(vendor="DFRobot", family="dfrobot-80m", label="DFRobot SEN0366 / 80 m laser module (80 06 83 ASCII)", baud=[9600],
                            strong=True, parse=parse_dfr80m,
                            source=["https://dfimg.dfrobot.com/nobody/wiki/068db268ba37a41067c1b17607932139.pdf"]),
}

COMMANDS = {
    "erdi-a": [("Single-shot ranging", "55 AA 88 FF FF FF FF 84"), ("Continuous ranging", "55 AA 89 FF FF FF FF 85"),
               ("Stop ranging", "55 AA 8E FF FF FF FF 8A")],
    "erdi-b-4-cm": [("Read software version", "5A 16 02 16 16 BB"), ("Stop ranging (LRF50H)", "5A 0A 02 00 00 F3"),
                    ("Start ranging (LRF50H)", "5A 0A 02 02 00 F1")],
    "jrt-aa": [("Continuous fast measurement", "AA 00 00 20 00 01 00 06 27"), ("Stop continuous", "58")],
    "nooploop-tofsense": [("Query module id 0", "57 10 FF FF 00 FF FF 63")],
    "dfrobot-sen0366": [("Single measurement", "80 06 02 78"), ("Continuous measurement", "80 06 03 77"), ("Read cache", "80 06 07 73")],
    "benewake-tf-9": [("Output format: standard 9-byte cm", "5A 05 05 01 65")],
}
for _k in ("erdi-b-4-mm", "erdi-lrf22vb", "erdi-lrf50vb", "erdi-b-5-cm"):
    COMMANDS[_k] = COMMANDS["erdi-b-4-cm"]
for _k, _p in PROFILES.items():
    if _p["family"] == "benewake-9":
        COMMANDS[_k] = COMMANDS["benewake-tf-9"]

BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]


class Decoder:
    """Incremental decoder: feed bytes with push(), get a list of frame dicts back."""

    def __init__(self, profile):
        if profile not in PROFILES:
            raise ValueError("unknown profile %r (see --list)" % profile)
        self.profile = profile
        self.p = PROFILES[profile]
        self.buf = bytearray()
        self.skipped = 0
        self.frames = 0
        self.bad = 0

    def push(self, data):
        self.buf += bytes(data)
        buf, p, out, i = self.buf, self.p, [], 0
        while i < len(buf):
            r = p["parse"](buf, i, p)
            if r is None:
                break
            # A checksum failure behind a weak one-byte header only counts as a damaged frame
            # when the next frame starts right after it; at the tail, wait for that byte.
            if r is not False and not r[1] and not p["strong"] and i + r[0] >= len(buf):
                break
            if r is False or (not r[1] and not p["strong"] and buf[i + r[0]] != buf[i]):
                i += 1
                self.skipped += 1
                continue
            L, ok, kind, info = r
            rec = dict(profile=self.profile, kind=kind, ok=ok, hex=hexs(buf[i:i + L]))
            rec.update(info)
            rec.setdefault("note", "")
            out.append(rec)
            if ok:
                self.frames += 1
                i += L
            else:
                self.bad += 1
                i += 1
        self.buf = bytearray(buf[i:])
        if len(self.buf) > 8192:
            self.buf = self.buf[-2048:]
        return out


def decode(data, profile):
    """Decode a complete capture. Returns (frames, remaining_bytes, skipped_bytes)."""
    d = Decoder(profile)
    frames = d.push(data)
    return frames, bytes(d.buf), d.skipped


def detect(data):
    """Rank protocol families by the share of the capture they explain (best first)."""
    seen, out = set(), []
    total = len(data) or 1
    for pid, p in PROFILES.items():
        if p["family"] in seen:
            continue
        seen.add(p["family"])
        d = Decoder(pid)
        frames = d.push(data)
        good = [f for f in frames if f["ok"]]
        if not good:
            continue
        covered = sum((len(f["hex"]) + 1) // 3 for f in good)
        out.append(dict(profile=pid, family=p["family"], vendor=p["vendor"], label=p["label"],
                        members=[k for k, q in PROFILES.items() if q["family"] == p["family"]],
                        frames=len(good), measurements=sum(1 for f in good if f["kind"] == "measurement"),
                        bad=d.bad, coverage=min(1.0, covered / total), baud=p["baud"]))
    out.sort(key=lambda r: (-r["coverage"], -r["measurements"], r["bad"]))
    return out


def parse_hex(text):
    tokens = re.sub(r"0x", " ", text, flags=re.I)
    tokens = re.sub(r"[^0-9a-fA-F]+", " ", tokens).split()
    data = bytearray()
    for t in tokens:
        if len(t) % 2:
            raise ValueError("odd-length hex token %r" % t)
        data += bytes.fromhex(t)
    return bytes(data)
