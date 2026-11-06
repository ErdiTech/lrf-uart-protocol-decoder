"""Command-line frame decoder for laser rangefinder UART captures.

    python lrf_decoder.py --profile erdi-b-4-cm "5C 02 11 EC"
    python lrf_decoder.py --profile erdi-a --file capture.bin
    python lrf_decoder.py --profile erdi-b-4-mm --port COM5      # needs pyserial
"""
import argparse
import re
import sys

PROFILES = {
    "erdi-b-4-cm": dict(kind="B", length=4, unit=0.01, oor=65535, baud=460800),
    "erdi-b-4-mm": dict(kind="B", length=4, unit=0.001, oor=None, baud=460800),
    "erdi-b-5-cm": dict(kind="B", length=5, unit=0.01, oor=16777215, baud=460800),
    "erdi-a": dict(kind="A", length=8, unit=0.1, oor=None, baud=115200),
    "benewake-tf-9": dict(kind="T", length=9, unit=0.01, oor=None, baud=115200),
}


def sum8(b):
    return sum(b) & 0xFF


def decode(buf, profile):
    """Return (frames, remaining_bytes, skipped). Each frame is a dict."""
    p = PROFILES[profile]
    n, i, skipped, out = p["length"], 0, 0, []
    while i < len(buf):
        head = buf[i]
        want = {"B": 0x5C, "A": 0x55, "T": 0x59}[p["kind"]]
        if head != want:
            i += 1
            skipped += 1
            continue
        if len(buf) - i < n:
            # a partial frame at the tail may still complete
            break
        f = bytes(buf[i:i + n])
        ok, info = False, {}
        if p["kind"] == "B":
            ok = (~sum8(f[1:-1])) & 0xFF == f[-1]
            raw = int.from_bytes(f[1:-1], "little")
            oor = p["oor"] is not None and raw >= p["oor"]
            info = dict(raw=raw, distance_m=None if oor else raw * p["unit"], out_of_range=oor)
        elif p["kind"] == "A":
            ok = f[1] == 0xAA and sum8(f[:7]) == f[7]
            raw = (f[5] << 8) | f[6]
            good = f[3] == 1
            info = dict(raw=raw, distance_m=raw * p["unit"] if good else None, out_of_range=not good)
        else:
            ok = f[1] == 0x59 and sum8(f[:8]) == f[8]
            raw = f[2] | (f[3] << 8)
            info = dict(raw=raw, distance_m=raw * p["unit"], strength=f[4] | (f[5] << 8), out_of_range=False)
        if ok:
            out.append(dict(hex=f.hex(" ").upper(), **info))
            i += n
        else:
            i += 1
            skipped += 1
    return out, bytes(buf[i:]), skipped


def parse_hex(text):
    tokens = re.sub(r"0x", " ", text, flags=re.I)
    tokens = re.sub(r"[^0-9a-fA-F]+", " ", tokens).split()
    data = bytearray()
    for t in tokens:
        if len(t) % 2:
            raise ValueError("odd-length hex token %r" % t)
        data += bytes.fromhex(t)
    return bytes(data)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profile", required=True, choices=sorted(PROFILES))
    ap.add_argument("--file")
    ap.add_argument("--port")
    ap.add_argument("hex", nargs="?")
    a = ap.parse_args(argv)
    if a.port:
        import serial  # pyserial
        s = serial.Serial(a.port, PROFILES[a.profile]["baud"], timeout=0.2)
        buf = b""
        while True:
            buf += s.read(256)
            frames, buf, _ = decode(buf, a.profile)
            for fr in frames:
                print(fr["hex"], "->", "out of range" if fr["distance_m"] is None else "%.3f m" % fr["distance_m"], flush=True)
    data = open(a.file, "rb").read() if a.file else parse_hex(a.hex or sys.stdin.read())
    frames, rest, skipped = decode(data, a.profile)
    for fr in frames:
        print(fr["hex"], "->", "out of range" if fr["distance_m"] is None else "%.3f m" % fr["distance_m"])
    print("%d frames, %d bytes skipped, %d bytes left" % (len(frames), skipped, len(rest)), file=sys.stderr)
    return 0 if frames else 1


if __name__ == "__main__":
    sys.exit(main())
