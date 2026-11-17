"""lrf-decoder: decode laser rangefinder / single-point LiDAR UART captures.

  lrf-decoder "59 59 79 00 00 02 F0 08 25"            auto-detect the protocol and decode
  lrf-decoder --profile erdi-b-4-cm "5C 02 11 EC"      force a profile
  lrf-decoder --file capture.bin                        raw binary capture (use --hex-file for a text hex dump)
  lrf-decoder --port COM5 --scan                        try every common baud rate, report what talks
  lrf-decoder --port /dev/ttyUSB0 --profile jrt-aa --send "AA 00 00 20 00 01 00 06 27"
  lrf-decoder --list                                    list profiles with their sources
"""
import argparse
import json
import sys
import time

from .core import BAUD_RATES, COMMANDS, PROFILES, Decoder, detect, parse_hex

__version__ = "1.0.0"


def fmt(fr):
    mark = "OK " if fr["ok"] else "BAD"
    if fr["kind"] != "measurement":
        val = fr["kind"]
    elif fr.get("distance_m") is None:
        val = "out of range / invalid"
    else:
        val = "%.3f m" % fr["distance_m"]
    extra = []
    if fr.get("strength") is not None:
        extra.append("strength %s" % fr["strength"])
    if fr.get("temp_c") is not None:
        extra.append("%.1f C" % fr["temp_c"])
    if fr.get("note"):
        extra.append(fr["note"])
    if not fr["ok"]:
        extra.append("CHECKSUM MISMATCH")
    return "%s %-48s -> %s%s" % (mark, fr["hex"], val, ("  [" + "; ".join(extra) + "]") if extra else "")


def print_detect(cands, out=sys.stdout):
    if not cands:
        print("no known protocol found in this data", file=out)
        return
    print("protocol candidates (best first):", file=out)
    for c in cands[:5]:
        print("  %-18s %5.1f%% of bytes, %d frames, %d bad, baud %s" % (
            c["profile"], 100 * c["coverage"], c["frames"], c["bad"], "/".join(map(str, c["baud"]))), file=out)
        if len(c["members"]) > 1:
            print("      same bytes, pick by model: %s" % ", ".join(c["members"]), file=out)


def run_scan(port, bauds, probe, seconds):
    import serial  # pyserial

    results = []
    for baud in bauds:
        with serial.Serial(port, baud, timeout=0.1) as s:
            s.reset_input_buffer()
            if probe:
                for cmds in COMMANDS.values():
                    for name, h in cmds:
                        if name.lower().startswith(("single", "query", "continuous fast", "read cache")):
                            s.write(parse_hex(h))
            data = b""
            end = time.time() + seconds
            while time.time() < end:
                data += s.read(512)
            if probe:
                s.write(parse_hex("58"))  # JRT: leave continuous mode
        cands = detect(data)
        best = cands[0] if cands else None
        print("%7d bps: %5d bytes  %s" % (baud, len(data), "%s %.0f%%" % (best["profile"], 100 * best["coverage"]) if best else "-"))
        if best:
            results.append((best["coverage"], best["frames"], baud, best))
    if results:
        results.sort(key=lambda r: (-r[0], -r[1]))
        cov, n, baud, best = results[0]
        print("best: %d bps, profile %s (%d frames)" % (baud, best["profile"], n))
        return 0
    print("nothing decodable; try --probe for modules that only answer requests, or check wiring/voltage")
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog="lrf-decoder", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hex", nargs="?", help="hex bytes, e.g. '5C 02 11 EC' (stdin if omitted and no --file/--port)")
    ap.add_argument("--profile", default="auto", help="profile id or 'auto' (default)")
    ap.add_argument("--file", help="binary capture file")
    ap.add_argument("--hex-file", help="text file containing a hex dump")
    ap.add_argument("--port", help="serial port (needs pyserial)")
    ap.add_argument("--baud", type=int, help="baud rate (default: the profile's documented default)")
    ap.add_argument("--scan", action="store_true", help="with --port: try %s bps" % "/".join(map(str, BAUD_RATES)))
    ap.add_argument("--probe", action="store_true", help="with --scan: send the documented read-only measurement requests")
    ap.add_argument("--seconds", type=float, default=1.5, help="listen time per baud rate when scanning")
    ap.add_argument("--send", help="with --port: hex bytes to send once after opening")
    ap.add_argument("--json", action="store_true", help="print one JSON object per frame")
    ap.add_argument("--list", action="store_true", help="list profiles and exit")
    ap.add_argument("--version", action="version", version="%(prog)s " + __version__)
    a = ap.parse_args(argv)

    if a.list:
        for k, p in PROFILES.items():
            print("%-22s %-9s %s\n%22s baud %s; source %s" % (k, p["vendor"], p["label"], "", "/".join(map(str, p["baud"])), p["source"][0]))
        return 0
    if a.profile != "auto" and a.profile not in PROFILES:
        ap.error("unknown profile %r; use --list" % a.profile)

    if a.port:
        if a.scan:
            return run_scan(a.port, [a.baud] if a.baud else BAUD_RATES, a.probe, a.seconds)
        import serial  # pyserial

        if a.profile == "auto":
            ap.error("--port needs --profile (or use --scan to find it)")
        baud = a.baud or PROFILES[a.profile]["baud"][0]
        dec = Decoder(a.profile)
        with serial.Serial(a.port, baud, timeout=0.2) as s:
            if a.send:
                s.write(parse_hex(a.send))
            print("listening on %s at %d bps with %s (Ctrl+C to stop)" % (a.port, baud, a.profile), file=sys.stderr)
            try:
                while True:
                    for fr in dec.push(s.read(256)):
                        print(json.dumps(fr) if a.json else fmt(fr), flush=True)
            except KeyboardInterrupt:
                pass
        print("%d frames, %d bad, %d bytes skipped" % (dec.frames, dec.bad, dec.skipped), file=sys.stderr)
        return 0

    if a.file:
        with open(a.file, "rb") as fh:
            data = fh.read()
    elif a.hex_file:
        with open(a.hex_file, encoding="utf-8", errors="replace") as fh:
            data = parse_hex(fh.read())
    else:
        data = parse_hex(a.hex if a.hex is not None else sys.stdin.read())

    profile = a.profile
    if profile == "auto":
        cands = detect(data)
        print_detect(cands, out=sys.stderr)
        if not cands:
            return 1
        profile = cands[0]["profile"]
        print("decoding with %s\n" % profile, file=sys.stderr)
    dec = Decoder(profile)
    frames = dec.push(data)
    for fr in frames:
        print(json.dumps(fr) if a.json else fmt(fr))
    print("%d frames, %d bad, %d bytes skipped, %d bytes left" % (dec.frames, dec.bad, dec.skipped, len(dec.buf)), file=sys.stderr)
    return 0 if dec.frames else 1


if __name__ == "__main__":
    sys.exit(main())
