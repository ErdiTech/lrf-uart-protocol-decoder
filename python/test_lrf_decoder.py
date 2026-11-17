import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pytest

from lrf_decoder import PROFILES, Decoder, crc16ccitt, decode, detect, parse_hex
from lrf_decoder.cli import main

with open(os.path.join(HERE, "..", "tests", "vectors.json"), encoding="utf-8") as fh:
    V = json.load(fh)


@pytest.mark.parametrize("v", V["vectors"], ids=lambda v: "%s:%s" % (v["profile"], v["hex"][:24]))
def test_vector(v):
    frames, _, _ = decode(parse_hex(v["hex"]), v["profile"])
    assert len(frames) == len(v["expect"]), frames
    for f, e in zip(frames, v["expect"]):
        for key, want in e.items():
            if isinstance(want, float) or (isinstance(want, int) and not isinstance(want, bool) and isinstance(f[key], float)):
                assert f[key] == pytest.approx(want), key
            else:
                assert f[key] == want, key


@pytest.mark.parametrize("d", V["detect"], ids=lambda d: d["family"])
def test_detect(d):
    cands = detect(parse_hex(d["hex"]))
    assert cands and cands[0]["family"] == d["family"], [(c["family"], c["coverage"]) for c in cands]


def test_sources_and_baud():
    for k, p in PROFILES.items():
        assert p["source"] and p["baud"], k


def test_crc_xmodem_check_value():
    assert crc16ccitt(b"123456789") == 0x31C3


def test_resync_and_partial_tail():
    fr, rest, skipped = decode(parse_hex("00 5C 02 11 EC 5C 02"), "erdi-b-4-cm")
    assert len(fr) == 1 and skipped == 1 and rest == b"\x5c\x02"


def test_byte_by_byte_equals_one_shot():
    for v in V["vectors"]:
        d = Decoder(v["profile"])
        got = []
        for b in parse_hex(v["hex"]):
            got += d.push(bytes([b]))
        assert len(got) == len(decode(parse_hex(v["hex"]), v["profile"])[0]), v


def test_bad_hex():
    with pytest.raises(ValueError):
        parse_hex("5C 2")


def test_cli_auto(capsys):
    assert main(["59 59 79 00 00 02 F0 08 25 59 59 7A 00 00 02 F0 08 26"]) == 0
    out = capsys.readouterr()
    assert "1.210 m" in out.out and "benewake" in out.err


def test_cli_list(capsys):
    assert main(["--list"]) == 0
    assert "jrt-aa" in capsys.readouterr().out
