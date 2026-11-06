import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import pytest

from lrf_decoder import decode, parse_hex


def test_h_series_centimetres():
    fr, rest, _ = decode(parse_hex("5C 02 11 EC"), "erdi-b-4-cm")
    assert fr[0]["distance_m"] == pytest.approx(43.54)


def test_vb_series_millimetres():
    fr, _, _ = decode(parse_hex("5C 02 11 EC"), "erdi-b-4-mm")
    assert fr[0]["distance_m"] == pytest.approx(4.354)


def test_family_a():
    fr, _, _ = decode(parse_hex("55 AA 88 01 FF 00 2A B1"), "erdi-a")
    assert fr[0]["distance_m"] == pytest.approx(4.2)


def test_resync_and_partial_tail():
    fr, rest, skipped = decode(parse_hex("00 5C 02 11 EC 5C 02"), "erdi-b-4-cm")
    assert len(fr) == 1 and skipped == 1 and rest == b"\x5c\x02"


def test_benewake_style():
    body = [0x59, 0x59, 0x64, 0x00, 0x20, 0x03, 0x00, 0x00]
    fr, _, _ = decode(bytes(body + [sum(body) & 0xFF]), "benewake-tf-9")
    assert fr[0]["distance_m"] == pytest.approx(1.0)


def test_bad_hex():
    with pytest.raises(ValueError):
        parse_hex("5C 2")
