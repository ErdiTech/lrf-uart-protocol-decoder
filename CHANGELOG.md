# Changelog

## 1.0.0 - 2026-11-17

### Added
- Protocol auto-detection (`detect()` in JS and Python): ranks every protocol family by the share
  of the capture it explains and lists the models that share the same frame.
- Baud-rate scan in the web page and CLI (`--scan`), with optional documented read-only
  measurement requests (`--probe`) for modules that only answer when asked.
- Checksum failures are reported and highlighted instead of being dropped.
- New profiles, each with its source manual cited in `docs/PROTOCOLS.md`:
  - Benewake per-model rules: TFmini-S, TFmini Plus, TF-Luna, TF02-Pro, TF03, mm output mode
  - LightWare binary protocol (SF30/D, CRC-16-CCITT) and ASCII metres (SF10/SF11, Benewake string mode)
  - Chengdu JRT register protocol (`AA 00 00 22`), from the vendor's published capture
  - Nooploop TOFSense / TOFSense-F frame and query frame
  - DFRobot SEN0366 / 80 m laser module ASCII protocol
  - ERDI LRF22VB / LRF50VB out-of-range markers, `5A` command frames, family A request frames
- Web page: vendor-grouped profile list, baud selector, examples, send box with each manual's
  commands, CSV export, `?hex=` links.
- Python package `lrf-uart-decoder` on PyPI with the `lrf-decoder` command
  (`python -m lrf_decoder`), `--json`, `--hex-file`, `--list`.
- Shared golden vectors (`tests/vectors.json`) used by both the Node and pytest suites.

### Changed
- `python/lrf_decoder.py` became the package `python/lrf_decoder/`. `decode(data, profile)` and
  `parse_hex()` keep their signatures; frame dicts gain `ok`, `kind` and `note`.

## 0.1.0 - 2026-11-06

- First release: ERDI `5C` / `55 AA` frames and Benewake-style 9-byte frames, WebSerial page and Python CLI.
