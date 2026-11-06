# Laser rangefinder UART protocol decoder (WebSerial + Python)

**Live decoder: <https://tommyapi.github.io/lrf-uart-protocol-decoder/>**

Decode **laser rangefinder module UART frames** in the browser or from the command line. Connect a module with **WebSerial** (Chrome / Edge) or paste captured hex. Built-in profiles: **ERDI 905 nm modules** (family B `5C` stream in centimetres and millimetres, LRF1500H 5-byte frame, family A `55 AA` replies) and **Benewake TFmini / TF-Luna style 9-byte frames**. Useful for debugging a **LiDAR distance sensor over serial**, checking checksums, finding the unit of a distance field, and writing your own driver.

## Features

- Streaming decoder with resync: garbage bytes are skipped, partial frames wait for the next chunk
- Per-profile checksum, byte order and unit; out-of-range markers reported as such
- Web UI (`docs/index.html`) with WebSerial connect and a paste-hex mode; no install, no server, no tracking
- Python CLI (`python/lrf_decoder.py`) for captures and live ports
- Node and pytest test suites using the golden frames printed in the ERDI manuals

## Profiles

| Profile | Frame | Checksum | Unit | Default baud |
|---|---|---|---|---|
| `erdi-b-4-cm` | `5C lo hi chk` (LRF50H, LRF100H, LRF200H, LRF300H, LRF600H) | `~(lo+hi)` | centimetres | 460800 |
| `erdi-b-4-mm` | `5C lo hi chk` (LRF22VB, LRF50VB) | `~(lo+hi)` | millimetres | 460800 |
| `erdi-b-5-cm` | `5C lo mid hi chk` (LRF1500H) | `~(lo+mid+hi)` | centimetres | 460800 |
| `erdi-a` | `55 AA fn STA FF DH DL SUM` (LR1000E2 ... LR2000E4, LRF1200A1, LRF3000A1, SPD1200N\*) | sum of all 7 preceding bytes | metres x 10 | 115200 |
| `benewake-tf-9` | `59 59 DL DH SL SH TL TH chk` | sum of first 8 bytes | centimetres | 115200 |

`5C 02 11 EC` decodes to **43.54 m** with `erdi-b-4-cm` and **4.354 m** with `erdi-b-4-mm`. Always select the profile by model name.

## Use it

Browser: open the live page, choose a profile, press **Connect serial port** (needs a secure context such as GitHub Pages or `localhost`), or paste hex and press **Decode**.

Python:

```bash
python python/lrf_decoder.py --profile erdi-b-4-cm "5C 02 11 EC"
python python/lrf_decoder.py --profile erdi-a --file capture.bin
pip install pyserial && python python/lrf_decoder.py --profile erdi-b-4-mm --port COM5
```

JavaScript:

```js
import { Decoder } from "./docs/decoder.js";
const d = new Decoder("erdi-b-4-mm");
d.push(chunk).forEach((f) => console.log(f.distance_m));
```

## Tests

```bash
node --test tests/          # Node 20+
pip install pytest && python -m pytest -q python
```

## Adding a profile

Add an entry to `PROFILES` and a branch in `tryFrame` (JS) / `decode` (Python), then add a golden-frame test. Only use values from the manufacturer's manual and say which manual in the PR.

## Related projects

- [ErdiTech/erdi-lrf-drivers](https://github.com/ErdiTech/erdi-lrf-drivers): Python, Arduino, ROS 2 drivers and protocol documents for ERDI modules
- [laser-rangefinder-module-database](https://github.com/Tommyapi/laser-rangefinder-module-database): open dataset of laser rangefinder specifications
- [awesome-laser-rangefinder](https://github.com/Tommyapi/awesome-laser-rangefinder): curated list of laser rangefinder resources

## Support / 打赏

If this project saved you time, you can buy the author a coffee in crypto. Thank you!
如果这个项目帮到了你，欢迎用加密货币打赏支持，谢谢！

| Network | Asset | Address |
|---|---|---|
| TRON (TRC20) | USDT | `TNGdTwE6opH6rmUyoG1JUR7FSyEb4k273R` |
| Ethereum / BNB Chain / Polygon | USDT | `0x59c43144a862b351fa1086ae842d1327c9d91613` |
| Ethereum / Arbitrum / Base | ETH | `0x59c43144a862b351fa1086ae842d1327c9d91613` |
| Bitcoin | BTC | `1RS6tWXHregLfEsiPzVUCUBuhgDg5uTSV` |
| Solana | USDT | `HZguEXfTtLChNdJTxAYCbcGscaqikz8JYrEpUhEyeVd9` |

Please send only the listed asset on the listed network — other tokens or networks may be lost.
请按表中对应的网络转对应的币种，转错币种或网络可能无法找回。

Built by tommyapi.com

## License

MIT — see [LICENSE](LICENSE).
