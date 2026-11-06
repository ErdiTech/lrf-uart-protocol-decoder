// lrf-uart-protocol-decoder: stream decoder for laser rangefinder UART frames.
// Works in browsers (WebSerial page) and Node (tests). No dependencies.

const sum8 = (bytes) => bytes.reduce((a, b) => (a + b) & 0xff, 0);
const hex = (bytes) => Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join(" ").toUpperCase();

// Each profile: find a frame at buf[i], return {len, frame} | null (need more) | false (no frame here).
export const PROFILES = {
  // ERDI family B: 5C <lo> <hi> <chk>, chk = ~(lo+hi). LRF1500H: 5C lo mid hi chk.
  "erdi-b-4-cm": { label: "ERDI H series (5C, 4-byte, centimetres)", baud: 460800, len: 4, unit: 0.01, oor: 65535, kind: "erdiB" },
  "erdi-b-4-mm": { label: "ERDI VB series (5C, 4-byte, millimetres)", baud: 460800, len: 4, unit: 0.001, oor: null, kind: "erdiB" },
  "erdi-b-5-cm": { label: "ERDI LRF1500H (5C, 5-byte, centimetres)", baud: 460800, len: 5, unit: 0.01, oor: 16777215, kind: "erdiB" },
  // ERDI family A: 55 AA fn STA FF DH DL SUM, reply sum covers the header; value x10 metres.
  "erdi-a": { label: "ERDI LR / LRF A / SPD (55 AA reply, metres x10)", baud: 115200, len: 8, unit: 0.1, kind: "erdiA" },
  // Benewake TF-series 9-byte: 59 59 Dist_L Dist_H Str_L Str_H Temp_L Temp_H chk(sum of first 8)
  "benewake-tf-9": { label: "Benewake TFmini / TF-Luna style (59 59, 9-byte, cm)", baud: 115200, len: 9, unit: 0.01, kind: "tf9" },
};

function inv8(bytes) {
  return ~sum8(bytes) & 0xff;
}

function tryFrame(profile, buf, i) {
  const p = PROFILES[profile];
  if (buf.length - i < 1) return null;
  if (p.kind === "erdiB") {
    if (buf[i] !== 0x5c) return false;
    if (buf.length - i < p.len) return null;
    const f = buf.slice(i, i + p.len);
    if (inv8(f.slice(1, -1)) !== f[f.length - 1]) return false;
    let raw = 0;
    for (let k = f.length - 2; k >= 1; k--) raw = raw * 256 + f[k];
    const oor = p.oor !== null && raw >= p.oor;
    return { len: p.len, frame: f, info: { raw, distance_m: oor ? null : raw * p.unit, out_of_range: oor } };
  }
  if (p.kind === "erdiA") {
    if (buf[i] !== 0x55) return false;
    if (buf.length - i < 2) return null;
    if (buf[i + 1] !== 0xaa) return false;
    if (buf.length - i < 8) return null;
    const f = buf.slice(i, i + 8);
    if (sum8(f.slice(0, 7)) !== f[7]) return false;
    const raw = (f[5] << 8) | f[6];
    const ok = f[3] === 1;
    return { len: 8, frame: f, info: { function: f[2], status: f[3], raw, distance_m: ok ? raw * p.unit : null, out_of_range: !ok } };
  }
  if (p.kind === "tf9") {
    if (buf[i] !== 0x59) return false;
    if (buf.length - i < 2) return null;
    if (buf[i + 1] !== 0x59) return false;
    if (buf.length - i < 9) return null;
    const f = buf.slice(i, i + 9);
    if (sum8(f.slice(0, 8)) !== f[8]) return false;
    const raw = f[2] | (f[3] << 8);
    return { len: 9, frame: f, info: { raw, distance_m: raw * p.unit, strength: f[4] | (f[5] << 8), temp_raw: f[6] | (f[7] << 8), out_of_range: false } };
  }
  return false;
}

export class Decoder {
  constructor(profile) {
    if (!PROFILES[profile]) throw new Error("unknown profile " + profile);
    this.profile = profile;
    this.buf = [];
    this.skipped = 0;
    this.frames = 0;
  }

  // Feed bytes (Uint8Array | number[]); returns the list of complete valid frames found.
  push(bytes) {
    for (const b of bytes) this.buf.push(b);
    const out = [];
    let i = 0;
    while (i < this.buf.length) {
      const r = tryFrame(this.profile, this.buf, i);
      if (r === null) break; // partial frame at tail: wait for more bytes
      if (r === false) {
        i += 1;
        this.skipped += 1;
        continue;
      }
      this.frames += 1;
      out.push({ hex: hex(r.frame), ...r.info });
      i += r.len;
    }
    this.buf = this.buf.slice(i);
    if (this.buf.length > 4096) this.buf = this.buf.slice(-64);
    return out;
  }
}

export function parseHex(text) {
  const clean = text.replace(/0x/gi, " ").replace(/[^0-9a-fA-F]+/g, " ").trim();
  if (!clean) return [];
  const parts = clean.split(" ");
  const bytes = [];
  for (const p of parts) {
    if (p.length % 2) throw new Error("odd-length hex token: " + p);
    for (let k = 0; k < p.length; k += 2) bytes.push(parseInt(p.slice(k, k + 2), 16));
  }
  return bytes;
}

export { hex };
