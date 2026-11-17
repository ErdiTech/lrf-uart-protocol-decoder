// lrf-uart-protocol-decoder: stream decoder + protocol auto-detection for
// laser rangefinder / single-point LiDAR UART frames.
// Works in browsers (WebSerial page) and Node (tests). No dependencies.
// Every profile cites the public manual it was built from; see docs/PROTOCOLS.md.

export const VERSION = "1.0.0";

const sum8 = (b, s = 0, e = b.length) => {
  let a = 0;
  for (let k = s; k < e; k++) a = (a + b[k]) & 0xff;
  return a;
};
const inv8 = (b, s, e) => ~sum8(b, s, e) & 0xff;
const u16le = (f, o) => f[o] | (f[o + 1] << 8);
const u16be = (f, o) => (f[o] << 8) | f[o + 1];
const hex = (bytes) => Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join(" ").toUpperCase();
const ascii = (f, s, e) => String.fromCharCode(...f.slice(s, e));
const h2 = (n) => "0x" + n.toString(16).padStart(2, "0").toUpperCase();

// LightWare binary protocol CRC: CRC-16-CCITT 0x1021, init 0 (SF30/D Product Guide v3.3 sample code).
export function crc16ccitt(data, s = 0, e = data.length) {
  let crc = 0;
  for (let i = s; i < e; i++) {
    let code = (crc >> 8) & 0xff;
    code ^= data[i];
    code ^= code >> 4;
    crc = (crc << 8) & 0xffff;
    crc ^= code;
    code = (code << 5) & 0xffff;
    crc ^= code;
    code = (code << 7) & 0xffff;
    crc ^= code;
  }
  return crc & 0xffff;
}

// Parser contract: parse(buf, i, p) returns
//   null                    -> could be a frame here, need more bytes
//   false                   -> no frame starts at i
//   {len, ok, kind, info}   -> a frame of `len` bytes; ok=false means checksum mismatch

// ---------- ERDI family B: 5C data frames, 5A command frames ----------
const ERDI_B_CMDS = { 0x0d: "read serial no.", 0x16: "read software version", 0x06: "set baud rate", 0x0b: "set frequency", 0x1b: "read frequency", 0x1f: "switch to I2C", 0x0a: "start/stop ranging" };
function parseErdiB(buf, i, p) {
  const n = buf.length - i;
  if (buf[i] === 0x5c) {
    if (n < p.len) return null;
    const ok = inv8(buf, i + 1, i + p.len - 1) === buf[i + p.len - 1];
    let raw = 0;
    for (let k = i + p.len - 2; k >= i + 1; k--) raw = raw * 256 + buf[k];
    const oor = p.oor !== null && raw >= p.oor;
    return { len: p.len, ok, kind: "measurement", info: { raw, distance_m: oor ? null : raw * p.unit, out_of_range: oor, note: oor ? "out-of-range marker " + p.oor : "" } };
  }
  if (buf[i] === 0x5a) {
    if (n < 3) return null;
    const cmd = buf[i + 1];
    const tried = [...new Set([buf[i + 2], 4, 2])];
    for (const dl of tried) {
      const len = 4 + dl;
      if (dl > 8 || n < len) continue;
      if (inv8(buf, i + 1, i + len - 1) === buf[i + len - 1]) {
        const reply = (cmd & 0x80) !== 0;
        const name = ERDI_B_CMDS[cmd & 0x7f] || "command " + h2(cmd & 0x7f);
        return { len, ok: true, kind: reply ? "reply" : "request", info: { note: (reply ? "reply: " : "request: ") + name } };
      }
    }
    return n < 4 + Math.max(...tried.filter((d) => d <= 8)) ? null : false;
  }
  return false;
}

// ---------- ERDI family A: 55 AA fn D1 D2 D3 D4 SUM ----------
const ERDI_A_FN = { 0x88: "single-shot ranging", 0x89: "continuous ranging", 0x8e: "stop ranging", 0x8a: "angle measurement", 0x80: "self-test", 0x86: "LD constant-on", 0x70: "external circuit enable", 0x71: "external circuit disable" };
function parseErdiA(buf, i, p) {
  if (buf[i] !== 0x55) return false;
  if (buf.length - i < 2) return null;
  if (buf[i + 1] !== 0xaa) return false;
  if (buf.length - i < 8) return null;
  const f = buf.slice(i, i + 8);
  const fn = f[2];
  const name = ERDI_A_FN[fn] || "function " + h2(fn);
  if (sum8(f, 0, 7) === f[7]) {
    const info = { function: fn, status: f[3], note: "reply: " + name };
    if (fn === 0x88 || fn === 0x89) {
      const raw = u16be(f, 5);
      const good = f[3] === 1;
      Object.assign(info, { raw, distance_m: good ? raw * p.unit : null, out_of_range: !good, note: good ? "" : "STA=" + f[3] + " (measurement failed / no echo)" });
      return { len: 8, ok: true, kind: "measurement", info };
    }
    if (fn === 0x8a) Object.assign(info, { raw: u16be(f, 5), note: "angle " + (u16be(f, 5) * 0.1).toFixed(1) + " deg" });
    if (fn === 0x80) info.note = "self-test, error code candidates D3=" + h2(f[5]) + " D4=" + h2(f[6]);
    return { len: 8, ok: true, kind: "reply", info };
  }
  if (sum8(f, 2, 7) === f[7]) return { len: 8, ok: true, kind: "request", info: { function: fn, note: "request: " + name } };
  return { len: 8, ok: false, kind: "measurement", info: { note: "reply checksum must be SUM(55..D4); request checksum SUM(fn..D4)" } };
}

// ---------- Benewake 9-byte frames (59 59 ...) and 5A command frames ----------
function benewakeInfo(p, dist, strength, temp) {
  const info = { raw: dist, distance_m: dist * p.unit, strength, out_of_range: false, note: "" };
  if (p.temp) info.temp_c = temp / 8 - 256;
  if (!p.strength) delete info.strength;
  const m = p.model;
  if (m === "tfmini-s") {
    const codes = { 65535: "-1: strength < 100", 65534: "-2: signal strength saturation", 65532: "-4: ambient light saturation" };
    if (codes[dist]) Object.assign(info, { distance_m: null, out_of_range: true, note: codes[dist] });
  } else if (m === "tfmini-plus" || m === "tf-luna") {
    if (strength < 100 || strength === 65535) Object.assign(info, { distance_m: null, out_of_range: true, note: strength === 65535 ? "strength 65535: overexposed, distance unreliable" : "strength < 100: distance unreliable (dummy distance " + dist + ")" });
  } else if (m === "tf02-pro") {
    if (strength < 60) Object.assign(info, { distance_m: null, out_of_range: true, note: "strength < 60: module outputs 4500 cm (out of range)" });
  }
  return info;
}
function parseBenewake(buf, i, p) {
  const n = buf.length - i;
  if (buf[i] === 0x59) {
    if (n < 2) return null;
    if (buf[i + 1] !== 0x59) return false;
    if (n < 9) return null;
    const ok = sum8(buf, i, i + 8) === buf[i + 8];
    return { len: 9, ok, kind: "measurement", info: benewakeInfo(p, u16le(buf, i + 2), u16le(buf, i + 4), u16le(buf, i + 6)) };
  }
  if (buf[i] === 0x5a) {
    // Command / response: 5A Len ID Data.. Checksum, Len = whole frame, checksum = low byte of sum of preceding bytes
    if (n < 2) return null;
    const len = buf[i + 1];
    if (len < 4 || len > 16) return false;
    if (n < len) return null;
    if (sum8(buf, i, i + len - 1) !== buf[i + len - 1]) return false;
    return { len, ok: true, kind: "request", info: { note: "command/response ID " + h2(buf[i + 2]) } };
  }
  return false;
}

// ---------- ASCII decimal metres, one value per CR LF line ----------
function parseAsciiMetres(buf, i) {
  const c = buf[i];
  const startOk = (c >= 0x30 && c <= 0x39) || c === 0x2d || c === 0x20 || c === 0x2b;
  if (!startOk) return false;
  if (i > 0 && buf[i - 1] !== 0x0a) return false;
  let j = i;
  while (j < buf.length && j - i < 48 && buf[j] !== 0x0a) {
    const b = buf[j];
    if (b !== 0x0d && (b < 0x20 || b > 0x7e)) return false;
    j++;
  }
  if (j >= buf.length) return j - i < 48 ? null : false;
  if (buf[j] !== 0x0a) return false;
  const line = ascii(buf, i, j).replace(/\r$/, "");
  const m = /^\s*([-+]?\d+(?:\.\d+)?)(?:\s*m\b)?(.*)$/.exec(line);
  if (!m) return false;
  const v = parseFloat(m[1]);
  return { len: j - i + 1, ok: true, kind: "measurement", info: { raw: m[1], distance_m: v, out_of_range: v < 0, note: m[2].trim() } };
}

// ---------- LightWare binary protocol (SF30/D Product Guide v3.3) ----------
function parseLightWare(buf, i) {
  const n = buf.length - i;
  if (buf[i] !== 0xaa) return false;
  if (n < 3) return null;
  const flags = u16le(buf, i + 1);
  if (flags & 0x3e) return false; // bits 1..5 reserved
  const plen = flags >> 6;
  if (plen < 1 || plen > 1023) return false;
  const len = 3 + plen + 2;
  if (n < len) return null;
  const ok = crc16ccitt(buf, i, i + len - 2) === u16le(buf, i + len - 2);
  const id = buf[i + 3];
  const write = (flags & 1) === 1;
  const info = { command: id, note: "command " + id + (write ? " (write)" : "") + ", " + plen + "-byte payload" };
  let kind = "reply";
  if (id === 44 && plen >= 3) {
    const vals = [];
    for (let k = i + 4; k + 1 < i + 3 + plen; k += 2) {
      let v = u16le(buf, k);
      if (v & 0x8000) v -= 0x10000;
      vals.push(v);
    }
    Object.assign(info, { raw: vals[0], distance_m: vals[0] / 100, out_of_range: false, values: vals, note: "(44) distance data, int16 fields " + vals.join(", ") + " - order set by command (29) distance output" });
    kind = "measurement";
  }
  return { len, ok, kind, info };
}

// ---------- Chengdu JRT register protocol (AA addr reg reg cnt cnt payload chk) ----------
function parseJrt(buf, i) {
  const n = buf.length - i;
  if (buf[i] !== 0xaa) return false;
  if (n < 6) return null;
  const reg = u16be(buf, i + 2);
  const cnt = u16be(buf, i + 4);
  if (cnt < 1 || cnt > 16) return false;
  const len = 7 + 2 * cnt;
  if (n < len) return null;
  const ok = sum8(buf, i + 1, i + len - 1) === buf[i + len - 1];
  if (!ok && !(reg === 0x22 && cnt === 3)) return false;
  if (reg === 0x22 && cnt === 3) {
    const mm = ((buf[i + 6] << 24) >>> 0) + (buf[i + 7] << 16) + (buf[i + 8] << 8) + buf[i + 9];
    return { len, ok, kind: "measurement", info: { raw: mm, distance_m: mm / 1000, strength: u16be(buf, i + 10), out_of_range: false, note: "strength = JRT signal quality" } };
  }
  if (reg === 0x20) return { len, ok, kind: "request", info: { note: "request: start measurement, mode " + h2(buf[i + 7]) + (buf[i + 7] === 6 ? " (continuous fast)" : "") } };
  return { len, ok, kind: "reply", info: { note: "register " + h2(reg) + ", " + cnt + " word(s)" } };
}

// ---------- Nooploop TOFSense NLink_TOFSense_Frame0 ----------
function parseTofSense(buf, i) {
  const n = buf.length - i;
  if (buf[i] !== 0x57) return false;
  if (n < 2) return null;
  if (buf[i + 1] === 0x10) {
    if (n < 8) return null;
    if (sum8(buf, i, i + 7) !== buf[i + 7]) return false;
    return { len: 8, ok: true, kind: "request", info: { note: "query frame for module id " + buf[i + 4] } };
  }
  if (buf[i + 1] !== 0x00) return false;
  if (n < 16) return null;
  const ok = sum8(buf, i, i + 15) === buf[i + 15];
  let dis = buf[i + 8] | (buf[i + 9] << 8) | (buf[i + 10] << 16);
  if (dis & 0x800000) dis -= 0x1000000;
  const status = buf[i + 11];
  const strength = u16le(buf, i + 12);
  return { len: 16, ok, kind: "measurement", info: { id: buf[i + 3], raw: dis, distance_m: dis / 1000, strength, status, out_of_range: false, note: "id " + buf[i + 3] + ", dis_status " + status + ", system_time " + ((buf[i + 4] | (buf[i + 5] << 8) | (buf[i + 6] << 16)) + buf[i + 7] * 16777216) + " ms" } };
}

// ---------- DFRobot SEN0366 / generic 80 m "ADDR 06 83 ASCII CS" module ----------
const neg8 = (b, s, e) => (0x100 - sum8(b, s, e)) & 0xff;
function parseDfr80m(buf, i) {
  const n = buf.length - i;
  if (n < 3) return null;
  if (buf[i + 1] !== 0x06) return false;
  const fn = buf[i + 2];
  if (fn === 0x02 || fn === 0x03 || fn === 0x07) {
    if (n < 4) return null;
    if (neg8(buf, i, i + 3) !== buf[i + 3]) return false;
    return { len: 4, ok: true, kind: "request", info: { note: "request: " + { 2: "single measurement", 3: "continuous measurement", 7: "read cache" }[fn] } };
  }
  if (fn !== 0x82 && fn !== 0x83) return false;
  for (const dl of [7, 8]) {
    const len = 4 + dl;
    if (n < len) return null;
    const text = ascii(buf, i + 3, i + 3 + dl);
    if (!/^[0-9.ER-]+$/.test(text)) continue;
    if (neg8(buf, i, i + len - 1) !== buf[i + len - 1]) continue;
    if (text.startsWith("ER")) return { len, ok: true, kind: "measurement", info: { raw: text, distance_m: null, out_of_range: true, note: "error " + text } };
    return { len, ok: true, kind: "measurement", info: { raw: text, distance_m: parseFloat(text), out_of_range: false, note: dl === 8 ? "0.1 mm resolution" : "" } };
  }
  return n >= 12 ? { len: 11, ok: false, kind: "measurement", info: { note: "CS = (~sum + 1) of all preceding bytes" } } : null;
}

// ---------- profile registry ----------
const BW = "Benewake";
const BW_SRC = ["https://cdn.sparkfun.com/assets/8/a/f/a/c/16977-TFMini-S_-_Micro_LiDAR_Module-Product_Manual.pdf"];
const bw = (model, label, extra = {}) => ({ vendor: BW, family: "benewake-9", label, baud: [115200], unit: 0.01, strength: true, temp: true, model, strong: true, parse: parseBenewake, ...extra });
const ERDI_B_SRC = ["https://github.com/ErdiTech/erdi-lrf-drivers/blob/main/docs/protocol-B-5C.md"];
const erdiB = (label, len, unit, oor) => ({ vendor: "ERDI", family: "erdi-5c-" + len, label, baud: [460800, 115200], len, unit, oor, parse: parseErdiB, source: ERDI_B_SRC });

export const PROFILES = {
  "erdi-b-4-cm": erdiB("ERDI H series LRF50H/100H/200H/300H/600H (5C, cm)", 4, 0.01, 65535),
  "erdi-b-4-mm": erdiB("ERDI VB series, generic (5C, mm)", 4, 0.001, null),
  "erdi-lrf22vb": erdiB("ERDI LRF22VB (5C, mm, 20000 = out of range)", 4, 0.001, 20000),
  "erdi-lrf50vb": erdiB("ERDI LRF50VB (5C, mm, 50000 = out of range)", 4, 0.001, 50000),
  "erdi-b-5-cm": erdiB("ERDI LRF1500H (5C, 5-byte, cm)", 5, 0.01, 16777215),
  "erdi-a": { vendor: "ERDI", family: "erdi-55aa", label: "ERDI LR / LRF-A1 / SPD (55 AA, metres x10)", baud: [115200], unit: 0.1, strong: true, parse: parseErdiA, source: ["https://github.com/ErdiTech/erdi-lrf-drivers/blob/main/docs/protocol-A-55AA.md"] },
  "benewake-tf-9": bw("generic", "Benewake 9-byte, model unknown (59 59, cm)", { source: BW_SRC }),
  "benewake-tfmini-s": bw("tfmini-s", "Benewake TFmini-S (59 59, cm, -1/-2/-4 codes)", { source: BW_SRC }),
  "benewake-tfmini-plus": bw("tfmini-plus", "Benewake TFmini Plus (59 59, cm)", { source: ["https://m.media-amazon.com/images/I/A1UADgRvNzL.pdf"] }),
  "benewake-tf-luna": bw("tf-luna", "Benewake TF-Luna (59 59, cm)", { source: ["https://cdn.compacttool.ru/downloads/TF-Luna%20Datasheet.pdf"] }),
  "benewake-tf02-pro": bw("tf02-pro", "Benewake TF02-Pro (59 59, cm)", { source: ["https://evelta.com/content/datasheets/BP-UM-26%20A02%20TF02-Pro%20User%20Manual.pdf"] }),
  "benewake-tf03": bw("tf03", "Benewake TF03 (59 59, cm, bytes 4-7 reserved)", { strength: false, temp: false, source: ["https://www.worldronemarket.com/benewake-tf03-rangefinder-180-m-user-manual/"] }),
  "benewake-tf-9-mm": bw("generic-mm", "Benewake 9-byte mm output mode (59 59, mm)", { unit: 0.001, source: BW_SRC }),
  "ascii-metres": { vendor: "ASCII", family: "ascii", label: "ASCII metres per line: LightWare SF10/SF11 serial, Benewake string/Pixhawk mode", baud: [115200, 9600, 19200, 38400, 57600], parse: parseAsciiMetres, source: ["https://acroname.com/sites/default/files/assets/sf11_-_laser_altimeter_manual_rev_0.pdf", ...BW_SRC] },
  "lightware-binary": { vendor: "LightWare", family: "lightware-binary", label: "LightWare binary protocol (AA, CRC-16; SF30/D distance ID 44)", baud: [115200, 921600], parse: parseLightWare, source: ["https://lightwarelidar.com/wp-content/uploads/2026/03/SF30D-Product-Guide-v3.3-1.pdf"] },
  "jrt-aa": { vendor: "JRT", family: "jrt-aa", label: "Chengdu JRT laser distance sensor (AA 00 00 22, mm)", baud: [19200, 115200, 9600, 38400], parse: parseJrt, source: ["https://www.jrt-measure.com/serial-laser-distance-sensor/60034347.html"] },
  "nooploop-tofsense": { vendor: "Nooploop", family: "nooploop-tofsense", label: "Nooploop TOFSense / TOFSense-F (57 00, 16-byte)", baud: [921600, 115200], strong: true, parse: parseTofSense, source: ["https://support.nooploop.com/tofsense/protocol/", "https://files.seeedstudio.com/products/101990702/TOFSense_User_Manual_V2.0_EN.pdf"] },
  "dfrobot-sen0366": { vendor: "DFRobot", family: "dfrobot-80m", label: "DFRobot SEN0366 / 80 m laser module (80 06 83 ASCII)", baud: [9600], strong: true, parse: parseDfr80m, source: ["https://dfimg.dfrobot.com/nobody/wiki/068db268ba37a41067c1b17607932139.pdf"] },
};

// Read-only measurement requests, as printed in each manual. Used by the web page's
// "send" buttons and, when the user opts in, by baud scanning.
export const COMMANDS = {
  "erdi-a": [["Single-shot ranging", "55 AA 88 FF FF FF FF 84"], ["Continuous ranging", "55 AA 89 FF FF FF FF 85"], ["Stop ranging", "55 AA 8E FF FF FF FF 8A"]],
  "erdi-b-4-cm": [["Read software version", "5A 16 02 16 16 BB"], ["Stop ranging (LRF50H)", "5A 0A 02 00 00 F3"], ["Start ranging (LRF50H)", "5A 0A 02 02 00 F1"]],
  "jrt-aa": [["Continuous fast measurement", "AA 00 00 20 00 01 00 06 27"], ["Stop continuous", "58"]],
  "nooploop-tofsense": [["Query module id 0", "57 10 FF FF 00 FF FF 63"]],
  "dfrobot-sen0366": [["Single measurement", "80 06 02 78"], ["Continuous measurement", "80 06 03 77"], ["Read cache", "80 06 07 73"]],
  "benewake-tf-9": [["Output format: standard 9-byte cm", "5A 05 05 01 65"]],
};
for (const k of ["erdi-b-4-mm", "erdi-lrf22vb", "erdi-lrf50vb", "erdi-b-5-cm"]) COMMANDS[k] = COMMANDS["erdi-b-4-cm"];
for (const k of Object.keys(PROFILES)) if (PROFILES[k].family === "benewake-9") COMMANDS[k] = COMMANDS["benewake-tf-9"];

export const BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

export class Decoder {
  constructor(profile) {
    if (!PROFILES[profile]) throw new Error("unknown profile " + profile);
    this.profile = profile;
    this.p = PROFILES[profile];
    this.buf = [];
    this.skipped = 0;
    this.frames = 0;
    this.bad = 0;
  }

  // Feed bytes (Uint8Array | number[]). Returns decoded records (valid and checksum-failed).
  push(bytes) {
    for (const b of bytes) this.buf.push(b);
    const out = [];
    const buf = this.buf;
    let i = 0;
    while (i < buf.length) {
      const r = this.p.parse(buf, i, this.p);
      if (r === null) break;
      // A checksum failure on a weak (one-byte) header only counts as a damaged frame when the
      // next frame starts right after it; at the tail we wait for that next byte.
      if (r !== false && !r.ok && !this.p.strong && i + r.len >= buf.length) break;
      if (r === false || (!r.ok && !this.p.strong && buf[i + r.len] !== buf[i])) {
        i += 1;
        this.skipped += 1;
        continue;
      }
      const rec = { profile: this.profile, kind: r.kind, ok: r.ok, hex: hex(buf.slice(i, i + r.len)), ...r.info };
      if (!rec.note) rec.note = "";
      out.push(rec);
      if (r.ok) {
        this.frames += 1;
        i += r.len;
      } else {
        this.bad += 1;
        i += 1;
      }
    }
    this.buf = buf.slice(i);
    if (this.buf.length > 8192) this.buf = this.buf.slice(-2048);
    return out;
  }
}

export function decode(bytes, profile) {
  const d = new Decoder(profile);
  const frames = d.push(bytes);
  return { frames, skipped: d.skipped, bad: d.bad, rest: d.buf };
}

// Rank protocol families by how much of the capture they explain.
export function detect(bytes) {
  const seen = new Set();
  const out = [];
  const total = bytes.length || 1;
  for (const [id, p] of Object.entries(PROFILES)) {
    if (seen.has(p.family)) continue;
    seen.add(p.family);
    const { frames, bad } = decode(bytes, id);
    const good = frames.filter((f) => f.ok);
    if (!good.length) continue;
    const covered = good.reduce((a, f) => a + f.hex.length / 3 + 1 / 3, 0);
    const meas = good.filter((f) => f.kind === "measurement").length;
    const members = Object.keys(PROFILES).filter((k) => PROFILES[k].family === p.family);
    out.push({ profile: id, family: p.family, vendor: p.vendor, label: p.label, members, frames: good.length, measurements: meas, bad, coverage: Math.min(1, covered / total), baud: p.baud });
  }
  out.sort((a, b) => b.coverage - a.coverage || b.measurements - a.measurements || a.bad - b.bad);
  return out;
}

export function parseHex(text) {
  const clean = text.replace(/0x/gi, " ").replace(/[^0-9a-fA-F]+/g, " ").trim();
  if (!clean) return [];
  const bytes = [];
  for (const p of clean.split(" ")) {
    if (p.length % 2) throw new Error("odd-length hex token: " + p);
    for (let k = 0; k < p.length; k += 2) bytes.push(parseInt(p.slice(k, k + 2), 16));
  }
  return bytes;
}

export { hex, sum8 };
