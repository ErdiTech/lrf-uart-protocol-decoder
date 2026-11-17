import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { Decoder, PROFILES, decode, detect, parseHex, crc16ccitt } from "../docs/decoder.js";

const V = JSON.parse(readFileSync(new URL("./vectors.json", import.meta.url), "utf8"));
const near = (a, b, msg) => assert.ok(Math.abs(a - b) < 1e-9, `${msg}: ${a} != ${b}`);

for (const v of V.vectors) {
  test(`${v.profile}: ${v.hex.slice(0, 40)} (${v.origin})`, () => {
    const { frames } = decode(parseHex(v.hex), v.profile);
    assert.equal(frames.length, v.expect.length, JSON.stringify(frames));
    v.expect.forEach((e, k) => {
      const f = frames[k];
      for (const [key, want] of Object.entries(e)) {
        if (typeof want === "number" && want !== null && typeof f[key] === "number") near(f[key], want, key);
        else assert.deepEqual(f[key], want, `${key} of frame ${k}`);
      }
    });
  });
}

for (const d of V.detect) {
  test(`detect ${d.family}`, () => {
    const r = detect(parseHex(d.hex));
    assert.ok(r.length > 0, "no candidates");
    assert.equal(r[0].family, d.family, JSON.stringify(r.map((x) => [x.family, x.coverage])));
  });
}

test("every profile cites a source and a baud rate", () => {
  for (const [k, p] of Object.entries(PROFILES)) {
    assert.ok(p.source && p.source.length, k);
    assert.ok(p.baud && p.baud.length, k);
  }
});

test("LightWare CRC equals CRC-16/XMODEM check value", () => {
  assert.equal(crc16ccitt([..."123456789"].map((c) => c.charCodeAt(0))), 0x31c3);
});

test("resync after garbage and split across pushes", () => {
  const d = new Decoder("erdi-b-4-cm");
  assert.equal(d.push(parseHex("00 5C 01 5C 02")).length, 0);
  const f = d.push(parseHex("11 EC 5C 02 11 EC"));
  assert.equal(f.length, 2);
  assert.ok(d.skipped >= 1);
});

test("byte-by-byte streaming equals one-shot decoding", () => {
  for (const v of V.vectors) {
    const d = new Decoder(v.profile);
    const got = [];
    for (const b of parseHex(v.hex)) got.push(...d.push([b]));
    assert.equal(got.length, decode(parseHex(v.hex), v.profile).frames.length, v.profile + " " + v.hex);
  }
});

test("parseHex accepts 0x and separators", () => {
  assert.deepEqual(parseHex("0x5C,0x02;11 ec"), [0x5c, 2, 0x11, 0xec]);
  assert.throws(() => parseHex("5C 2"));
});
