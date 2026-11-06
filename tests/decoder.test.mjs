import test from "node:test";
import assert from "node:assert/strict";
import { Decoder, parseHex } from "../docs/decoder.js";

const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`);

test("manual vector: H series is centimetres", () => {
  const d = new Decoder("erdi-b-4-cm");
  const f = d.push(parseHex("5C 02 11 EC"));
  assert.equal(f.length, 1);
  near(f[0].distance_m, 43.54);
});

test("same bytes on VB series are millimetres", () => {
  const d = new Decoder("erdi-b-4-mm");
  near(d.push(parseHex("5C 02 11 EC"))[0].distance_m, 4.354);
});

test("LRF1500H five-byte frame", () => {
  const body = [0x02, 0x11, 0x03];
  const chk = ~body.reduce((a, b) => a + b, 0) & 0xff;
  const d = new Decoder("erdi-b-5-cm");
  const f = d.push([0x5c, ...body, chk]);
  near(f[0].distance_m, 0x031102 * 0.01);
});

test("out of range marker (H series 65535)", () => {
  const d = new Decoder("erdi-b-4-cm");
  const f = d.push(parseHex("5C FF FF 01"));
  assert.equal(f[0].out_of_range, true);
  assert.equal(f[0].distance_m, null);
});

test("family A reply scales by ten", () => {
  const d = new Decoder("erdi-a");
  near(d.push(parseHex("55 AA 88 01 FF 00 2A B1"))[0].distance_m, 4.2);
});

test("family A no echo", () => {
  const d = new Decoder("erdi-a");
  const f = d.push(parseHex("55 AA 88 00 FF 00 00 86"));
  assert.equal(f.length, 1);
  assert.equal(f[0].distance_m, null);
});

test("Benewake-style 9-byte frame", () => {
  const d = new Decoder("benewake-tf-9");
  const body = [0x59, 0x59, 0x64, 0x00, 0x20, 0x03, 0x00, 0x00];
  const chk = body.reduce((a, b) => a + b, 0) & 0xff;
  const f = d.push([...body, chk]);
  near(f[0].distance_m, 1.0);
  assert.equal(f[0].strength, 0x320);
});

test("resync after garbage and split across pushes", () => {
  const d = new Decoder("erdi-b-4-cm");
  assert.equal(d.push(parseHex("00 5C 01 5C 02")).length, 0);
  const f = d.push(parseHex("11 EC 5C 02 11 EC"));
  assert.equal(f.length, 2);
  assert.ok(d.skipped >= 1);
});

test("parseHex accepts 0x and separators", () => {
  assert.deepEqual(parseHex("0x5C,0x02;11 ec"), [0x5c, 2, 0x11, 0xec]);
  assert.throws(() => parseHex("5C 2"));
});
