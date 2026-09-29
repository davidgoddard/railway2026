'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const { crc32, CameraFrameStream } = require('./camera-frame-stream');

function encoded(fill) {
  const pixels = Buffer.alloc(320 * 240, fill);
  const header = Buffer.alloc(24);
  header.write('RAILFRM1'); header.writeUInt32LE(fill, 8);
  header.writeUInt32LE(pixels.length, 12); header.writeUInt32LE(crc32(pixels), 16);
  header.writeUInt16LE(320, 20); header.writeUInt16LE(240, 22);
  return Buffer.concat([header, pixels]);
}

test('paired comparison frames survive arbitrary USB fragmentation and interleaved logs', () => {
  const input = Buffer.concat([Buffer.from('crop_benchmark full\n'), encoded(12),
    Buffer.from('crop_benchmark crop\n'), encoded(34),
    Buffer.from('crop experiment complete: full-frame monitoring restored\ntrailer\n')]);
  const stream = new CameraFrameStream();
  for (let i = 0; i < input.length; i += 17) stream.push(input.subarray(i, i + 17));
  assert.equal(stream.frames.length, 2);
  assert.equal(stream.frames[0].pixels[0], 12); assert.equal(stream.frames[1].pixels.at(-1), 34);
  assert.match(stream.text, /full-frame monitoring restored/);
  assert.match(stream.text, /crop_benchmark crop/);
});

test('single-frame capture remains compatible', () => {
  const stream = new CameraFrameStream();stream.push(encoded(9));
  assert.equal(stream.frames.length, 1);assert.equal(stream.frames[0].number, 9);
});

test('bad CRC and inconsistent dimensions are rejected', () => {
  const corrupt = encoded(9);corrupt[30] ^= 1;
  assert.throws(() => new CameraFrameStream().push(corrupt), /CRC/);
  const wrongSize = encoded(9);wrongSize.writeUInt16LE(319, 20);
  assert.throws(() => new CameraFrameStream().push(wrongSize), /header/);
});
