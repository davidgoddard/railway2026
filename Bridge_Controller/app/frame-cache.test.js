'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const os = require('node:os');
const path = require('node:path');
const { saveFrame, loadFrame } = require('./frame-cache');

test('keeps the newest lossless frame separately for each camera', async () => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'railway-frames-'));
  try {
    const macA = 'AA:BB:CC:DD:EE:01', macB = 'AA:BB:CC:DD:EE:02';
    const pixels = Buffer.alloc(320 * 240);
    for (let i = 0; i < pixels.length; i++) pixels[i] = (i * 17 + Math.floor(i / 320)) & 255;
    await saveFrame(directory, { mac: macA, frame: 1, width: 320, height: 240, pixels });
    await saveFrame(directory, { mac: macB, frame: 8, width: 320, height: 240, pixels: Buffer.alloc(pixels.length, 9) });
    await saveFrame(directory, { mac: macA, frame: 2, width: 320, height: 240, pixels: Buffer.alloc(pixels.length, 42) });
    const a = await loadFrame(directory, macA), b = await loadFrame(directory, macB);
    assert.equal(a.frame, 2); assert.equal(a.cached, true); assert.equal(a.pixels[123], 42);
    assert.equal(b.frame, 8); assert.equal(b.pixels[123], 9);
  } finally { await fs.rm(directory, { recursive: true, force: true }); }
});
