'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const zlib = require('node:zlib');
const { crc32, Snapshot, parseRow, validateConfig, commandsForConfig } = require('./protocol');
const { sameCameraConfiguration, alignConfigurationToFrame, frameMatchesConfiguration } = require('./config-match');
const mac = 'AA:BB:CC:DD:EE:FF';
const config = () => ({ revision: 4, settings: { resolution: 0, brightness: 0, contrast: 0, saturation: 0, vflip: 0, hmirror: 0 }, cells: [
  { id: 1, group: 20, x: 40, y: 50, radius: 5, shape: 0, floor: 80, tolerance: 10, threshold: 200, enter: 3, clear: 5 },
  { id: 2, group: 20, x: 45, y: 50, radius: 5, shape: 0, floor: 80, tolerance: 10, threshold: 200, enter: 3, clear: 5 }
], topics: { 20: 'platform-1' } });
test('parses bridge configuration and topic rows', () => {
  assert.deepEqual(parseRow('TOPIC 20 platform-1'), { type: 'topic', id: 20, name: 'platform-1' });
  assert.equal(parseRow('CELL 0 1 20 40 50 5 0 80 10 200 3 5').cell.threshold, 200);
});
test('builds a complete ordered upload including the MQTT alias', () => {
  const commands = commandsForConfig(mac, config());
  assert.equal(commands[0], `BEGIN ${mac} 5 2 0 0 0 0 0 0`);
  assert.equal(commands[3], `COMMIT ${mac}`);
  assert.equal(commands[4], `TOPIC ${mac} 20 706C6174666F726D2D31`);
});
test('rejects IDs used by another camera and invalid sensor bounds', () => {
  assert.throws(() => validateConfig(config(), [20]), /already used/);
  const c = config(); c.cells[0].x = 320;
  assert.throws(() => validateConfig(c), /outside/);
});
test('a new camera draft can align to an 800 × 600 frame', () => {
  const draft = config();
  draft.revision = 0;
  draft.cells[0].x = 231;
  draft.cells[0].y = 511;
  assert.throws(() => validateConfig(draft), /outside the selected 320 × 240 image/);
  assert.equal(alignConfigurationToFrame(draft, { width: 800, height: 600 }), true);
  assert.equal(draft.settings.resolution, 2);
  assert.doesNotThrow(() => validateConfig(draft));
});
test('an old cached frame does not match the bridge configuration', () => {
  const saved = config();
  assert.equal(frameMatchesConfiguration(saved, { width: 800, height: 600 }), false);
  assert.equal(frameMatchesConfiguration(saved, { width: 320, height: 240 }), true);
  assert.equal(saved.settings.resolution, 0);
  assert.equal(sameCameraConfiguration(structuredClone(saved), saved), true);
});
test('assembles and checks grayscale snapshot CRC and offset', () => {
  const bytes = Buffer.alloc(320 * 240, 127), checksum = crc32(bytes).toString(16).toUpperCase().padStart(8, '0');
  const snap = new Snapshot(['EVENT','SNAP_BEGIN',mac,'9','320','240',String(bytes.length),checksum]);
  for (let i = 0; i < bytes.length; i += 180) snap.add(['EVENT','SNAP_DATA',mac,String(i),bytes.subarray(i,i+180).toString('hex').toUpperCase()]);
  assert.equal(snap.finish(['EVENT','SNAP_END',mac,'ok']).pixels.length, bytes.length);
  assert.throws(() => snap.add(['EVENT','SNAP_DATA',mac,'0','AA']), /out of order/);
});
test('checks and expands a losslessly compressed frame', () => {
  const pixels = Buffer.alloc(320 * 240);
  for (let i = 0; i < pixels.length; i++) pixels[i] = Math.floor(i / 320) % 8;
  const wire = zlib.deflateRawSync(pixels), checksum = bytes => crc32(bytes).toString(16).toUpperCase().padStart(8, '0');
  const wireChecksum = checksum(wire);
  const snap = new Snapshot(['EVENT','SNAP_BEGIN',mac,'10','320','240',String(pixels.length),checksum(pixels),String(wire.length),wireChecksum,'1']);
  for (let i = 0; i < wire.length; i += 180) snap.add(['EVENT','SNAP_DATA',mac,String(i),wire.subarray(i,i+180).toString('hex').toUpperCase()]);
  assert.deepEqual(snap.finish(['EVENT','SNAP_END',mac,'ok']).pixels, pixels);
  wire[0] ^= 1;
  const damaged = new Snapshot(['EVENT','SNAP_BEGIN',mac,'10','320','240',String(pixels.length),checksum(pixels),String(wire.length),wireChecksum,'1']);
  for (let i = 0; i < wire.length; i += 180) damaged.add(['EVENT','SNAP_DATA',mac,String(i),wire.subarray(i,i+180).toString('hex').toUpperCase()]);
  assert.throws(() => damaged.finish(['EVENT','SNAP_END',mac,'ok']), /transfer CRC/);
});
test('Live view accepts the same saved config with default topics and parsed field order', () => {
  const draft = config();
  draft.topics = {};
  const saved = { revision: 5, settings: { ...draft.settings }, cells: draft.cells.map(cell => ({
    id: cell.id, group: cell.group, x: cell.x, y: cell.y, radius: cell.radius,
    shape: cell.shape, floor: cell.floor, tolerance: cell.tolerance,
    threshold: cell.threshold, enter: cell.enter, clear: cell.clear
  })), topics: { 20: '20' } };
  assert.equal(sameCameraConfiguration(draft, saved), true);
  saved.cells[0].x++;
  assert.equal(sameCameraConfiguration(draft, saved), false);
});
