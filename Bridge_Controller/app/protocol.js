'use strict';
const zlib = require('node:zlib');

const RESOLUTIONS = [[320, 240], [640, 480], [800, 600], [1024, 768]];
const MAC = /^[0-9A-F]{2}(?::[0-9A-F]{2}){5}$/;
const SLUG = /^[a-z0-9_-]{1,32}$/;
const DEFAULT_CELL = { radius: 5, shape: 0, floor: 80, threshold: 400, enter: 1, clear: 5, createdRevision: 0 };
function hex(value) { return Buffer.from(value, 'utf8').toString('hex').toUpperCase() || '-'; }
function parseRow(line) {
  const p = line.trim().split(/\s+/);
  if (p[0] === 'CAMERA' && MAC.test(p[1])) return { type: 'camera', mac: p[1], online: p[2] === 'online', revision: +p[3], remoteRevision: +p[4], count: +p[5], baseline: p[6] === '1' };
  if (p[0] === 'CONFIG' && MAC.test(p[1])) return { type: 'config', mac: p[1], revision: +p[2], count: +p[3], lastAutoSizeRevision: +p[4], settings: { resolution: +p[5], brightness: +p[6], contrast: +p[7], saturation: +p[8], vflip: +p[9], hmirror: +p[10] } };
  if (p[0] === 'CELL') return { type: 'cell', cell: { id: +p[2], group: +p[3], x: +p[4], y: +p[5], radius: +p[6], shape: +p[7], floor: +p[8], threshold: +p[9], enter: +p[10], clear: +p[11], createdRevision: +p[12] } };
  if (p[0] === 'TOPIC') return { type: 'topic', id: +p[1], name: p[2] };
  return null;
}
function crc32(bytes) {
  let crc = 0xFFFFFFFF;
  for (const byte of bytes) { crc ^= byte; for (let n = 0; n < 8; n++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xEDB88320 : 0); }
  return (crc ^ 0xFFFFFFFF) >>> 0;
}
class Snapshot {
  constructor(parts) {
    const [, , mac, frame, width, height, count, crc] = parts;
    this.mac = mac; this.frame = +frame; this.width = +width; this.height = +height;
    this.rawCount = +count; this.crc = parseInt(crc, 16) >>> 0; this.offset = 0;
    this.count = parts[8] === undefined ? this.rawCount : +parts[8];
    this.wireCrc = parts[9] === undefined ? this.crc : parseInt(parts[9], 16) >>> 0;
    this.codec = parts[10] === undefined ? 0 : +parts[10];
    if (!MAC.test(mac) || !RESOLUTIONS.some(([w, h]) => w === this.width && h === this.height) || this.rawCount !== this.width * this.height ||
      ![0, 1].includes(this.codec) || !Number.isInteger(this.count) || this.count < 1 || this.count > this.rawCount ||
      (this.codec === 0 && (this.count !== this.rawCount || this.wireCrc !== this.crc))) throw Error('Invalid frame header');
    this.wire = Buffer.alloc(this.count);
  }
  add(parts) {
    const offset = +parts[3], value = parts[4];
    if (parts[2] !== this.mac || offset !== this.offset || !/^(?:[0-9A-F]{2})+$/.test(value || '')) throw Error('Frame chunk is out of order');
    const chunk = Buffer.from(value, 'hex');
    if (offset + chunk.length > this.count) throw Error('Frame exceeds declared size');
    chunk.copy(this.wire, offset); this.offset += chunk.length;
  }
  finish(parts) {
    if (parts[2] !== this.mac || parts[3] !== 'ok' || this.offset !== this.count || crc32(this.wire) !== this.wireCrc) throw Error('Frame failed transfer CRC or length check');
    let pixels;
    try { pixels = this.codec === 1 ? zlib.inflateRawSync(this.wire, { maxOutputLength: this.rawCount }) : this.wire; }
    catch { throw Error('Frame decompression failed'); }
    if (pixels.length !== this.rawCount || crc32(pixels) !== this.crc) throw Error('Frame failed image CRC or length check');
    return { mac: this.mac, frame: this.frame, width: this.width, height: this.height, pixels };
  }
}
function validateConfig(config, otherIds = []) {
  if (!config || !RESOLUTIONS[config.settings?.resolution]) throw Error('Choose a valid resolution');
  const [width, height] = RESOLUTIONS[config.settings.resolution];
  const cells = config.cells || [];
  if (cells.length > 300) throw Error('Camera supports at most 300 sensors');
  const ids = new Set(), groups = new Set(), outputs = new Set(otherIds);
  for (const cell of cells) {
    if (!Number.isInteger(cell.id) || cell.id < 1 || cell.id > 2147483647 || ids.has(cell.id)) throw Error('Sensor IDs must be unique positive numbers');
    ids.add(cell.id);
    if (cell.group) groups.add(cell.group);
    if (!Number.isInteger(cell.x) || cell.x < 0 || cell.x >= width || !Number.isInteger(cell.y) || cell.y < 0 || cell.y >= height) throw Error(`Sensor ${cell.id} at ${cell.x}, ${cell.y} is outside the selected ${width} × ${height} image`);
    if (!Number.isInteger(cell.radius) || cell.radius < 3 || cell.radius > 50) throw Error('Sensor radius must be 3–50 px');
    if (cell.shape !== 0 && cell.shape !== 1) throw Error('Invalid sensor shape');
    if (cell.floor < 10 || cell.floor > 500 || !Number.isInteger(cell.threshold) || cell.threshold < 50 || cell.threshold > 1000 || !Number.isInteger(cell.enter) || cell.enter < 1 || cell.enter > 255 || !Number.isInteger(cell.clear) || cell.clear < 1 || cell.clear > 255 || !Number.isInteger(cell.createdRevision ?? 0) || (cell.createdRevision ?? 0) < 0) throw Error('Invalid sensor thresholds');
  }
  if (groups.size > 64) throw Error('Camera supports at most 64 blocks');
  for (const cell of cells) {
    if (!cell.group && groups.has(cell.id)) throw Error('A sensor ID overlaps a block ID');
    const output = cell.group || cell.id;
    if (outputs.has(output)) throw Error(`Output ID ${output} is already used by another camera`);
  }
  for (const [id, name] of Object.entries(config.topics || {})) if (!SLUG.test(name) || !cells.some(c => (c.group || c.id) === +id)) throw Error('Topic names must be 1–32 lowercase letters, digits, _ or -');
  const names = cells.map(c => config.topics?.[c.group || c.id] || String(c.group || c.id));
  const perOutput = new Map(); cells.forEach((c, i) => perOutput.set(c.group || c.id, names[i]));
  if (new Set(perOutput.values()).size !== perOutput.size) throw Error('Topic names must be unique');
  return true;
}
function commandsForConfig(mac, config) {
  validateConfig(config);
  const s = config.settings;
  return [
    `BEGIN ${mac} ${config.revision + 1} ${config.cells.length} ${s.resolution} ${s.brightness} ${s.contrast} ${s.saturation} ${s.vflip} ${s.hmirror}`,
    ...config.cells.map((c, i) => `CELL ${mac} ${i} ${c.id} ${c.group} ${c.x} ${c.y} ${c.radius} ${c.shape} ${c.floor} ${c.threshold} ${c.enter} ${c.clear} ${c.createdRevision || 0}`),
    `COMMIT ${mac}`,
    ...[...new Set(config.cells.map(c => c.group || c.id))].map(id => `TOPIC ${mac} ${id} ${hex(config.topics?.[id] || String(id))}`)
  ];
}
module.exports = { RESOLUTIONS, DEFAULT_CELL, MAC, SLUG, hex, parseRow, crc32, Snapshot, validateConfig, commandsForConfig };
