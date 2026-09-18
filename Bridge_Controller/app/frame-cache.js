'use strict';
const fs = require('node:fs');
const path = require('node:path');
const zlib = require('node:zlib');
const { crc32, MAC, RESOLUTIONS } = require('./protocol');

function cachePath(directory, mac) {
  if (!MAC.test(mac)) throw Error('Invalid camera address');
  return path.join(directory, mac.replaceAll(':', '') + '.frame');
}

function saveFrame(directory, frame) {
  const target = cachePath(directory, frame.mac);
  if (!RESOLUTIONS.some(([w, h]) => w === frame.width && h === frame.height)) throw Error('Invalid frame dimensions');
  const pixels = Buffer.from(frame.pixels);
  if (pixels.length !== frame.width * frame.height) throw Error('Invalid frame length');
  const header = Buffer.from(JSON.stringify({ frame: frame.frame, width: frame.width, height: frame.height,
    crc: crc32(pixels), at: Date.now() }) + '\n');
  const data = Buffer.concat([header, zlib.deflateSync(pixels, { level: 3 })]);
  fs.mkdirSync(directory, { recursive: true });
  const temporary = target + '.' + process.pid + '.tmp';
  try { fs.writeFileSync(temporary, data); fs.renameSync(temporary, target); }
  finally { try { fs.rmSync(temporary, { force: true }); } catch {} }
}

async function loadFrame(directory, mac) {
  const target = cachePath(directory, mac);
  let data;
  try { data = await fs.promises.readFile(target); }
  catch (error) { if (error.code === 'ENOENT') return null; throw error; }
  const newline = data.indexOf(10);
  if (newline < 0 || newline > 256) throw Error('Invalid saved frame header');
  const header = JSON.parse(data.subarray(0, newline).toString('utf8'));
  if (!RESOLUTIONS.some(([w, h]) => w === header.width && h === header.height)) throw Error('Invalid saved frame dimensions');
  const length = header.width * header.height;
  const pixels = zlib.inflateSync(data.subarray(newline + 1), { maxOutputLength: length });
  if (pixels.length !== length || crc32(pixels) !== header.crc) throw Error('Saved frame checksum failed');
  return { mac, frame: header.frame, width: header.width, height: header.height, pixels, at: header.at, cached: true };
}

module.exports = { saveFrame, loadFrame };
