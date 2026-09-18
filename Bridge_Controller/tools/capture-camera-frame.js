'use strict';

const fs = require('node:fs');
const path = require('node:path');
const zlib = require('node:zlib');
const { SerialPort } = require('serialport');

const magic = Buffer.from('RAILFRM1');
const headerBytes = 24;
const sizes = new Set(['320x240', '640x480', '800x600', '1024x768']);

function crc32(bytes) {
  let crc = 0xFFFFFFFF;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
  }
  return (crc ^ 0xFFFFFFFF) >>> 0;
}

function chunk(type, data) {
  const name = Buffer.from(type);
  const result = Buffer.alloc(12 + data.length);
  result.writeUInt32BE(data.length, 0);
  name.copy(result, 4);
  data.copy(result, 8);
  result.writeUInt32BE(crc32(result.subarray(4, 8 + data.length)), 8 + data.length);
  return result;
}

function png(width, height, pixels) {
  const header = Buffer.alloc(13);
  header.writeUInt32BE(width, 0);
  header.writeUInt32BE(height, 4);
  header[8] = 8; // Eight-bit grayscale.
  const rows = Buffer.alloc(height * (width + 1));
  for (let y = 0; y < height; y++) pixels.copy(rows, y * (width + 1) + 1, y * width, (y + 1) * width);
  return Buffer.concat([
    Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]),
    chunk('IHDR', header),
    chunk('IDAT', zlib.deflateSync(rows)),
    chunk('IEND', Buffer.alloc(0))
  ]);
}

async function main() {
  const serialPath = process.argv[2];
  const output = path.resolve(process.argv[3] || 'camera-frame.png');
  if (!serialPath || !serialPath.startsWith('/dev/')) {
    throw Error('Usage: node tools/capture-camera-frame.js /dev/cu.usbserial-... [output.png]');
  }
  const port = new SerialPort({ path: serialPath, baudRate: 115200, autoOpen: false });
  let buffer = Buffer.alloc(0), frame = null, finished = false;
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => fail(Error('Timed out waiting for the camera frame')), 180000);
    function fail(error) {
      if (finished) return;
      finished = true; clearTimeout(timer); port.close(() => reject(error));
    }
    port.on('error', fail);
    port.on('data', data => {
      if (finished) return;
      buffer = Buffer.concat([buffer, data]);
      if (!frame) {
        const index = buffer.indexOf(magic);
        if (index < 0) { buffer = buffer.subarray(Math.max(0, buffer.length - magic.length + 1)); return; }
        if (buffer.length - index < headerBytes) { buffer = buffer.subarray(index); return; }
        const header = buffer.subarray(index, index + headerBytes);
        const count = header.readUInt32LE(12), width = header.readUInt16LE(20), height = header.readUInt16LE(22);
        if (!sizes.has(`${width}x${height}`) || count !== width * height) return fail(Error('Invalid frame header'));
        frame = { number: header.readUInt32LE(8), crc: header.readUInt32LE(16), width, height, count };
        buffer = buffer.subarray(index + headerBytes);
      }
      if (buffer.length < frame.count) return;
      const pixels = buffer.subarray(0, frame.count);
      if (crc32(pixels) !== frame.crc) return fail(Error('Frame CRC failed; try again'));
      try { fs.writeFileSync(output, png(frame.width, frame.height, pixels)); }
      catch (error) { return fail(error); }
      finished = true; clearTimeout(timer);
      port.close(() => resolve());
      process.stdout.write(`Saved frame ${frame.number} (${frame.width}x${frame.height}) to ${output}\n`);
    });
    port.open(error => {
      if (error) return fail(error);
      setTimeout(() => { if (!finished) port.write('F\n', error => { if (error) fail(error); }); }, 2500);
    });
  });
}

if (require.main === module) main().catch(error => { process.stderr.write(`${error.message}\n`); process.exitCode = 1; });

module.exports = { crc32, png };
