'use strict';

const fs = require('node:fs');
const path = require('node:path');
const zlib = require('node:zlib');
const { SerialPort } = require('serialport');

const { crc32, CameraFrameStream } = require('./camera-frame-stream');

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
  const crop = process.argv.includes('--crop');
  const args = process.argv.slice(2).filter(arg => arg !== '--crop');
  const serialPath = args[0];
  const output = path.resolve(args[1] || (crop ? 'camera-crop.png' : 'camera-frame.png'));
  if (args.length > 2) throw Error('Unexpected arguments');
  if (!serialPath || !serialPath.startsWith('/dev/')) {
    throw Error('Usage: node tools/capture-camera-frame.js /dev/cu.usbserial-... [output.png] [--crop]');
  }
  const port = new SerialPort({ path: serialPath, baudRate: 921600, autoOpen: false });
  const stream = new CameraFrameStream();
  let finished = false;
  const base = output.replace(/\.png$/i, '');
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => fail(Error('Timed out waiting for the camera frame')), 180000);
    function fail(error) {
      if (finished) return;
      finished = true; clearTimeout(timer);
      if (crop) {
        process.stderr.write(stream.text);
        try { fs.writeFileSync(`${base}.log`, stream.text); } catch {}
      }
      port.close(() => reject(error));
    }
    port.on('error', fail);
    port.on('data', data => {
      if (finished) return;
      try {
        stream.push(data);
        if (crop && /crop experiment.*(?:unavailable|requires|no smaller|no memory|failed|recovery failed)/i.test(stream.text)) {
          return fail(Error('Crop experiment did not complete; see camera log'));
        }
        if (stream.frames.length < (crop ? 2 : 1)) return;
        if (crop && !stream.text.includes('crop experiment complete: full-frame monitoring restored')) return;
        for (let i = 0; i < (crop ? 2 : 1); i++) {
          const frame = stream.frames[i];
          const filename = crop ? `${base}-${i === 0 ? 'full' : 'crop'}.png` : output;
          fs.writeFileSync(filename, png(frame.width, frame.height, frame.pixels));
          process.stdout.write(`Saved frame ${frame.number} (${frame.width}x${frame.height}) to ${filename}\n`);
        }
        if (crop) {
          fs.writeFileSync(`${base}.log`, stream.text);
          process.stdout.write(stream.text);
        }
      } catch (error) { return fail(error); }
      finished = true; clearTimeout(timer);
      port.close(() => resolve());
    });
    port.open(error => {
      if (error) return fail(error);
      setTimeout(() => { if (!finished) port.write(crop ? 'O FRAME\n' : 'F\n', error => { if (error) fail(error); }); }, 2500);
    });
  });
}

if (require.main === module) main().catch(error => { process.stderr.write(`${error.message}\n`); process.exitCode = 1; });

module.exports = { crc32, png };
