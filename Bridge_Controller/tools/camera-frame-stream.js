'use strict';

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

// Binary frames may be fragmented across USB reads, with text between frames.
class CameraFrameStream {
  constructor() { this.buffer = Buffer.alloc(0); this.pending = null; this.frames = []; this.text = ''; }
  push(data) {
    this.buffer = Buffer.concat([this.buffer, data]);
    for (;;) {
      if (!this.pending) {
        const index = this.buffer.indexOf(magic);
        if (index < 0) {
          const keep = Math.min(this.buffer.length, magic.length - 1);
          this.text += this.buffer.subarray(0, this.buffer.length - keep).toString('utf8');
          this.buffer = this.buffer.subarray(this.buffer.length - keep);
          return;
        }
        this.text += this.buffer.subarray(0, index).toString('utf8');
        this.buffer = this.buffer.subarray(index);
        if (this.buffer.length < headerBytes) return;
        const count = this.buffer.readUInt32LE(12);
        const width = this.buffer.readUInt16LE(20), height = this.buffer.readUInt16LE(22);
        if (!sizes.has(`${width}x${height}`) || count !== width * height) throw Error('Invalid frame header');
        this.pending = { number: this.buffer.readUInt32LE(8), crc: this.buffer.readUInt32LE(16), width, height, count };
        this.buffer = this.buffer.subarray(headerBytes);
      }
      if (this.buffer.length < this.pending.count) return;
      const pixels = this.buffer.subarray(0, this.pending.count);
      if (crc32(pixels) !== this.pending.crc) throw Error('Frame CRC failed; try again');
      this.frames.push({ ...this.pending, pixels });
      this.buffer = this.buffer.subarray(this.pending.count);
      this.pending = null;
    }
  }
}

module.exports = { crc32, CameraFrameStream };
