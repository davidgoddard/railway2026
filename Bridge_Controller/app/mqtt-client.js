'use strict';
const net = require('node:net');
const { EventEmitter } = require('node:events');

function field(value) { const bytes = Buffer.from(value, 'utf8'), out = Buffer.alloc(2); out.writeUInt16BE(bytes.length); return Buffer.concat([out, bytes]); }
function packet(type, body) { let length = body.length; const size = []; do { let digit = length % 128; length = Math.floor(length / 128); if (length) digit |= 128; size.push(digit); } while (length); return Buffer.concat([Buffer.from([type, ...size]), body]); }

class MqttSubscriber extends EventEmitter {
  constructor(options) { super(); this.options = options; this.closed = false; this.timer = null; this.socket = null; this.buffer = Buffer.alloc(0); this.connected = false; this.subscribeCallback = null; this.open(); }
  open() {
    if (this.closed) return;
    this.buffer = Buffer.alloc(0); this.connected = false; this.subscribeCallback = null;
    const socket = this.socket = net.createConnection({ host: this.options.host, port: this.options.port });
    socket.setTimeout(45000);
    socket.on('connect', () => {
      const { clientId, username, password } = this.options;
      const flags = 2 | (username ? 0x80 : 0) | (password ? 0x40 : 0);
      const body = [field('MQTT'), Buffer.from([4, flags, 0, 30]), field(clientId)];
      if (username) body.push(field(username));
      if (password) body.push(field(password));
      socket.write(packet(0x10, Buffer.concat(body)));
    });
    socket.on('data', chunk => { try { this.onData(chunk); } catch (error) { this.emit('error', error); socket.destroy(); } });
    socket.on('timeout', () => socket.destroy(Error('Broker timed out')));
    socket.on('error', error => this.emit('error', error));
    socket.on('close', () => { this.connected = false; clearInterval(this.ping); this.emit('offline'); if (!this.closed) this.timer = setTimeout(() => { this.emit('reconnect'); this.open(); }, 3000); });
  }
  onData(chunk) {
    this.buffer = Buffer.concat([this.buffer, chunk]);
    if (this.buffer.length > 1024 * 1024) throw Error('MQTT packet exceeds size limit');
    while (this.buffer.length >= 2) {
      let size = 0, multiplier = 1, pos = 1, digit;
      do { if (pos >= this.buffer.length) return; digit = this.buffer[pos++]; size += (digit & 127) * multiplier; multiplier *= 128; if (pos > 5 || size > 1024 * 1024) throw Error('Invalid MQTT packet length'); } while (digit & 128);
      if (this.buffer.length < pos + size) return;
      const header = this.buffer[0], body = this.buffer.subarray(pos, pos + size);
      this.buffer = this.buffer.subarray(pos + size);
      if ((header >> 4) === 2) {
        if (body.length !== 2 || body[1] !== 0) throw Error(`Broker rejected MQTT connection (${body[1] ?? 'invalid reply'})`);
        this.connected = true; this.emit('connect');
        this.ping = setInterval(() => { if (this.connected) this.socket.write(Buffer.from([0xc0, 0])); }, 20000);
      } else if ((header >> 4) === 3) {
        if (body.length < 2) throw Error('Invalid MQTT publication');
        const nameLength = body.readUInt16BE(0), qos = (header >> 1) & 3;
        if (body.length < nameLength + 2 + (qos ? 2 : 0)) throw Error('Invalid MQTT publication');
        const topic = body.subarray(2, 2 + nameLength).toString('utf8');
        const offset = 2 + nameLength + (qos ? 2 : 0);
        if (qos === 1) this.socket.write(packet(0x40, body.subarray(2 + nameLength, offset)));
        this.emit('message', topic, body.subarray(offset), { retain: !!(header & 1) });
      } else if ((header >> 4) === 9) {
        const callback = this.subscribeCallback; this.subscribeCallback = null;
        callback?.(body.length < 3 || body[2] >= 0x80 ? Error(`Broker rejected subscription (${body[2] ?? 'invalid reply'})`) : null);
      }
    }
  }
  subscribe(topic, _options, callback) {
    if (!this.connected) return callback(Error('Broker is not connected'));
    const id = Buffer.from([0, 1]);
    this.subscribeCallback = callback;
    this.socket.write(packet(0x82, Buffer.concat([id, field(topic), Buffer.from([0])])));
  }
  end() { this.closed = true; clearTimeout(this.timer); clearInterval(this.ping); this.socket?.destroy(); }
}
module.exports = { connect: options => new MqttSubscriber(options), MqttSubscriber };
