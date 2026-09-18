'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const { EventEmitter } = require('node:events');
const { MqttSubscriber } = require('./mqtt-client');

test('parses split retained and live MQTT publications and confirms subscription', () => {
  const client = new EventEmitter();
  Object.setPrototypeOf(client, MqttSubscriber.prototype);
  client.buffer = Buffer.alloc(0);
  const writes = [];
  client.socket = { write: data => writes.push(data) };
  client.connected = true;
  let subscribed = false;
  client.subscribe('railway/home/areas/+/state', {}, error => { assert.equal(error, null); subscribed = true; });
  assert.match(writes[0].toString(), /railway\/home\/areas\/\+\/state/);
  const messages = [];
  client.on('message', (topic, payload, packet) => messages.push({ topic, value: payload.toString(), retained: packet.retain }));
  const topic = Buffer.from('railway/home/areas/platform-1/state');
  const publication = (state, retained) => {
    const body = Buffer.concat([Buffer.from([0, topic.length]), topic, Buffer.from(state)]);
    return Buffer.concat([Buffer.from([retained ? 0x31 : 0x30, body.length]), body]);
  };
  const data = Buffer.concat([Buffer.from([0x90, 3, 0, 1, 0]), publication('clear', true), publication('occupied', false)]);
  client.onData(data.subarray(0, 7));
  client.onData(data.subarray(7));
  assert.equal(subscribed, true);
  assert.deepEqual(messages.map(({ value, retained }) => [value, retained]), [['clear', true], ['occupied', false]]);
});
