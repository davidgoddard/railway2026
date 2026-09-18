'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const { areaMessage } = require('./mqtt-monitor');

test('recognizes a retained MQTT deletion and valid sensor state', () => {
  const topic = 'railway/home/areas/platform-1/state';
  assert.deepEqual(areaMessage('railway/home', topic, Buffer.alloc(0)), { topic, name: 'platform-1', removed: true });
  assert.deepEqual(areaMessage('railway/home', topic, Buffer.from('occupied')), { topic, name: 'platform-1', value: 'occupied' });
  assert.equal(areaMessage('railway/home', 'railway/home/areas/platform-1/health', Buffer.from('occupied')), null);
});
