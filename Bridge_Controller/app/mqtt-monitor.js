'use strict';
const mqtt = require('./mqtt-client');
const fs = require('node:fs');
const path = require('node:path');

function validate(settings) {
  const host = String(settings.host || '').trim();
  const port = Number(settings.port);
  const root = String(settings.root || '').trim().replace(/\/+$/, '');
  const user = String(settings.user || '');
  const password = String(settings.password || '');
  if (!host || !/^[a-z0-9.:-]+$/i.test(host) || !Number.isInteger(port) || port < 1 || port > 65535 || !root || root.split('/').some(part => !part || /[+#]/.test(part))) throw Error('Enter a valid broker host, port, and topic root.');
  return { host, port, root, user, password };
}
function areaMessage(root, topic, payload) {
  const prefix = `${root}/areas/`;
  if (!topic.startsWith(prefix) || !topic.endsWith('/state')) return null;
  const name = topic.slice(prefix.length, -'/state'.length);
  if (!name || name.includes('/')) return null;
  const value = payload.toString('utf8');
  if (value === '') return { topic, name, removed: true };
  return ['clear', 'occupied', 'unknown'].includes(value) ? { topic, name, value } : null;
}

function createMonitor(directory, safeStorage, emit) {
  const filename = path.join(directory, 'mqtt-monitor.json');
  let client = null, config = null, status = 'disconnected', detail = '', generation = 0, bridgeOnline = null;
  function notify() { emit('monitor:status', { status, detail, bridgeOnline, configured: !!config, settings: config && { host: config.host, port: config.port, root: config.root, user: config.user } }); }
  function stop() { generation++; if (client) { client.removeAllListeners(); client.end(true); client = null; } status = 'disconnected'; detail = ''; bridgeOnline = null; notify(); }
  function start() {
    stop(); if (!config) return;
    const current = generation;
    status = 'connecting'; notify();
    client = mqtt.connect({ host: config.host, port: config.port, username: config.user || undefined, password: config.password || undefined, clientId: `railway-viewer-${require('node:crypto').randomBytes(6).toString('hex')}` });
    client.on('connect', () => {
      if (current !== generation) return;
      client.subscribe(`${config.root}/areas/+/state`, { qos: 0 }, error => {
        if (current !== generation) return;
        if (error) { status = 'error'; detail = error.message; notify(); return; }
        client.subscribe(`${config.root}/controller/health`, { qos: 0 }, healthError => {
          if (current !== generation) return;
          status = 'connected';
          detail = healthError ? `Monitoring area states; bridge health unavailable: ${healthError.message}` : `Subscribed to ${config.root}/areas/+/state`;
          notify();
        });
      });
    });
    client.on('message', (topic, payload, packet) => {
      if (current !== generation) return;
      if (topic === `${config.root}/controller/health`) { bridgeOnline = payload.toString('utf8') === 'online'; notify(); return; }
      const area = areaMessage(config.root, topic, payload);
      if (!area) return;
      if (area.removed) emit('monitor:removed', area);
      else emit('monitor:message', { ...area, retained: !!packet.retain, at: Date.now() });
    });
    client.on('reconnect', () => { if (current === generation) { status = 'connecting'; detail = 'Reconnecting to broker…'; notify(); } });
    client.on('offline', () => { if (current === generation) { status = 'disconnected'; detail = 'Broker connection lost'; bridgeOnline = null; notify(); } });
    client.on('error', error => { if (current === generation) { status = 'error'; detail = error.message; notify(); } });
  }
  function load() {
    try {
      const saved = JSON.parse(fs.readFileSync(filename, 'utf8'));
      const password = saved.password && safeStorage.isEncryptionAvailable() ? safeStorage.decryptString(Buffer.from(saved.password, 'base64')) : '';
      config = validate({ ...saved, password });
      start();
    } catch (error) { if (error.code !== 'ENOENT') { status = 'error'; detail = `Could not load saved monitor settings: ${error.message}`; notify(); } }
  }
  function save(settings) {
    const next = validate(settings);
    const saved = { host: next.host, port: next.port, root: next.root, user: next.user };
    if (next.password) {
      if (!safeStorage.isEncryptionAvailable()) throw Error('Secure password storage is unavailable on this computer.');
      saved.password = safeStorage.encryptString(next.password).toString('base64');
    }
    fs.mkdirSync(directory, { recursive: true });
    fs.writeFileSync(filename, JSON.stringify(saved), { mode: 0o600 });
    config = next; start(); return true;
  }
  function forget() { stop(); config = null; try { fs.unlinkSync(filename); } catch (error) { if (error.code !== 'ENOENT') throw error; } notify(); }
  return { load, save, forget, stop, notify };
}
module.exports = { createMonitor, validate, areaMessage };
