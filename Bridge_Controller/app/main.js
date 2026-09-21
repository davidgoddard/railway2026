'use strict';
const { app, BrowserWindow, ipcMain, safeStorage } = require('electron');
const { SerialPort } = require('serialport');
const path = require('node:path');
const { parseRow, Snapshot, validateConfig, commandsForConfig, MAC } = require('./protocol');
const { saveFrame, loadFrame } = require('./frame-cache');
const { createMonitor } = require('./mqtt-monitor');

let win, port, pending, queue = Promise.resolve(), buffer = '', snapshot, refreshTimer, frameTimer, requestedFrameMac = '';
const configs = new Map(), cameras = new Map(), states = new Map(), cellStates = new Map();
const healthSamples = new Map();
let network = { wifi: 'disconnected', wifiDetail: '', mqtt: 'disconnected', mqttDetail: '', wifiSaved: null, mqttSaved: null };
let monitor;
let monitorStarted = false;
function frameDirectory() { return path.join(app.getPath('userData'), 'camera-frames'); }
function send(type, payload) { if (win && !win.isDestroyed()) win.webContents.send(type, payload); }
function state() { send('bridge:state', { connected: !!port?.isOpen, path: port?.path || '', cameras: [...cameras.values()], configs: Object.fromEntries(configs), states: Object.fromEntries(states), cellStates: Object.fromEntries(cellStates), network }); }
function failPending(message) { if (pending) { clearTimeout(pending.timer); pending.reject(Error(message)); pending = null; } }
function clearFrameRequest() { clearTimeout(frameTimer); frameTimer = null; requestedFrameMac = ''; }
function expectFrame(mac) {
  clearFrameRequest();requestedFrameMac = mac;
  frameTimer = setTimeout(() => { if (requestedFrameMac !== mac) return; clearFrameRequest();snapshot = null;send('bridge:error', `Frame request from ${mac} timed out before a complete image arrived.`); }, 120000);
}
function diagnosticFromParts(parts, start, at = Date.now()) {
  if (parts.length < start + 6 || !/^[0-9A-F]{8}$/.test(parts[start]) || !/^\d+$/.test(parts[start + 1])) return null;
  return { boot: parts[start], sequence: +parts[start + 1], uptime: +parts[start + 2], code: parts[start + 3], mac: parts[start + 4], detail: parts.slice(start + 5).join(' '), at };
}
function updateRadioFromDiagnostic(event) {
  if (!event || !['WIFI_RADIO', 'CHANNEL_CHANGE'].includes(event.code)) return false;
  const match = new RegExp(`\\b${event.code === 'CHANNEL_CHANGE' ? 'to' : 'channel'}=(\\d+)`).exec(event.detail);
  const channel = match ? +match[1] : 0;
  if (channel < 1 || channel > 13 || network.radioChannel === channel) return false;
  network.radioChannel = channel;
  return true;
}
async function logs() {
  const rows = await run('LOG');
  const now = rows.find(line => line.startsWith('LOG_NOW '))?.split(' ');
  const bridgeUptime = now ? +now[2] : 0, wallNow = Date.now();
  for (const row of rows) {
    if (!row.startsWith('DIAG ')) continue;
    const parts = row.split(' ');
    const at = now && parts[1] === now[1] ? wallNow - ((bridgeUptime - (+parts[3])) >>> 0) : wallNow;
    const event = diagnosticFromParts(parts, 1, at);
    if (event) { updateRadioFromDiagnostic(event); send('bridge:diagnostic', event); }
  }
  state();
  return rows.length;
}
function handleLine(line) {
  if (!line) return;
  const parts = line.split(' ');
  if (parts[0] === 'READY') {
    const channel = +(/\bchannel=(\d+)/.exec(line)?.[1] || 0);
    if (channel >= 1 && channel <= 13) network.radioChannel = channel;
    state(); send('bridge:notice', `Bridge ${parts[2]} connected`); return;
  }
  if (parts[0] === 'EVENT') {
    const kind = parts[1], mac = parts[2];
    try {
      if (kind === 'CONFIG_ERROR' && parts[3] === '6') clearFrameRequest();
      if (kind === 'WIFI') { network.wifi = parts[2]; network.wifiDetail = parts.slice(3).join(' '); state(); send('bridge:network-event', { service: 'Wi-Fi', status: network.wifi, detail: network.wifiDetail }); }
      else if (kind === 'DIAG') { const event = diagnosticFromParts(parts, 2); if (event) { if (updateRadioFromDiagnostic(event)) state(); send('bridge:diagnostic', event); } }
      else if (kind === 'MQTT_STATUS') { network.mqtt = parts[2]; network.mqttDetail = parts.slice(3).join(' '); state(); send('bridge:network-event', { service: 'MQTT broker', status: network.mqtt, detail: network.mqttDetail }); }
      else if (kind === 'SNAP_BEGIN') { snapshot = new Snapshot(parts); send('bridge:progress', { mac, received: 0, total: snapshot.count }); }
      else if (kind === 'SNAP_DATA' && snapshot) { snapshot.add(parts); if (snapshot.offset % 12000 < 180) send('bridge:progress', { mac, received: snapshot.offset, total: snapshot.count }); }
      else if (kind === 'SNAP_END' && snapshot) { clearFrameRequest();if (parts[3] === 'timeout') { snapshot = null; send('bridge:error', 'Frame transfer stalled or the camera went offline. Wait for it to reconnect, then try again.'); } else { const frame = snapshot.finish(parts); snapshot = null; try { saveFrame(frameDirectory(), frame); } catch (error) { send('bridge:error', `Could not keep this frame for next time: ${error.message}`); } send('bridge:frame', frame); send('bridge:notice', `Frame received from ${mac}`); } }
      else if (kind === 'STATE') { const entry = { mac, id: +parts[3], value: parts[4], score: +parts[5], frame: +parts[6], at: Date.now() }; states.set(`${mac}:${parts[3]}`, entry); send('bridge:state-event', entry); }
      else if (kind === 'CELL_STATE') { const entry = { mac, id: +parts[3], value: parts[4], score: +parts[5], frame: +parts[6], at: Date.now() }; cellStates.set(`${mac}:${parts[3]}`, entry); send('bridge:cell-state-event', entry); }
      else if (kind === 'CELL_ANALYSIS') {
        const numbers=value=>value.split(',').map(Number);
        const entry={mac,id:+parts[3],frame:+parts[4],score:+parts[5],threshold:+parts[6],referenceEdges:+parts[7],liveEdges:+parts[8],peakCount:+parts[9],angles:numbers(parts[10]),reference:numbers(parts[11]),live:numbers(parts[12]),at:Date.now()};
        if(entry.angles.length!==10||entry.reference.length!==31||entry.live.length!==31||entry.peakCount<0||entry.peakCount>10||[entry.id,entry.frame,entry.score,entry.threshold,entry.referenceEdges,entry.liveEdges,...entry.angles,...entry.reference,...entry.live].some(value=>!Number.isFinite(value)))throw Error('Invalid cell analysis from bridge');
        send('bridge:cell-analysis',entry);
      }
      else if (kind === 'DISCOVER') { send('bridge:notice', `Camera ${mac} discovered`); refresh().catch(report); }
      else if (kind === 'CONFIG_APPLIED') { send('bridge:notice', `Camera ${mac} applied revision ${parts[3]}`); refresh().catch(report); }
      else if (kind === 'CONFIG_ERROR' && parts[3] === '5') {
        const reason = { 1: 'The camera rejected the baseline request data.', 2: 'The camera has not applied its configuration yet, or a snapshot is still active. Save changes and wait for the camera revision to match.', 3: 'The camera ran out of memory.', 4: 'The camera could not capture or save the baseline. Check its LittleFS storage and camera serial log.', 5: 'The camera is busy; wait for the current operation to finish.' }[parts[4]] || `Camera error code ${parts[4]}`;
        send('bridge:request-result', { mac, kind: 'baseline', ok: false, reason, raw: line });
      }
      else if (kind === 'TIMEOUT' && parts[3] === '5') send('bridge:request-result', { mac, kind: 'baseline', ok: false, reason: 'The camera did not acknowledge the baseline request. Check that it is online and wait for any transfer to finish.', raw: line });
      else if (kind === 'REQUEST_ACK' && parts[3] === '5') send('bridge:request-result', { mac, kind: 'baseline', ok: parts[4] === '0', reason: parts[4] === '0' ? 'The camera captured and saved a new baseline.' : `Camera status ${parts[4]}`, raw: line });
      else if (kind === 'CONFIG_ERROR' && parts[3] === '6' && parts[4] === '5') send('bridge:error', 'Camera is still transferring a frame, or has not captured one yet. Wait for the transfer to finish and try again.');
      else if (kind === 'TIMEOUT' && ['2','3','4'].includes(parts[3])) send('bridge:request-result', { mac, kind: 'config', ok: false, reason: `Saved on the bridge, but the camera did not acknowledge configuration step ${parts[3]}. The bridge will retry when the camera announces itself again.`, raw: line });
      else if (kind === 'CONFIG_ERROR' && ['2','3','4'].includes(parts[3])) send('bridge:request-result', { mac, kind: 'config', ok: false, reason: `Saved on the bridge, but the camera rejected configuration step ${parts[3]} with status ${parts[4]}.`, raw: line });
      else if (kind === 'TIMEOUT' && parts[3] === '6') { clearFrameRequest();send('bridge:error', 'The camera did not respond to the frame request. Check that it is online, then try again.'); }
      else if (kind === 'CONFIG_ERROR' || kind === 'TIMEOUT') send('bridge:error', line);
      else if (kind === 'CAMERA') {
        if (parts[3] === 'offline' && requestedFrameMac === mac) { clearFrameRequest();snapshot = null;send('bridge:error', `Camera ${mac} went offline before its frame arrived.`); }
        for (const [key, value] of states) if (value.mac === mac) states.set(key, { ...value, value: 'unknown', score: null, frame: null, at: Date.now() });
        for (const [key, value] of cellStates) if (value.mac === mac) cellStates.set(key, { ...value, value: 'unknown', score: null, frame: null, at: Date.now() });
        const c = cameras.get(mac); if (c && parts[3] === 'offline') { c.online = false; c.fps = null; healthSamples.delete(mac); }
        state();
      }
      else if (kind === 'HEALTH') { const c = cameras.get(mac); if (c) {
        const frame = Number(parts[4]), now = Date.now(), snapshotActive = parts[7] === '1';
        const previous = healthSamples.get(mac);
        c.fps = Number.isSafeInteger(frame) && previous && !snapshotActive && !previous.snapshotActive &&
          frame >= previous.frame && now - previous.at >= 1000 && now - previous.at <= 15000
          ? Math.round((frame - previous.frame) * 10000 / (now - previous.at)) / 10 : null;
        if (Number.isSafeInteger(frame)) healthSamples.set(mac, { frame, at: now, snapshotActive });
        c.baseline = parts[6] === '1'; c.snapshotActive = snapshotActive; c.online = true; state();
      } }
      else if (kind === 'MQTT') send('bridge:mqtt', { topic: parts[2], value: parts.slice(3).join(' '), at: Date.now() });
    } catch (e) { snapshot = null; send('bridge:error', e.message); }
    send('bridge:event', line); return;
  }
  if (pending) {
    if (parts[0] === 'OK' || parts[0] === 'ERR') {
      const active = pending; pending = null; clearTimeout(active.timer);
      if (parts[0] === 'ERR') active.reject(Error(line)); else active.resolve(active.rows);
    } else pending.rows.push(line);
  }
}
function onData(chunk) {
  buffer += chunk.toString('utf8');
  if (buffer.length > 200000) { buffer = ''; send('bridge:error', 'Serial line exceeded size limit'); }
  let end; while ((end = buffer.indexOf('\n')) >= 0) { const line = buffer.slice(0, end).trim(); buffer = buffer.slice(end + 1); handleLine(line); }
}
function run(command, timeout = 12000) {
  const task = queue.catch(() => {}).then(() => new Promise((resolve, reject) => {
    if (!port?.isOpen) return reject(Error('Connect a bridge first'));
    if (!/^[\x20-\x7E]+$/.test(command)) return reject(Error('Invalid command text'));
    pending = { rows: [], resolve, reject, timer: setTimeout(() => failPending(`Timed out: ${command.split(' ')[0]}`), timeout) };
    port.write(command + '\n', err => { if (err) failPending(err.message); });
  })); queue = task; return task;
}
function report(error) { send('bridge:error', error.message || String(error)); }
async function refresh() {
  const rows = await run('LIST');
  const next = new Map();
  for (const line of rows) { const row = parseRow(line); if (row?.type === 'camera') next.set(row.mac, row); }
  const previousCameras = new Map(cameras);
  cameras.clear(); for (const [mac, camera] of next) cameras.set(mac, { ...camera, fps: previousCameras.get(mac)?.fps ?? null, snapshotActive: previousCameras.get(mac)?.snapshotActive ?? false });
  for (const mac of healthSamples.keys()) if (!cameras.has(mac)) healthSamples.delete(mac);
  for (const mac of cameras.keys()) {
    const getRows = await run(`GET ${mac}`);
    const config = { revision: 0, cells: [], topics: {}, settings: { resolution: 0, brightness: 0, contrast: 0, saturation: 0, vflip: 0, hmirror: 0 } };
    for (const line of getRows) { const row = parseRow(line); if (row?.type === 'config') { config.revision = row.revision; config.settings = row.settings; } else if (row?.type === 'cell') config.cells.push(row.cell); else if (row?.type === 'topic') config.topics[row.id] = row.name; }
    configs.set(mac, config);
  }
  for (const mac of configs.keys()) if (!cameras.has(mac)) configs.delete(mac);
  let stateRows;
  try { stateRows = await run('STATES'); }
  catch (error) { if (/ERR CAMERA unknown/.test(error.message)) throw Error('Bridge firmware is too old for this app. Upload the updated Bridge_Controller.ino.'); throw error; }
  const currentCells = new Set();
  for (const line of stateRows) { const p = line.split(' '); if (p[0] === 'OUTPUT' && MAC.test(p[1])) { const previous=states.get(`${p[1]}:${p[2]}`);states.set(`${p[1]}:${p[2]}`, { mac: p[1], id: +p[2], value: p[3], score: previous?.value===p[3]?previous.score:null, frame: previous?.value===p[3]?previous.frame:null, at: Date.now() }); } else if (p[0] === 'SENSOR' && MAC.test(p[1])) { const key=`${p[1]}:${p[2]}`,previous=cellStates.get(key);currentCells.add(key);cellStates.set(key,{mac:p[1],id:+p[2],value:p[3],score:previous?.value===p[3]?previous.score:null,frame:previous?.value===p[3]?previous.frame:null,at:Date.now()}); } }
  for (const key of cellStates.keys()) if (!currentCells.has(key)) cellStates.delete(key);
  const statusRows = await run('STATUS');
  for (const line of statusRows) { const p = line.split(' '); if (p[0] === 'RADIO_STATUS') { network.radioChannel = +p[1]; network.actualRadioChannel = +p[2]; } else if (p[0] === 'WIFI_STATUS' && !(network.wifi === 'failed' && p[1] === 'disconnected')) { network.wifi = p[1]; network.wifiDetail = p[2] === '-' ? '' : p.slice(2).join(' '); } else if (p[0] === 'MQTT_STATUS' && !(network.mqtt === 'failed' && p[1] === 'disconnected')) { network.mqtt = p[1]; network.mqttDetail = p[2] || ''; } else if (p[0] === 'SAVED_SETTINGS') { network.wifiSaved = p[1] === '1'; network.mqttSaved = p[2] === '1'; } }
  state(); return true;
}
async function connect(serialPath) {
  if (port?.isOpen) await disconnect();
  const available = await SerialPort.list();
  if (!available.some(p => p.path === serialPath)) throw Error('Selected serial port is unavailable');
  const next = new SerialPort({ path: serialPath, baudRate: 115200, autoOpen: false });
  try { await new Promise((resolve, reject) => next.open(err => err ? reject(err) : resolve())); }
  catch (error) {
    if (/busy|access denied|permission denied|EBUSY/i.test(error.message)) throw Error('USB port is in use. Close Arduino Serial Monitor and any other serial app, then try again.');
    throw error;
  }
  port = next; buffer = ''; queue = Promise.resolve();
  port.on('data', onData);
  port.on('error', err => { failPending(err.message); report(err); });
  port.on('close', () => { failPending('Bridge disconnected'); clearFrameRequest();port = null; clearInterval(refreshTimer); state(); });
  state();
  try { await refresh(); }
  catch (error) { await disconnect(); if (/Timed out: LIST/.test(error.message)) throw Error('The selected port did not answer as a Railway bridge. Check the USB cable, firmware and board serial settings.'); throw error; }
  logs().catch(() => {});
  refreshTimer = setInterval(() => refresh().catch(report), 7000);
}
async function disconnect() {
  clearInterval(refreshTimer); clearFrameRequest();failPending('Bridge disconnected');
  if (port?.isOpen) await new Promise(resolve => port.close(() => resolve()));
  port = null; snapshot = null; cameras.clear(); configs.clear(); states.clear(); cellStates.clear(); network = { wifi: 'disconnected', wifiDetail: '', mqtt: 'disconnected', mqttDetail: '', wifiSaved: null, mqttSaved: null }; state();
}
function createWindow() {
  win = new BrowserWindow({ width: 1440, height: 900, minWidth: 1080, minHeight: 700, backgroundColor: '#0c1420', title: 'Railway Bridge Controller', webPreferences: { preload: path.join(__dirname, 'preload.js'), contextIsolation: true, nodeIntegration: false, sandbox: true } });
  win.loadFile(path.join(__dirname, 'index.html'));
  win.webContents.on('did-finish-load', () => { state(); if (!monitorStarted) { monitorStarted = true; monitor.load(); } else monitor.notify(); });
}
app.whenReady().then(() => {
  monitor = createMonitor(app.getPath('userData'), safeStorage, send);
  ipcMain.handle('monitor-save', (_, settings) => monitor.save(settings));
  ipcMain.handle('monitor-forget', () => monitor.forget());
  ipcMain.handle('ports', () => SerialPort.list().then(list => list.map(p => ({ path: p.path, manufacturer: p.manufacturer || '', vendorId: p.vendorId || '', productId: p.productId || '' }))));
  ipcMain.handle('connect', (_, serialPath) => connect(serialPath));
  ipcMain.handle('disconnect', disconnect);
  ipcMain.handle('refresh', refresh);
  ipcMain.handle('logs', logs);
  ipcMain.handle('frame', async (_, mac) => { if (!MAC.test(mac)) throw Error('Invalid camera'); expectFrame(mac);try { return await run(`FRAME ${mac}`); } catch (error) { clearFrameRequest();throw error; } });
  ipcMain.handle('cached-frame', (_, mac) => loadFrame(frameDirectory(), mac).catch(() => null));
  ipcMain.handle('baseline', (_, mac) => { if (!MAC.test(mac)) throw Error('Invalid camera'); return run(`BASELINE ${mac}`); });
  ipcMain.handle('calibrate', (_, mac) => { if (!MAC.test(mac)) throw Error('Invalid camera'); return run(`CALIBRATE ${mac}`); });
  ipcMain.handle('analysis', (_, mac, id) => { if (!MAC.test(mac) || !Number.isInteger(id) || id < 1) throw Error('Invalid sensor'); return run(`ANALYSIS ${mac} ${id}`); });
  ipcMain.handle('save', async (_, mac, config) => {
    if (!MAC.test(mac) || !configs.has(mac)) throw Error('Unknown camera');
    const otherIds = [...configs].filter(([id]) => id !== mac).flatMap(([, c]) => c.cells.map(cell => cell.group || cell.id));
    validateConfig(config, otherIds);
    const otherTopics = new Set([...configs].filter(([id]) => id !== mac).flatMap(([, c]) => [...new Set(c.cells.map(cell => cell.group || cell.id))].map(id => c.topics?.[id] || String(id))));
    for (const id of new Set(config.cells.map(cell => cell.group || cell.id))) if (otherTopics.has(config.topics?.[id] || String(id))) throw Error(`Topic name for area ${id} is already used by another camera`);
    const current = configs.get(mac); config.revision = current.revision;
    for (const command of commandsForConfig(mac, config)) await run(command, 30000);
    await refresh(); return true;
  });
  ipcMain.handle('wifi', (_, ssid, password) => run(`WIFI ${require('./protocol').hex(ssid)} ${require('./protocol').hex(password)}`));
  ipcMain.handle('mqtt', (_, host, portNumber, root, user, password) => run(`MQTT ${require('./protocol').hex(host)} ${portNumber} ${require('./protocol').hex(root)} ${require('./protocol').hex(user)} ${require('./protocol').hex(password)}`));
  ipcMain.handle('forget-wifi', () => run('FORGET_WIFI'));
  ipcMain.handle('forget-mqtt', () => run('FORGET_MQTT'));
  createWindow();
  app.on('activate', () => { if (BrowserWindow.getAllWindows().length === 0) createWindow(); });
});
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit(); });
app.on('before-quit', () => monitor?.stop());
