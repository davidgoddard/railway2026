'use strict';
const { contextBridge, ipcRenderer } = require('electron');
const events = ['bridge:state', 'bridge:notice', 'bridge:error', 'bridge:event', 'bridge:state-event', 'bridge:cell-state-event', 'bridge:mqtt', 'bridge:progress', 'bridge:frame', 'bridge:network-event', 'bridge:request-result', 'bridge:diagnostic'];
contextBridge.exposeInMainWorld('bridge', {
  ports: () => ipcRenderer.invoke('ports'), connect: path => ipcRenderer.invoke('connect', path), disconnect: () => ipcRenderer.invoke('disconnect'), refresh: () => ipcRenderer.invoke('refresh'),
  frame: mac => ipcRenderer.invoke('frame', mac), cachedFrame: mac => ipcRenderer.invoke('cached-frame', mac), baseline: mac => ipcRenderer.invoke('baseline', mac), save: (mac, config) => ipcRenderer.invoke('save', mac, config),
  wifi: (ssid, password) => ipcRenderer.invoke('wifi', ssid, password), mqtt: (host, port, root, user, password) => ipcRenderer.invoke('mqtt', host, port, root, user, password),
  forgetWifi: () => ipcRenderer.invoke('forget-wifi'), forgetMqtt: () => ipcRenderer.invoke('forget-mqtt'),
  logs: () => ipcRenderer.invoke('logs'),
  on: (name, callback) => { if (!events.includes(name)) throw Error('Unknown event'); const listener = (_, data) => callback(data); ipcRenderer.on(name, listener); return () => ipcRenderer.removeListener(name, listener); }
});
