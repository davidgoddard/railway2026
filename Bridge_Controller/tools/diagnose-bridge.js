'use strict';

const { SerialPort } = require('serialport');

const serialPath = process.argv[2];
if (!serialPath) throw Error('Usage: node tools/diagnose-bridge.js /dev/cu.usbmodem...');

const port = new SerialPort({ path: serialPath, baudRate: 115200, autoOpen: false });
let input = '';
let cameraMac = '';
let snapBytes = 0;
let snapChunks = 0;
let finished = false;

function write(command) {
  process.stdout.write(`> ${command}\n`);
  port.write(`${command}\n`);
}

function finish(code = 0) {
  if (finished) return;
  finished = true;
  clearTimeout(timeout);
  port.close(() => { process.exitCode = code; });
}

function lineReceived(line) {
  if (!line) return;
  const fields = line.split(' ');
  if (fields[0] === 'CAMERA' && fields[2] === 'online' && !cameraMac) cameraMac = fields[1];
  if (fields[0] === 'EVENT' && fields[1] === 'SNAP_DATA') {
    snapChunks++;
    snapBytes += (fields[4] || '').length / 2;
    if (snapChunks % 250 === 0) process.stdout.write(`... snapshot ${snapBytes} bytes in ${snapChunks} chunks\n`);
    return;
  }
  process.stdout.write(`${line}\n`);
  if (line === 'OK LIST') {
    write('STATUS');
    setTimeout(() => write('LOG'), 300);
    setTimeout(() => {
      if (!cameraMac) {
        process.stderr.write('No camera returned by LIST\n');
        finish(2);
      } else write(`FRAME ${cameraMac}`);
    }, 900);
  }
  if (fields[0] === 'EVENT' && fields[1] === 'SNAP_END') finish(fields[3] === 'ok' ? 0 : 3);
  if (fields[0] === 'EVENT' && fields[1] === 'TIMEOUT' && fields[3] === '6') finish(4);
}

port.on('data', bytes => {
  input += bytes.toString('utf8');
  for (;;) {
    const newline = input.indexOf('\n');
    if (newline < 0) break;
    lineReceived(input.slice(0, newline).replace(/\r$/, ''));
    input = input.slice(newline + 1);
  }
});
port.on('error', error => { process.stderr.write(`${error.message}\n`); finish(1); });

const timeout = setTimeout(() => {
  process.stderr.write('Timed out waiting for diagnosis to finish\n');
  finish(5);
}, 180000);

port.open(error => {
  if (error) {
    process.stderr.write(`${error.message}\n`);
    finish(1);
    return;
  }
  setTimeout(() => write('LIST'), 2000);
});
