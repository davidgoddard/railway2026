'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const {crc32}=require('./protocol');
const {FocusStreamParser}=require('./focus-stream');

function packet(frame,width,height,pixels=Buffer.alloc(width*height,frame)){
  const header=Buffer.alloc(24);header.write('RAILFRM1');header.writeUInt32LE(frame,8);header.writeUInt32LE(pixels.length,12);header.writeUInt32LE(crc32(pixels),16);header.writeUInt16LE(width,20);header.writeUInt16LE(height,22);return Buffer.concat([header,pixels]);
}

test('assembles arbitrarily split frames and retains the following frame',()=>{
  const frames=[],errors=[],wire=Buffer.concat([Buffer.from('console noise\n'),packet(7,8,4),packet(8,5,3)]),parser=new FocusStreamParser(frame=>frames.push(frame),error=>errors.push(error));
  for(let offset=0;offset<wire.length;offset+=7)parser.push(wire.subarray(offset,offset+7));
  assert.deepEqual(frames.map(frame=>[frame.frame,frame.width,frame.height,frame.pixels.length]),[[7,8,4,32],[8,5,3,15]]);assert.deepEqual(errors,[]);
});

test('reports a corrupt frame and resynchronises to the next one',()=>{
  const frames=[],errors=[],bad=packet(1,4,4);bad[24]^=0xff;const parser=new FocusStreamParser(frame=>frames.push(frame),error=>errors.push(error));
  parser.push(Buffer.concat([bad,packet(2,4,4)]));
  assert.equal(errors.length,1);assert.deepEqual(frames.map(frame=>frame.frame),[2]);
});
