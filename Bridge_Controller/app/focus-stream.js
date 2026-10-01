'use strict';
const { crc32 } = require('./protocol');

const MAGIC=Buffer.from('RAILFRM1'),HEADER_BYTES=24,MAX_FRAME_BYTES=64*1024*1024;

class FocusStreamParser {
  constructor(onFrame,onError){this.onFrame=onFrame;this.onError=onError;this.reset();}
  reset(){this.buffer=Buffer.alloc(0);this.receive=null;}
  push(chunk){
    let data=Buffer.from(chunk);
    while(data.length){
      if(!this.receive){
        const candidate=this.buffer.length?Buffer.concat([this.buffer,data]):data,start=candidate.indexOf(MAGIC);
        if(start<0){this.buffer=Buffer.from(candidate.subarray(Math.max(0,candidate.length-MAGIC.length+1)));return;}
        if(candidate.length-start<HEADER_BYTES){this.buffer=Buffer.from(candidate.subarray(start));return;}
        const header=candidate.subarray(start,start+HEADER_BYTES),frame=header.readUInt32LE(8),bytes=header.readUInt32LE(12),checksum=header.readUInt32LE(16),width=header.readUInt16LE(20),height=header.readUInt16LE(22);
        if(!width||!height||bytes!==width*height||bytes>MAX_FRAME_BYTES){
          this.onError(`Camera returned an invalid frame header (${width}x${height}, ${bytes} bytes)`);
          this.buffer=Buffer.alloc(0);data=candidate.subarray(start+MAGIC.length);continue;
        }
        this.receive={frame,bytes,checksum,width,height,pixels:Buffer.allocUnsafe(bytes),received:0};this.buffer=Buffer.alloc(0);data=candidate.subarray(start+HEADER_BYTES);
      }
      const count=Math.min(data.length,this.receive.bytes-this.receive.received);
      if(count){data.copy(this.receive.pixels,this.receive.received,0,count);this.receive.received+=count;data=data.subarray(count);}
      if(this.receive.received<this.receive.bytes)return;
      const completed=this.receive;this.receive=null;
      if(crc32(completed.pixels)!==completed.checksum){this.onError(`Camera frame ${completed.frame} failed its checksum; resynchronising`);continue;}
      this.onFrame({frame:completed.frame,width:completed.width,height:completed.height,pixels:Uint8Array.from(completed.pixels)});
    }
  }
}

module.exports={FocusStreamParser};
