const test=require('node:test');
const assert=require('node:assert/strict');
const SensorTest=require('./sensor-test');

test('finds middle sensors by nearest neighbours and excludes endpoints',()=>{
  const cells=[0,10,20,30].map((x,index)=>({id:index+1,group:7,x,y:index===2?2:0}));
  assert.deepEqual(SensorTest.candidates(cells).map(item=>item.id),[2,3]);
});

test('does not use sensors from another block as neighbours',()=>{
  const cells=[{id:1,group:7,x:0,y:0},{id:2,group:7,x:10,y:0},{id:3,group:7,x:20,y:0},{id:4,group:8,x:9,y:0}];
  assert.deepEqual(SensorTest.candidates(cells).map(item=>item.neighbors),[[1,3]]);
});

test('counts passes, misses, and the tested sensor peak score',()=>{
  const session=SensorTest.create([{id:1,group:7,x:0,y:0},{id:2,group:7,x:10,y:0},{id:3,group:7,x:20,y:0}]);
  session.ingest({id:1,value:'occupied',at:1000,score:500});
  session.ingest({id:2,value:'clear',at:1500,score:390},false);
  session.ingest({id:2,value:'occupied',at:2000,score:500});
  session.ingest({id:3,value:'occupied',at:3000,score:500});
  session.ingest({id:3,value:'occupied',at:10000,score:500});
  session.ingest({id:1,value:'occupied',at:12000,score:500});
  assert.deepEqual(session.summary().map(({passes,misses,maxScore})=>({passes,misses,maxScore})),[{passes:1,misses:1,maxScore:500}]);
});
