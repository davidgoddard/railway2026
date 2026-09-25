const test=require('node:test');
const assert=require('node:assert/strict');
const CameraReadiness=require('./camera-readiness');

const saved={revision:7,cells:[{id:1}],settings:{}};
const online={online:true,remoteRevision:7,baseline:true};

test('guides a camera through the normal readiness lifecycle',()=>{
  assert.equal(CameraReadiness.derive({camera:{online:false}}).nextAction.id,'diagnostics');
  assert.equal(CameraReadiness.derive({camera:{online:true},hasFrame:false}).nextAction.id,'fetch-frame');
  assert.equal(CameraReadiness.derive({camera:{online:true},hasFrame:true,draft:{cells:[]},draftMatchesSaved:true}).nextAction.id,'add-sensors');
  assert.equal(CameraReadiness.derive({camera:online,hasFrame:true,savedConfig:saved,draft:{...saved,cells:[...saved.cells,{id:2}]},draftMatchesSaved:false}).nextAction.id,'save-deploy');
  assert.equal(CameraReadiness.derive({camera:{...online,remoteRevision:6},hasFrame:true,savedConfig:saved,draft:saved,draftMatchesSaved:true}).nextAction.id,'wait-deployment');
  assert.equal(CameraReadiness.derive({camera:{...online,baseline:false},hasFrame:true,savedConfig:saved,draft:saved,draftMatchesSaved:true}).nextAction.id,'calibrate');
  assert.equal(CameraReadiness.derive({camera:online,hasFrame:true,savedConfig:saved,draft:saved,draftMatchesSaved:true}).nextAction.id,'test-detector');
});

test('only declares a camera ready when its current revision has passed testing',()=>{
  const result=CameraReadiness.derive({camera:online,hasFrame:true,savedConfig:saved,draft:saved,draftMatchesSaved:true,test:{revision:7,suspects:0,intermittent:0}});
  assert.equal(result.ready,true);
  assert.equal(result.testStatus,'complete');
  assert.equal(result.nextAction.id,'ready');
  assert.equal(CameraReadiness.derive({...result,camera:online,hasFrame:true,savedConfig:saved,draft:saved,draftMatchesSaved:true,test:{revision:6,suspects:0,intermittent:0}}).testStatus,'untested');
});

test('reports faults and attention without hiding deployed state',()=>{
  const pending=CameraReadiness.derive({camera:{online:true,remoteRevision:6},hasFrame:true,savedConfig:saved,draft:saved,draftMatchesSaved:true});
  assert.deepEqual(pending.faults,['Deployment pending']);
  const attention=CameraReadiness.derive({camera:online,hasFrame:true,savedConfig:saved,draft:saved,draftMatchesSaved:true,test:{revision:7,suspects:1,intermittent:0}});
  assert.equal(attention.deployment,'current');
  assert.equal(attention.testStatus,'attention');
  assert.equal(attention.ready,false);
  assert.equal(attention.nextAction.id,'review-test');
});
