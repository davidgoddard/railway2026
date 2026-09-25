(function(root,factory){
  const api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  else root.CameraReadiness=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  function derive(input={}){
    const camera=input.camera||{},saved=input.savedConfig||null,draft=input.draft||null;
    const cells=draft?.cells||saved?.cells||[];
    const online=!!camera.online;
    const configured=cells.length>0;
    const draftChanges=!!draft&&(!saved||input.draftMatchesSaved===false);
    const deployed=configured&&!draftChanges&&!!saved?.revision&&camera.remoteRevision===saved.revision;
    const calibrated=deployed&&!!camera.baseline;
    const tested=calibrated&&!!input.test&&input.test.revision===saved.revision;
    const testStatus=!tested?'untested':input.test.suspects>0?'attention':input.test.intermittent>0?'attention':'complete';
    const faults=[];
    if(!online)faults.push('Camera offline');
    if(configured&&!draftChanges&&saved?.revision&&camera.remoteRevision!==saved.revision)faults.push('Deployment pending');
    if(deployed&&!camera.baseline)faults.push('Empty-track calibration required');
    let nextAction;
    if(!online)nextAction={id:'diagnostics',label:'System setup'};
    else if(input.hasFrame===false)nextAction={id:'fetch-frame',label:'Fetch camera image'};
    else if(!configured)nextAction={id:'add-sensors',label:'Add sensors'};
    else if(draftChanges)nextAction={id:'save-deploy',label:'Save & deploy'};
    else if(!deployed)nextAction={id:'wait-deployment',label:'Waiting for deployment'};
    else if(!calibrated)nextAction={id:'calibrate',label:'Calibrate empty track'};
    else if(!tested)nextAction={id:'test-detector',label:'Test detector'};
    else if(testStatus==='attention')nextAction={id:'review-test',label:'Review test results'};
    else nextAction={id:'ready',label:'Ready to monitor'};
    return{
      connection:online?'online':'offline',frame:input.hasFrame===true?'available':input.hasFrame===false?'missing':'unknown',
      configuration:!configured?'missing':draftChanges?'draft':'saved',
      deployment:deployed?'current':configured?'pending':'not-configured',
      calibration:calibrated?'current':'required',testStatus,faults,nextAction,
      ready:online&&deployed&&calibrated&&testStatus==='complete'
    };
  }
  return{derive};
});
