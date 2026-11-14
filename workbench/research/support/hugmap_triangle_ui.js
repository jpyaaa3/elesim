// Standalone overlay. Geometry work runs in a worker, away from canvas events.
(() => {
  const config=__TRIANGLE_CONFIG__,selectorSource=$('hugTriangleSource').textContent;
  let active=null,worker=null,projectionWorker=null,projecting=null,serial=0;const cache=new Map();
  const controls={select:$('triangleSelect'),study:$('triangleStudy'),show:$('triangleShow'),step:$('triangleStep'),status:$('triangleStatus')};
  const key=()=>[ds[+$('diameter').value],muLimit(),Number(controls.step.value)].join('|');
  function status(text){controls.status.textContent=text;}
  function cancel(){serial++;if(worker){worker.terminate();status('Selection cancelled.');}worker=null;
    if(projectionWorker)projectionWorker.terminate();projectionWorker=null;projecting=null;controls.select.disabled=false;controls.select.textContent='Select C triangle';if(!running)$('stop').disabled=true;}
  function geometryWorker(){
    const body=selectorSource+`\nself.onmessage=e=>{try{const {kernel,D,step,mu,anchors}=e.data;
      if(anchors){const projection=HugTriangle.project(kernel,D,mu,anchors);self.postMessage({projection},[projection.triangles_mm.buffer,projection.boundary_mm.buffer]);}
      else{const rows=HugTriangle.scan(kernel,D,step,mu);self.postMessage({result:HugTriangle.select(rows,step)});}
    }catch(error){self.postMessage({error:error.message});}};`;
    const url=URL.createObjectURL(new Blob([body],{type:'text/javascript'})),out=(()=>{try{return new Worker(url);}finally{URL.revokeObjectURL(url);}})();return out;
  }
  function requestProjection(selected){
    if(selected.projection||selected.projectionError||projecting===selected)return;
    if(projectionWorker)projectionWorker.terminate();
    const token=serial;projecting=selected;
    try{projectionWorker=geometryWorker();
    projectionWorker.onmessage=e=>{
      if(token!==serial||selected!==active||selected.key!==key())return;
      projectionWorker.terminate();projectionWorker=null;projecting=null;
      if(e.data.error)selected.projectionError=e.data.error;else selected.projection=e.data.projection;draw();
    };
    projectionWorker.onerror=e=>{if(token!==serial||selected!==active)return;projectionWorker.terminate();projectionWorker=null;projecting=null;selected.projectionError=e.message;draw();};
    projectionWorker.onmessageerror=()=>projectionWorker.onerror({message:'Unable to decode projection result'});
    projectionWorker.postMessage({kernel:config.kernel,D:ds[+$('diameter').value],mu:muLimit(),anchors:selected.result.anchors});
    }catch(error){if(projectionWorker)projectionWorker.terminate();projectionWorker=null;projecting=null;selected.projectionError=error.message;}
  }
  function drawProjection(selected){
    requestProjection(selected);if(!selected.projection)return;
    const {triangles_mm:triangles,boundary_mm:boundary}=selected.projection;
    if(!selected.xyCanvas){
      const layer=document.createElement('canvas');layer.width=canvas.width;layer.height=canvas.height;
      const paint=layer.getContext('2d');paint.fillStyle='#8b25b9';paint.beginPath();
      for(let i=0;i<triangles.length;i+=6){
        const p=[[X(triangles[i]),Y(triangles[i+1])],[X(triangles[i+2]),Y(triangles[i+3])],[X(triangles[i+4]),Y(triangles[i+5])]];
        // Consistent winding fills the union even when contact projections
        // fold or overlap. Missing branches remain holes in the sampled mesh.
        if((p[1][0]-p[0][0])*(p[2][1]-p[0][1])-(p[1][1]-p[0][1])*(p[2][0]-p[0][0])<0)[p[1],p[2]]=[p[2],p[1]];
        paint.moveTo(...p[0]);paint.lineTo(...p[1]);paint.lineTo(...p[2]);paint.closePath();
      }paint.fill();selected.xyCanvas=layer;
    }
    ctx.save();ctx.globalAlpha=.12;ctx.drawImage(selected.xyCanvas,0,0);ctx.restore();ctx.beginPath();
    for(let i=0;i<boundary.length;i+=4){ctx.moveTo(X(boundary[i]),Y(boundary[i+1]));ctx.lineTo(X(boundary[i+2]),Y(boundary[i+3]));}ctx.stroke();
  }
  function overlay(){
    if(!active||!controls.show.checked)return;
    if(active.key!==key()){status('Parameters changed — select a new C triangle.');return;}
    if(+$('band').value!==-90){status('C triangle is available in Band 1 / roll −90°.');return;}
    const q=qMode(),project=r=>q?[QX(r.q_deg[0]),QY(r.q_deg[1])]:[X(r.xy_mm[0]),Y(r.xy_mm[1])];
    ctx.save();ctx.strokeStyle='#8b25b9';ctx.lineWidth=2.5;ctx.fillStyle='rgba(139,37,185,0.10)';
    for(const row of active.result.rows){const [x,y]=project(row);ctx.fillStyle='rgba(139,37,185,0.5)';ctx.fillRect(x-1,y-1,2,2);}
    const vertices=active.result.anchors.map(project);
    if(q){ctx.beginPath();vertices.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.closePath();ctx.fillStyle='rgba(139,37,185,0.10)';ctx.fill();ctx.stroke();}
    else drawProjection(active);
    vertices.forEach(([x,y])=>{ctx.beginPath();ctx.arc(x,y,6,0,2*Math.PI);ctx.fillStyle='white';ctx.fill();ctx.stroke();});ctx.restore();
    const s=active.result.selection;
    status(`${active.label} | ${s.feasible_samples??active.result.rows.length} candidates | covered ${(100*s.coverage).toFixed(1)}% of sampled solution area | ${s.covered_area_deg2.toFixed(2)} deg²\n`+
      active.result.anchors.map(r=>`q3 ${r.q_deg[0].toFixed(2)}°, q4 ${r.q_deg[1].toFixed(2)}° | x ${r.xy_mm[0].toFixed(2)}, y ${r.xy_mm[1].toFixed(2)} mm | μ ${r.mu_min.toFixed(3)}`).join('\n')+
      (q?'\nTriangle interiors may contain invalid holes.':active.projectionError?`\nXY mapping failed: ${active.projectionError}`:active.projection?
        '\nXY maps the q-triangle through the contact model (≤0.5° mesh); invalid cells and contact branches stay separate.':'\nMapping the q-triangle to XY…'));
  }
  const baseTick=animationTick;animationTick=function(now){try{baseTick(now);}catch(error){stopConstruction();$('animationStatus').textContent='Construction failed: '+error.message;}};
  const baseDraw=draw;draw=function(){baseDraw();overlay();};
  for(const id of ['plane','overlay','preparation'])for(const event of ['oninput','onchange'])if($(id)[event]===baseDraw)$(id)[event]=draw;
  controls.show.checked=true;controls.show.oninput=()=>draw();
  function setStudy(){
    if(!config.study)return;cancel();const study=config.study;
    if(!ds.includes(study.diameter_mm)){ds.push(study.diameter_mm);ds.sort((a,b)=>a-b);$('diameter').max=ds.length-1;}
    $('diameter').value=ds.indexOf(study.diameter_mm);$('mu').value=study.mu_max;controls.step.value=study.step_deg;
    $('band').value='-90';$('plane').value='q';controls.show.checked=true;
    active={key:key(),label:`RL study · D=${study.diameter_mm} mm · physical admission screened`,result:study.result};
    $('q3').value=study.result.anchors[0].q_deg[0];$('q4').value=study.result.anchors[0].q_deg[1];clearConstruction();
  }
  controls.study.disabled=!config.study;
  controls.study.onclick=setStudy;
  controls.select.onclick=()=>{
    if(worker){cancel();return;}
    cancel();stopConstruction();$('band').value='-90';const request=key();controls.show.checked=true;
    if(cache.has(request)){active=cache.get(request);draw();return;}
    active=null;draw();status('Selecting C triangle…');controls.select.disabled=false;controls.select.textContent='Cancel selection';$('stop').disabled=false;
    const token=serial;
    try{worker=geometryWorker();
    worker.onmessage=e=>{if(token!==serial)return;const result=e.data;cancel();if(result.error){status(result.error);return;}
      active={key:request,label:'Function + CAD / preparation filters · physical admission not tested',result:result.result};cache.set(request,active);if(cache.size>16)cache.delete(cache.keys().next().value);draw();};
    worker.onerror=e=>{if(token!==serial)return;cancel();status('Triangle selection failed: '+e.message);};
    worker.onmessageerror=()=>worker.onerror({message:'Unable to decode selection result'});
    worker.postMessage({kernel:config.kernel,D:ds[+$('diameter').value],step:Number(controls.step.value),mu:muLimit()});
    }catch(error){cancel();status('Triangle selection failed: '+error.message);}
  };
  for(const id of ['stop','clear','construct']){const original=$(id).onclick;$(id).onclick=function(e){cancel();if(original)original.call(this,e);};}
  for(const id of ['diameter','mu']){const original=$(id).oninput;$(id).oninput=function(e){cancel();if(original)original.call(this,e);draw();};}
  controls.step.oninput=()=>{cancel();draw();};
  const click=canvas.onclick;
  canvas.onclick=e=>{
    if(active&&active.key===key()&&controls.show.checked&&+$('band').value===-90){
      const box=canvas.getBoundingClientRect(),px=(e.clientX-box.left)*canvas.width/box.width,py=(e.clientY-box.top)*canvas.height/box.height;
      for(const r of active.result.anchors){const x=qMode()?QX(r.q_deg[0]):X(r.xy_mm[0]),y=qMode()?QY(r.q_deg[1]):Y(r.xy_mm[1]);
        if((px-x)**2+(py-y)**2<144){stopConstruction();$('q3').value=r.q_deg[0];$('q4').value=r.q_deg[1];draw();return;}}
    }if(click)click(e);
  };
  if(config.study)setStudy();else draw();
})();
