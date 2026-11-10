// Standalone overlay. Geometry work runs in a worker, away from canvas events.
(() => {
  const config=__TRIANGLE_CONFIG__,selectorSource=$('hugTriangleSource').textContent;
  let active=null,worker=null,serial=0;const cache=new Map();
  const controls={select:$('triangleSelect'),study:$('triangleStudy'),show:$('triangleShow'),step:$('triangleStep'),status:$('triangleStatus')};
  const key=()=>[ds[+$('diameter').value],muLimit(),Number(controls.step.value)].join('|');
  function status(text){controls.status.textContent=text;}
  function cancel(){serial++;if(worker){worker.terminate();status('Selection cancelled.');}worker=null;controls.select.disabled=false;}
  function overlay(){
    if(!active||!controls.show.checked)return;
    if(active.key!==key()){status('Parameters changed — select a new C triangle.');return;}
    if(+$('band').value!==-90){status('C triangle is available in Band 1 / roll −90°.');return;}
    const q=qMode(),project=r=>q?[QX(r.q_deg[0]),QY(r.q_deg[1])]:[X(r.xy_mm[0]),Y(r.xy_mm[1])];
    ctx.save();ctx.strokeStyle='#8b25b9';ctx.lineWidth=2.5;ctx.fillStyle='rgba(139,37,185,0.10)';
    for(const row of active.result.rows){const [x,y]=project(row);ctx.fillStyle='rgba(139,37,185,0.5)';ctx.fillRect(x-1,y-1,2,2);}
    const vertices=active.result.anchors.map(project);ctx.beginPath();vertices.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.closePath();
    if(q){ctx.fillStyle='rgba(139,37,185,0.10)';ctx.fill();}else ctx.setLineDash([6,4]);ctx.stroke();ctx.setLineDash([]);
    vertices.forEach(([x,y],i)=>{ctx.beginPath();ctx.arc(x,y,6,0,2*Math.PI);ctx.fillStyle='white';ctx.fill();ctx.stroke();ctx.fillStyle='#6b158c';ctx.font='bold 15px system-ui';ctx.fillText('ABC'[i],x+9,y-8);});ctx.restore();
    const s=active.result.selection;
    status(`${active.label} | ${s.feasible_samples??active.result.rows.length} candidates | covered ${(100*s.coverage).toFixed(1)}% of sampled solution area | ${s.covered_area_deg2.toFixed(2)} deg²\n`+
      active.result.anchors.map((r,i)=>`${'ABC'[i]}: q3 ${r.q_deg[0].toFixed(2)}°, q4 ${r.q_deg[1].toFixed(2)}° | x ${r.xy_mm[0].toFixed(2)}, y ${r.xy_mm[1].toFixed(2)} mm | μ ${r.mu_min.toFixed(3)}`).join('\n')+
      (q?'\nTriangle interiors may contain invalid holes.':'\nXY dashed lines join anchors; interpolation uses the q triangle.'));
  }
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
    cancel();stopConstruction();$('band').value='-90';const request=key();controls.show.checked=true;
    if(cache.has(request)){active=cache.get(request);draw();return;}
    active=null;draw();status('Selecting C triangle…');controls.select.disabled=true;
    const token=serial;
    const body=selectorSource+`\nself.onmessage=e=>{try{const {kernel,D,step,mu}=e.data,rows=HugTriangle.scan(kernel,D,step,mu);self.postMessage({result:HugTriangle.select(rows,step)});}catch(error){self.postMessage({error:error.message});}};`;
    const url=URL.createObjectURL(new Blob([body],{type:'text/javascript'}));worker=new Worker(url);URL.revokeObjectURL(url);
    worker.onmessage=e=>{if(token!==serial)return;worker.terminate();worker=null;controls.select.disabled=false;if(e.data.error){status(e.data.error);return;}
      active={key:request,label:'Function + CAD / preparation filters · physical admission not tested',result:e.data.result};cache.set(request,active);if(cache.size>16)cache.delete(cache.keys().next().value);draw();};
    worker.onerror=e=>{if(token!==serial)return;cancel();status('Triangle selection failed: '+e.message);};
    worker.postMessage({kernel:config.kernel,D:ds[+$('diameter').value],step:Number(controls.step.value),mu:muLimit()});
  };
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
