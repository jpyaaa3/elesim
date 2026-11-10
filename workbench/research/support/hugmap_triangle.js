/* C-band triangle selection: same raster objective and vertex-exchange search as
 * hug_c/triangle.py. Large-pool random starts use a portable seeded JS stream;
 * imported RL anchors preserve the exact Python-selected result. */
const HugTriangle = (() => {
  const cross=(a,b)=>a[0]*b[1]-a[1]*b[0], sub=(a,b)=>[a[0]-b[0],a[1]-b[1]];
  function raster(points,step) {
    const n=Math.round(180/step),grid=new Uint8Array(n*n),prefix=new Float64Array(n*(n+1));
    for(const p of points){const i=Math.floor((p[0]+180)/step),j=Math.floor((p[1]+180)/step);if(i<0||j<0||i>=n||j>=n)throw Error('q outside C grid');grid[j*n+i]=1;}
    for(let j=0;j<n;j++)for(let i=0;i<n;i++)prefix[j*(n+1)+i+1]=prefix[j*(n+1)+i]+grid[j*n+i];
    return vertices=>{
      const lo=Math.min(...vertices.map(v=>v[1])),hi=Math.max(...vertices.map(v=>v[1]));let sum=0;
      for(let k=Math.max(0,Math.ceil((lo+180)/step*4-.5));k<n*4;k++){
        const y=-180+(k+.5)*step/4;if(y>=hi)break;let left=Infinity,right=-Infinity;
        for(let e=0;e<3;e++){const a=vertices[e],b=vertices[(e+1)%3],dy=b[1]-a[1];if(Math.abs(dy)<1e-12)continue;const t=(y-a[1])/dy;if(t>=0&&t<1){const x=a[0]+t*(b[0]-a[0]);left=Math.min(left,x);right=Math.max(right,x);}}
        if(!Number.isFinite(left)||!Number.isFinite(right))continue;
        const row=Math.floor(k/4),integral=x=>{const u=Math.max(0,Math.min(n,(x+180)/step)),i=Math.min(Math.floor(u),n-1);return prefix[row*(n+1)+i]+(u-i)*grid[row*n+i];};
        sum+=Math.max(0,integral(right)-integral(left));
      }
      return sum*step*step/4;
    };
  }
  function random(seed){let t=seed>>>0;return ()=>{t+=0x6D2B79F5;let x=Math.imul(t^(t>>>15),1|t);x^=x+Math.imul(x^(x>>>7),61|x);return ((x^(x>>>14))>>>0)/4294967296;};}
  function sample(a,n,rng){const b=a.slice();for(let i=0;i<n;i++){const j=i+Math.floor(rng()*(b.length-i));[b[i],b[j]]=[b[j],b[i]];}return b.slice(0,n);}
  function compare(a,b){for(let i=0;i<a.length;i++){if(a[i]>b[i])return 1;if(a[i]<b[i])return -1;}return 0;}
  function select(rows,step,{seed=42,poolSize=64,restarts=12}={}){
    if(!(step>0&&Math.abs(180/step-Math.round(180/step))<1e-9))throw Error('Step must divide 180 degrees');
    if(rows.length<3)throw Error('Need at least three feasible C poses');
    if(rows.some(r=>r.q_deg.length!==2||r.q_deg.some(q=>!Number.isFinite(q)||q>=0||q< -180)||!Number.isFinite(r.mu_min)))throw Error('Invalid C candidates');
    if(rows.some(r=>'preparation_passed' in r)&&rows.some(r=>r.preparation_passed!==true||!Number.isFinite(r.preparation_margin_mm)||r.preparation_margin_mm<=0))throw Error('Preparation envelope must pass');
    const p=rows.map(r=>r.q_deg),covered=raster(p,step),keys=new Map();
    p.forEach((q,i)=>keys.set(q.map(x=>Math.floor((x+180)/step)).join(','),i));
    const boundary=[];
    for(const [key,i] of keys){const [x,y]=key.split(',').map(Number);if([[1,0],[-1,0],[0,1],[0,-1]].some(([dx,dy])=>!keys.has(`${x+dx},${y+dy}`)))boundary.push(i);}
    const rng=random(seed);let pool=boundary;
    if(boundary.length>poolSize){const extremes=[];for(let k=0;k<16;k++){const a=2*Math.PI*k/16,c=Math.cos(a),s=Math.sin(a);let best=0;for(let i=1;i<p.length;i++)if(p[i][0]*c+p[i][1]*s>p[best][0]*c+p[best][1]*s)best=i;extremes.push(best);}pool=[...new Set([...extremes,...sample(boundary,poolSize,rng)])].sort((a,b)=>a-b);}
    function score(ids){const v=ids.map(i=>p[i]),area=Math.abs(cross(sub(v[1],v[0]),sub(v[2],v[0])))/2;if(area<1e-8)return [-1,-Infinity,-Infinity,-Infinity];
      const perimeter2=v.reduce((s,q,i)=>s+(q[0]-v[(i+1)%3][0])**2+(q[1]-v[(i+1)%3][1])**2,0);
      return [Math.round(Math.min(area,covered(v))*1e9)/1e9,Math.min(...ids.map(i=>rows[i].preparation_margin_mm||0)),-Math.max(...ids.map(i=>rows[i].mu_min)),area/perimeter2];}
    let best=null,value=null;
    const consider=ids=>{const s=score(ids);if(value===null||compare(s,value)>0){best=ids.slice();value=s;}};
    const exchange=(start,candidates,passes)=>{let ids=start.slice(),s=score(ids);for(let pass=0;pass<passes;pass++){let changed=false;for(let axis=0;axis<3;axis++)for(const candidate of candidates){const next=ids.slice();next[axis]=candidate;const ns=score(next);if(compare(ns,s)>0){ids=next;s=ns;changed=true;}}if(!changed)break;}return ids;};
    if(rows.length<=24){for(let a=0;a<rows.length-2;a++)for(let b=a+1;b<rows.length-1;b++)for(let c=b+1;c<rows.length;c++)consider([a,b,c]);}
    else{const trials=Array.from({length:512},()=>{const ids=sample(pool,3,rng);return {ids,score:score(ids)};});trials.sort((a,b)=>compare(b.score,a.score));for(const t of trials.slice(0,restarts))consider(exchange(t.ids,pool,12));consider(exchange(best,boundary,5));}
    if(value[0]<0)throw Error('No nondegenerate C triangle');
    const anchors=best.map(i=>rows[i]),v=anchors.map(r=>r.q_deg),area=Math.abs(cross(sub(v[1],v[0]),sub(v[2],v[0])))/2;
    return {anchors,rows,selection:{method:rows.length<=24?'exhaustive-raster':'seeded-boundary-vertex-exchange',covered_area_deg2:value[0],solution_area_deg2:rows.length*step*step,coverage:value[0]/(rows.length*step*step),triangle_area_deg2:area,vertices_total_deg:v,resolution_total_deg:step,continuous_global_optimum_proven:false}};
  }
  function scan(kernel,D,step,mu){
    if(!(D>0&&Number.isFinite(D)&&Number.isFinite(mu)&&mu>=0&&step>=.5&&step<=10&&Math.abs(180/step-Math.round(180/step))<1e-9))throw Error('Invalid diameter, mu or grid step');
    const evaluator=new Function(kernel+`;return function(q3,q4,D,mu){
      const m=model(q3,q4,D,-90);let polys=null,bad=false,best=null;
      for(const c of m.candidates){if(!Number.isFinite(c.mu)||c.mu>mu||c.x>0||c.y<0||Math.hypot(c.x,c.y)>500)continue;
        if(!polys){polys=collisionGeometry(q3,q4);bad=selfCollision(polys);}if(bad||objectCollision(polys,c,D,-90))continue;
        const prep=preparationReport(c,D);if(prep.blocked)continue;
        if(!best||c.mu<best.mu_min)best={q_deg:[q3,q4],xy_mm:[c.x,c.y],mu_min:c.mu,feature:c.kind,root:c.branch,preparation_passed:true,preparation_margin_mm:prep.minimum_gap_mm-prep.required_clearance_mm};
      }return best;
    };`)();
    const rows=[];for(let i=0;i<Math.round(180/step);i++)for(let j=0;j<Math.round(180/step);j++){const r=evaluator(-180+(i+.5)*step,-180+(j+.5)*step,D,mu);if(r)rows.push(r);}return rows;
  }
  return {raster,select,scan};
})();
if(typeof module!=='undefined')module.exports=HugTriangle;
