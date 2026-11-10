// BEGIN GENERATED PREPARATION COLLISION
// Built from the URDF and actual presets. Coordinates are housing-centred map mm.
const PREPARATION_COLLISION=__PREPARATION_DATA__;
let preparationBounds=null;
function preparationCollision(c,D){
 const R=D/2,margin=R+PREPARATION_COLLISION.clearance_mm+PREPARATION_COLLISION.sampling_padding_mm;
 if(!preparationBounds){
  preparationBounds=[Infinity,Infinity,-Infinity,-Infinity];
  for(const part of PREPARATION_COLLISION.parts){const b=polygonBounds(part.poly);preparationBounds[0]=Math.min(preparationBounds[0],b[0]);preparationBounds[1]=Math.min(preparationBounds[1],b[1]);preparationBounds[2]=Math.max(preparationBounds[2],b[2]);preparationBounds[3]=Math.max(preparationBounds[3],b[3]);}
 }
 const b=preparationBounds;
 if(c.x<b[0]-margin||c.x>b[2]+margin||c.y<b[1]-margin||c.y>b[3]+margin)return false;
 for(const part of PREPARATION_COLLISION.parts){
  const p=part.poly,bounds=polygonBounds(p);
  if(c.x<bounds[0]-margin||c.x>bounds[2]+margin||c.y<bounds[1]-margin||c.y>bounds[3]+margin)continue;
  if(diskGap(p,[c.x,c.y],R)<=margin-R)return true;
 }
 return false;
}
function preparationReport(c,D){
 let closest=null,phase='Home → Extend Arm envelope',minimum=Infinity;
 const threshold=PREPARATION_COLLISION.clearance_mm+PREPARATION_COLLISION.sampling_padding_mm;
 for(const part of PREPARATION_COLLISION.parts){
  const gap=diskGap(part.poly,[c.x,c.y],D/2);
  if(gap<minimum){minimum=gap;closest=part;}
 }
 if(closest){
  if(diskGap(closest.home||closest.poly,[c.x,c.y],D/2)<=threshold)phase='Home';
  else if(diskGap(closest.extend||closest.poly,[c.x,c.y],D/2)<=threshold)phase='Extend Arm';
 }
 return {blocked:minimum<=threshold,phase,part:closest?closest.name:null,minimum_gap_mm:minimum,required_clearance_mm:threshold};
}
// All callers (scan, inverse gap repair and exported RL kernel) use this gate.
function objectCollision(polys,c,D,roll){
 return finalObjectCollision(polys,c,D,roll)||preparationCollision(c,D);
}
function collisionLabel(c){
 const status=[];
 if(c.finalCollision)status.push('FINAL POSE COLLISION / clearance violation');
 if(c.preparation&&c.preparation.blocked)status.push(`${c.preparation.phase}: ${c.preparation.part}, gap ${c.preparation.minimum_gap_mm.toFixed(2)} mm (conservative envelope)`);
 return status.length?status.join('; '):'final projection + Home → Extend Arm envelope clear; roll/closing/lift untested';
}
// END GENERATED PREPARATION COLLISION
