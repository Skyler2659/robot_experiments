import * as THREE from 'three';
const P={center:[.297,.318,-.02],depth:.035,length:.179,height:.149,deckZ:.136,thickness:.003,holeRadius:.0105,pitch:.025};
export const objectParameters={tube_rack:{movable:true,dimensions_m:[.044,P.length,P.height],position_W_m:[P.center[0],P.center[1],P.center[2]+P.height/2],rotation_XYZ_rad:[0,0,-.018],unknowns:['All dimensions and placement estimated from the assigned RGB and fixed camera calibration.','Hidden back edges and lower support frame completed as symmetric acrylic construction.','Acrylic optical properties, thickness, and hole diameter are visual estimates.']}};
export function build(){
 const group=new THREE.Group();group.name='tube_rack';group.userData={object_id:'tube_rack',geometry_origin:'single_rgb_rcwm'};group.position.set(...P.center);group.rotation.z=-.018;
 const acrylic=new THREE.MeshPhysicalMaterial({color:0x7fafaa,roughness:.19,metalness:.04,transparent:true,opacity:.065,side:THREE.DoubleSide,depthWrite:false});
 const endMaterial=acrylic.clone();endMaterial.opacity=.46;endMaterial.color.setHex(0x9dcac5);
 const rimMaterial=new THREE.MeshStandardMaterial({color:0x9ec8c2,roughness:.24,metalness:.18,transparent:true,opacity:.55,depthWrite:false});
 const edgeMat=new THREE.MeshBasicMaterial({color:0xa0d1cb,transparent:true,opacity:.12,depthWrite:false});
 function add(geom,x=0,y=0,z=0){const mesh=new THREE.Mesh(geom,acrylic);mesh.position.set(x,y,z);group.add(mesh);const lines=new THREE.LineSegments(new THREE.EdgesGeometry(geom,28),edgeMat);lines.position.copy(mesh.position);group.add(lines);return mesh;}
 function box(x,y,z,sx,sy,sz){const m=add(new THREE.BoxGeometry(sx,sy,sz),x,y,z);const mat=acrylic.clone();mat.opacity=.23;m.material=mat;return m;}
 const d=P.depth,l=P.length,t=P.thickness;
 // Broad end walls continue to the table and rise above the perforated deck.
 for(const y of [-l/2+t/2,l/2-t/2]){const wall=box(0,y,P.height/2,.044,t,P.height);wall.material=endMaterial;}
 // Extruded top plate: actual through holes, no disks or texture substitutes.
 const shape=new THREE.Shape();shape.moveTo(-d/2,-l/2+t);shape.lineTo(d/2,-l/2+t);shape.lineTo(d/2,l/2-t);shape.lineTo(-d/2,l/2-t);shape.closePath();
 for(let i=0;i<6;i++){const hole=new THREE.Path();hole.absarc(0,(i-2.5)*P.pitch,P.holeRadius,0,Math.PI*2,true);shape.holes.push(hole);}
 const deck=add(new THREE.ExtrudeGeometry(shape,{depth:t,bevelEnabled:false,curveSegments:64}),0,0,P.deckZ);deck.material=[acrylic,rimMaterial];
 for(let i=0;i<6;i++)for(const z of [P.deckZ+t]){const rg=new THREE.TorusGeometry(P.holeRadius,.0006,8,64);const pos=rg.attributes.position;const colors=[];for(let j=0;j<pos.count;j++){const a=Math.atan2(pos.getY(j),pos.getX(j));const h=.16+.84*Math.pow(Math.abs(Math.cos(a-Math.PI/4)),6);colors.push(h,h,h);}rg.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));const rm=rimMaterial.clone();rm.vertexColors=true;rm.opacity=.85;const rim=new THREE.Mesh(rg,rm);rim.position.set(0,(i-2.5)*P.pitch,z);group.add(rim);}
 // A thin lower rectangular support frame leaves the large center open.
 const lower=.077,rail=.0035;
 for(const x of [-.0195+rail/2,.0195-rail/2])box(x,0,lower,rail,l-t*2,rail);
 for(const y of [-l/2+t*2,l/2-t*2])box(0,y,lower,.039-rail*2,rail,rail);
 // Molded deck lips and small attachment collars visible at the end plates.
 for(const x of [-d/2+t/2,d/2-t/2])box(x,0,P.deckZ+t*.65,t,l-t*2,t*1.3);
 for(const y of [-l/2+t,l/2-t])for(const x of [-d/2+t,d/2-t])box(x,y,lower+.003,.006,.006,.006);
 // Small round polished arrises catch the light around the structural plates.
 function polished(a,b,r=.00042){const av=new THREE.Vector3(...a),bv=new THREE.Vector3(...b),delta=bv.clone().sub(av);const m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,delta.length(),8),rimMaterial);m.position.copy(av.add(bv).multiplyScalar(.5));m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),delta.normalize());group.add(m);}
 for(const x of [-d/2,d/2]){for(const z of [P.deckZ,P.deckZ+t,lower])polished([x,-l/2+t,z],[x,l/2-t,z]);}
 for(const x of [-.022,.022])for(const y of [-l/2,l/2])polished([x,y,0],[x,y,P.height]);
 return group;
}
