import * as THREE from 'three';
import {RoundedBoxGeometry} from '/.render-tools/node_modules/three/examples/jsm/geometries/RoundedBoxGeometry.js';
const specs={
  "rear_sensor_bracket": {
    "movable": false,
    "role": "fixed_obstacle",
    "dimensions_m": [
      0.08113814277901278,
      0.15509790255420364,
      0.1697329386560355
    ],
    "position_W_m": [
      0.055569072164366774,
      0.6937534972539875,
      0.08486646953663401
    ],
    "rotation_XYZ_rad": [
      0,
      0,
      0
    ],
    "unknowns": [
      "Estimated from assigned single RGB: metric depth, hidden faces, section thickness and exact attachments are not measured."
    ]
  },
  "rear_sensor_housing": {
    "movable": false,
    "role": "fixed_obstacle",
    "dimensions_m": [
      0.04972573661149472,
      0.105499998550862,
      0.04422755757110425
    ],
    "position_W_m": [
      0.01969951474095808,
      0.6949831000076008,
      0.16854277828063038
    ],
    "rotation_XYZ_rad": [
      0,
      0,
      0
    ],
    "unknowns": [
      "Estimated from assigned single RGB: metric depth, hidden faces, section thickness and exact attachments are not measured."
    ]
  },
  "rear_sensor_black_cable": {
    "movable": false,
    "role": "fixed_obstacle",
    "dimensions_m": [
      0.009758148342370987,
      0.033972978591918945,
      0.06661508232355118
    ],
    "position_W_m": [
      0.033840397372841835,
      0.6358560919761658,
      0.14936776086688042
    ],
    "rotation_XYZ_rad": [
      0,
      0,
      0
    ],
    "unknowns": [
      "Estimated from assigned single RGB: metric depth, hidden faces, section thickness and exact attachments are not measured."
    ]
  },
  "rear_sensor_pale_cable": {
    "movable": false,
    "role": "fixed_obstacle",
    "dimensions_m": [
      0.11780606955289841,
      0.15246444940567017,
      0.21502700448036194
    ],
    "position_W_m": [
      0.08714905753731728,
      0.5821728408336639,
      0.29379524290561676
    ],
    "rotation_XYZ_rad": [
      0,
      0,
      0
    ],
    "unknowns": [
      "Estimated from assigned single RGB: metric depth, hidden faces, section thickness and exact attachments are not measured."
    ]
  },
  "rear_sensor_thin_cable": {
    "movable": false,
    "role": "fixed_obstacle",
    "dimensions_m": [
      0.10512387752532959,
      0.057910919189453125,
      0.30769671499729156
    ],
    "position_W_m": [
      0.09057261049747467,
      0.5453959107398987,
      0.24680178612470627
    ],
    "rotation_XYZ_rad": [
      0,
      0,
      0
    ],
    "unknowns": [
      "Estimated from assigned single RGB: metric depth, hidden faces, section thickness and exact attachments are not measured."
    ]
  }
};
export const objectSpecs=specs;
const uncertain=['Estimated from assigned single RGB: metric depth, hidden faces, section thickness and exact attachments are not measured.'];
function root(id){const g=new THREE.Group();g.name=id;g.userData={object_id:id,geometry_origin:'single_rgb_rcwm'};specs[id]={movable:false,role:'fixed_obstacle',dimensions_m:[0,0,0],position_W_m:[0,0,0],rotation_XYZ_rad:[0,0,0],unknowns:[...uncertain]};return g;}
function finish(g){g.updateMatrixWorld(true);const b=new THREE.Box3().setFromObject(g),c=b.getCenter(new THREE.Vector3());specs[g.name].dimensions_m=b.getSize(new THREE.Vector3()).toArray();specs[g.name].position_W_m=c.toArray();for(const o of g.children)o.position.sub(c);g.position.copy(c);return g;}
const black=new THREE.MeshStandardMaterial({color:0x101d17,roughness:.48,metalness:.18});
const edges=new THREE.MeshStandardMaterial({color:0x17241e,roughness:.47,metalness:.52});
const steel=new THREE.MeshStandardMaterial({color:0x62776a,roughness:.4,metalness:.7});
const pale=new THREE.MeshStandardMaterial({color:0x95dfe6,roughness:.45,metalness:.08});
const cablemat=new THREE.MeshStandardMaterial({color:0x080f0c,roughness:.6});
const lightCable=new THREE.MeshStandardMaterial({color:0xb4efed,roughness:.82});
function mesh(g,geo,mat,p){const m=new THREE.Mesh(geo,mat);if(p)m.position.fromArray(p);m.castShadow=m.receiveShadow=true;g.add(m);return m;}
function box(g,d,p,mat=black,rad=0){return mesh(g,rad?new RoundedBoxGeometry(...d,3,rad):new THREE.BoxGeometry(...d),mat,p);}
const C=new THREE.Vector3(.6395681755265095,.3287295403275146,.6801332239685496);
const R=new THREE.Matrix3().set(.019972727841955695,.8513131122742009,-.5242776697634223,.9997954989714638,-.018669097755332705,.007773353549397498,-.003170233264806081,-.5243257095155789,-.8515118906756568);
// Assigned cropped camera ray, intersected with an explicitly estimated height.
function at(u,v,z){const d=new THREE.Vector3((u+1135.445068359375)/4070.3016357421875,(v-1661.6215209960938)/4061.8458251953125,1).applyMatrix3(R);return C.clone().addScaledVector(d,(z-C.z)/d.z);}
function bar(g,a,b,width,depth,mat=black){const m=mesh(g,new THREE.BoxGeometry(width,depth,a.distanceTo(b)),mat);m.position.copy(a).add(b).multiplyScalar(.5);m.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),b.clone().sub(a).normalize());return m;}
function tube(g,pts,r,mat){const c=new THREE.CatmullRomCurve3(pts.map(p=>at(...p)));return mesh(g,new THREE.TubeGeometry(c,96,r,12,false),mat);}
function fastener(g,p,r=.0028,normal=new THREE.Vector3(1,0,0)){const m=mesh(g,new THREE.CylinderGeometry(r,r,.002,16),steel,p.clone().addScaledVector(normal,.0006).toArray());m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),normal);const n=mesh(g,new THREE.CylinderGeometry(r*.38,r*.38,.0003,6),black,p.clone().addScaledVector(normal,.00165).toArray());n.quaternion.copy(m.quaternion);}
export function build(){const all=new THREE.Group();all.name='rear_sensor_mount';
const b=root('rear_sensor_bracket');
// Two finite rectangular cheeks, separated by an open slot, with lower feet.
const upper=at(772,622,.095),lower=at(852,1045,.014);
const cheek=bar(b,upper,lower,.025,.008);const pa=cheek.geometry.attributes.position;const len=upper.distanceTo(lower);for(let i=0;i<pa.count;i++){const f=(pa.getZ(i)/len+.5);pa.setX(i,pa.getX(i)*(1-.58*f));}pa.needsUpdate=true;cheek.geometry.computeVertexNormals();const upper2=at(620,698,.070),lower2=at(737,991,.014);bar(b,upper2,lower2,.010,.006);
box(b,[lower.x-.04+.022,.026,.0145],[(lower.x+.04)/2,lower.y,.00725],black,.0015);box(b,[lower2.x-.04+.022,.022,.0145],[(lower2.x+.04)/2,lower2.y,.00725],black,.0015);
// Broad transverse adjustment rail and a smaller parallel rear rail.
bar(b,at(205,656,.069),at(1000,510,.105),.012,.012);
bar(b,at(415,667,.079),at(894,634,.082),.009,.012,edges);
// Top cradle and short pedestal preserve the dark opening on the left.
bar(b,at(776,584,.105),at(774,391,.147),.025,.020);
bar(b,at(560,353,.157),at(1041,350,.159),.012,.021);
bar(b,at(482,440,.131),at(501,343,.157),.014,.012);
// Depth bridges join the front cheeks and pedestal to the rear transverse rail.
box(b,[.036,.022,.020],[.033,.720,.099]);
bar(b,upper2,new THREE.Vector3(.0233,.696,.0878),.011,.011);
b.updateMatrixWorld(true);const bracketSurfaces=[...b.children];
for(const [u,v,z,r] of [[755,598,.120,.0035],[766,631,.114,.0023],[589,415,.15,.0034],[855,423,.143,.0019],[923,423,.143,.0019],[449,696,.079,.0017],[526,696,.079,.0017],[606,696,.079,.0017],[655,695,.079,.0017],[668,759,.059,.0024],[706,764,.059,.0023]]){const ray=new THREE.Raycaster(C,at(u,v,z).sub(C).normalize());const hit=ray.intersectObjects(bracketSurfaces,false)[0];if(hit){const n=hit.face.normal.clone().applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(hit.object.matrixWorld)).normalize();fastener(b,hit.point,r,n);}}
all.add(finish(b));
const h=root('rear_sensor_housing');const hc=[.024,.697,.18];
box(h,[.043,.098,.025],hc,pale,.004);
// Dark shallow slots have rounded ends and recess into the front wall.
for(let i=0;i<23;i++){const y=.669+i*.0030;box(h,[.0006,.00125,.0057],[.04565,y,.188],new THREE.MeshStandardMaterial({color:0x213e35,roughness:.94}),.0005);}
box(h,[.030,.015,.022],[.024,.648,.174],pale,.002);
box(h,[.009,.007,.010],[.035,.645,.180],black,.002);
// Tilt the front face toward the observation, retaining a closed back and rounded sides.
const oldCenter=new THREE.Vector3(...hc),front=at(831,260,.18),q=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-.55);
const newCenter=front.clone().sub(new THREE.Vector3(.0215,0,0).applyQuaternion(q));
for(const m of h.children){m.position.sub(oldCenter).applyQuaternion(q).add(newCenter);m.quaternion.premultiply(q);}
all.add(finish(h));
const bc=root('rear_sensor_black_cable');tube(bc,[[524,239,.181],[454,282,.172],[379,370,.151],[328,460,.133],[328,502,.121],[394,495,.119],[476,456,.132],[483,365,.153],[501,278,.172]],.00225,cablemat);all.add(finish(bc));
const pc=root('rear_sensor_pale_cable');tube(pc,[[620,196,.19],[555,198,.19],[475,200,.19]],.0032,lightCable);tube(pc,[[622,196,.19],[500,198,.189],[355,169,.203],[211,112,.224],[180,64,.239],[188,-25,.262],[222,-250,.32],[252,-600,.40]],.00225,lightCable);all.add(finish(pc));const tc=root('rear_sensor_thin_cable');tube(tc,[[325,-610,.40],[292,-260,.32],[267,-35,.269],[218,72,.235],[158,235,.193],[91,451,.139],[23,628,.093]],.0011,lightCable);all.add(finish(tc));
return all;}
