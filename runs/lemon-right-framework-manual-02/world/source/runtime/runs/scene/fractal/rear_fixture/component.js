import * as THREE from 'three';
import {build as buildSensorMount,objectSpecs as sensorSpecs} from '../rear_sensor_mount/component.js';
export const objectSpecs={
 ...sensorSpecs,
 rear_center_post:{movable:false,role:'fixed_obstacle',dimensions_m:[.030,.030,.46],position_W_m:[-.024,.334,0],rotation_XYZ_rad:[0,0,0],unknowns:['Post section, foot attachment and upper continuation inferred from RGB; upper endpoint outside image.']},
 rear_mount_base:{movable:false,role:'fixed_obstacle',dimensions_m:[.095,.24,.0152],position_W_m:[.0125,.681,.007],rotation_XYZ_rad:[0,0,0],unknowns:['Black base perimeter and thickness estimated; underside not observed.']},
 rear_checker_card:{movable:false,role:'fixed_obstacle',dimensions_m:[.056,.062,.0015],position_W_m:[.025,.596,.015],rotation_XYZ_rad:[0,0,0],unknowns:['Checker card size and grid inferred from crop; flat on base assumed.']}
};
const black=new THREE.MeshStandardMaterial({color:0x090e0c,roughness:.57,metalness:.52});
const edge=new THREE.MeshStandardMaterial({color:0x111a17,roughness:.55,metalness:.6});
const bolt=new THREE.MeshStandardMaterial({color:0x58655c,roughness:.35,metalness:.8});
function root(id){const g=new THREE.Group();g.name=id;g.userData={object_id:id,geometry_origin:'single_rgb_rcwm'};g.position.fromArray(objectSpecs[id].position_W_m);g.rotation.fromArray(objectSpecs[id].rotation_XYZ_rad);return g;}
function box(g,size,pos,mat=black){const m=new THREE.Mesh(new THREE.BoxGeometry(...size),mat);m.position.fromArray(pos);m.castShadow=m.receiveShadow=true;g.add(m);return m;}
function cyl(g,r,h,pos,mat=black){const m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,h,32),mat);m.rotation.x=Math.PI/2;m.position.fromArray(pos);m.castShadow=true;g.add(m);return m;}
function checker(){const c=document.createElement('canvas');c.width=600;c.height=500;const x=c.getContext('2d');x.fillStyle='#9cbdb4';x.fillRect(0,0,600,500);for(let j=0;j<4;j++)for(let i=0;i<5;i++){x.fillStyle=(i+j)%2?'#071411':'#9cbdb4';x.fillRect(22+i*111,22+j*113,111,113);}const t=new THREE.CanvasTexture(c);t.colorSpace=THREE.SRGBColorSpace;return t;}
export function build(){const all=new THREE.Group();all.name='rear_fixture';
const post=root('rear_center_post');cyl(post,.015,.008,[0,0,.004]);cyl(post,.012,.452,[0,0,.234]);cyl(post,.013,.007,[0,0,.012],edge);cyl(post,.0028,.001,[.012,0,.009],bolt);all.add(post);
const base=root('rear_mount_base');box(base,[.095,.24,.014],[0,0,0]);for(const x of [-.035,.035])for(const y of [-.100,.100])cyl(base,.0035,.0015,[x,y,.0077],bolt);all.add(base);
const card=root('rear_checker_card');box(card,[.056,.062,.0015],[0,0,0],new THREE.MeshStandardMaterial({color:0x91aaa1,roughness:.95}));const cg=new THREE.PlaneGeometry(.062,.056);const cm=new THREE.Mesh(cg,new THREE.MeshBasicMaterial({map:checker()}));cm.rotation.z=Math.PI/2;cm.position.z=.0008;card.add(cm);all.add(card);
all.add(buildSensorMount());Object.assign(objectSpecs,sensorSpecs);return all;}
