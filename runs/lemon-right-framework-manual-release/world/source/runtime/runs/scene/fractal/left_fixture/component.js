import * as THREE from 'three';
export const objectSpecs={
 left_mounting_plate:{movable:false,role:'fixed_obstacle',dimensions_m:[.111,.247,.007],position_W_m:[-.0115,.004,.0035],rotation_XYZ_rad:[0,0,-.019],unknowns:['Single RGB estimate; rear edge is occluded by public robot. Thickness estimated at 7 mm; central clearance surrounds the unchanged public base footprint so the fixture does not bury its flange. Clearance/pocket construction is occluded and estimated; tabletop datum is inherited.']},
 left_checker_card:{movable:false,role:'fixed_obstacle',dimensions_m:[.054,.065,.001],position_W_m:[.008,-.075,.0074],rotation_XYZ_rad:[0,0,-.025],unknowns:['Card dimensions, backing and attachment inferred from visible checker corners; 5 columns by 4 rows estimated.']},
 left_support_bracket:{movable:false,role:'fixed_obstacle',dimensions_m:[.076,.0795,.0635],position_W_m:[-.015,.098,.007],rotation_XYZ_rad:[0,0,0],unknowns:['Rounded upright/gusset silhouette and foot inferred from crop; hidden rear profile, thickness, fasteners and depth are estimates. Independent of supplied robot base flange.']}
};
function root(id){const g=new THREE.Group();g.name=id;g.userData={object_id:id,geometry_origin:'single_rgb_rcwm'};g.position.fromArray(objectSpecs[id].position_W_m);g.rotation.set(...objectSpecs[id].rotation_XYZ_rad);return g;}
function mesh(geo,mat,parent,x=0,y=0,z=0){const m=new THREE.Mesh(geo,mat);m.position.set(x,y,z);m.castShadow=m.receiveShadow=true;parent.add(m);return m;}
function slab(x,y,z,bevel=.002,hole=null){const s=new THREE.Shape();const a=x/2,b=y/2,r=bevel;s.moveTo(-a+r,-b);s.lineTo(a-r,-b);s.lineTo(a,-b+r);s.lineTo(a,b-r);s.lineTo(a-r,b);s.lineTo(-a+r,b);s.lineTo(-a,b-r);s.lineTo(-a,-b+r);s.closePath();if(hole){const h=new THREE.Path();hole.forEach(([x,y],i)=>i?h.lineTo(x,y):h.moveTo(x,y));h.closePath();s.holes.push(h);}const g=new THREE.ExtrudeGeometry(s,{depth:z,bevelEnabled:false});g.translate(0,0,-z/2);return g;}
export function build(){const assembly=new THREE.Group();assembly.name='left_fixture';
 const black=new THREE.MeshStandardMaterial({color:0x0e1b19,roughness:.83,metalness:.08});
 const edge=new THREE.MeshStandardMaterial({color:0x1c2822,roughness:.52,metalness:.55});
 const bolt=new THREE.MeshStandardMaterial({color:0x3b463d,roughness:.46,metalness:.7});
 const p=root('left_mounting_plate');const hole=[[-.056,-.041],[.041,-.041],[.041,.041],[-.056,.041]].map(([x,y])=>{x+=.0115;y-=.004;return [Math.cos(.019)*x-Math.sin(.019)*y,Math.sin(.019)*x+Math.cos(.019)*y];});mesh(slab(.111,.247,.007,.003,hole),black,p);assembly.add(p);
 const c=root('left_checker_card');const pale=new THREE.MeshStandardMaterial({color:0xafdce8,roughness:.88});const ink=new THREE.MeshStandardMaterial({color:0x152722,roughness:.87});mesh(slab(.054,.065,.0008,.0014),pale,c);
 // Grid rows run along W x; columns along W y, as seen by the calibrated camera.
 const dx=.052/4,dy=.063/5;
 for(let row=0;row<4;row++)for(let col=0;col<5;col++)if((row+col)%2===1)mesh(new THREE.BoxGeometry(dx,dy,.00006),ink,c,-.026+(row+.5)*dx,-.0315+(col+.5)*dy,.00043);
 assembly.add(c);
 const b=root('left_support_bracket');const foot=new THREE.Shape();foot.moveTo(-.038,-.024);foot.lineTo(.035,-.058);foot.lineTo(.038,-.055);foot.lineTo(.038,.005);foot.lineTo(-.035,.021);foot.lineTo(-.038,.018);foot.closePath();const fg=new THREE.ExtrudeGeometry(foot,{depth:.005,bevelEnabled:false});mesh(fg,black,b);
 // Transverse rounded lug: closed six-millimeter plate in the YZ plane.
 const s=new THREE.Shape();s.moveTo(-.0075,.005);s.lineTo(-.0075,.020);s.lineTo(-.003,.050);s.quadraticCurveTo(0,.063,.0075,.063);s.quadraticCurveTo(.014,.063,.0175,.048);s.lineTo(.021,.020);s.lineTo(.021,.005);s.closePath();
 const gg=new THREE.ExtrudeGeometry(s,{depth:.006,bevelEnabled:true,bevelSize:.0005,bevelThickness:.0005,bevelSegments:2,steps:1});
 // Shape's u,v become W y,z, extrusion becomes W x.
 gg.applyMatrix4(new THREE.Matrix4().set(0,0,1,0, 1,0,0,0, 0,1,0,0, 0,0,0,1));mesh(gg,black,b,-.018,0,0);
 // A real gusset joins the lug to its foot without filling the surrounding space.
 const tri=new THREE.Shape();tri.moveTo(-.015,.005);tri.lineTo(-.015,.036);tri.lineTo(.030,.005);tri.closePath();
 const tg=new THREE.ExtrudeGeometry(tri,{depth:.004,bevelEnabled:false});tg.rotateX(Math.PI/2);mesh(tg,black,b,0,-.001,.0002);
 for(const [x,y] of [[-.029,-.013],[-.029,.013],[.029,-.046],[.029,-.002]]){const bg=new THREE.CylinderGeometry(.0022,.0022,.001,12);bg.rotateX(Math.PI/2);mesh(bg,bolt,b,x,y,.0055);}
 mesh(new THREE.BoxGeometry(.0012,.060,.001),edge,b,.037,-.026,.0045);
 assembly.add(b);return assembly;
}
