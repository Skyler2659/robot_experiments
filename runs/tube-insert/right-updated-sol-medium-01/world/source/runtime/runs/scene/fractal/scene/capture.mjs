import fs from 'node:fs';
import {openViewer} from '../../runner_source/tools/lib/viewer_harness.mjs';
const root=process.cwd();const v=await openViewer(root,'runs/scene/scene/index.html',{width:1920,height:1080});await v.page.evaluate(async()=>await window.ready);console.log(v.errors);
const out='runs/scene/fractal/scene/review';
await v.page.screenshot({path:out+'/whole.png'});
fs.writeFileSync(out+'/object_parameters.json',JSON.stringify(await v.page.evaluate(()=>({objects:window.objectParameters,robotBounds:window.robotBounds})),null,2));
for(let i=0;i<4;i++){const png=await v.page.evaluate(async i=>{const T=await import('three');const c=new T.PerspectiveCamera(45,16/9,.01,20);c.up.set(0,0,1);const a=i*Math.PI/2;c.position.set(.25+1.5*Math.cos(a),.35+1.5*Math.sin(a),.85);c.lookAt(.25,.35,0);window.renderer.render(window.scene,c);return window.renderer.domElement.toDataURL().split(',')[1];},i);fs.writeFileSync(out+'/compass_'+i+'.png',Buffer.from(png,'base64'));}await v.close();
