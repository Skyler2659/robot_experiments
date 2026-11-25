from PIL import Image
def crop(box): display(Image.fromarray(robo.image('top')).crop(box))
def localmesh(key):
 T=np.array(snap.transform(key)); return (world.mesh(key).vertices-T[:3,3])@T[:3,:3]
vt=localmesh(tube); vr=localmesh(rack); print('tube quant',np.round(np.quantile(vt[:,0],[0,.1,.25,.5,.75,.9,1]),4)); print('rack z',np.unique(np.round(vr[:,2],3),return_counts=True)); print('vertical fit',[(z,bp(750,656,z)[:2],bp(1127,735,z)[:2]) for z in [.05,.07,.09,.11]]); crop((730,565,1170,880))