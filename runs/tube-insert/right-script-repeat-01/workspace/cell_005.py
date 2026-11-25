tube='loose_tube'; rack='tube_rack'; up=np.array([0.,0.,1.]); uvh=np.array([(850,677),(905,668),(959,660),(1014,651),(1067,643),(1121,635)])
def pose(rot,pos):
 tf=np.eye(4); tf[:3,:3]=rot; tf[:3,3]=pos; return tf
def prop(tf,ori='xyz',trans='xyz'):
 return {'T_W_object':tf.tolist(),'translation_observed_axes':list(trans),'orientation_observed_axes':list(ori)}
local=np.array([-.0625,-.0375,-.0125,.0125,.0375,.0625]); zz=np.linspace(.085,.15,131); hr=[]
for ht in zz:
 pts=np.array([back(uv,ht) for uv in uvh]); yy=unit(local@pts); hr.append(np.sum((pts-pts.mean(0)-local[:,None]*yy)**2))
ht=zz[np.argmin(hr)]; pts=np.array([back(uv,ht) for uv in uvh]); ry=unit(local@pts); rt=pose(np.column_stack([np.cross(ry,up),ry,up]),pts.mean(0)-up*.139)
closed=back((397,677),-.0049); mouth=back((439,812),-.0049); axis=unit(mouth-closed); tx=np.cross(axis,up); tt=pose(np.column_stack([tx,axis,up]),(closed+mouth)/2)
sv=np.asarray(world.mesh(tube).vertices); lv=(sv-s.transform(tube)[:3,3])@s.transform(tube)[:3,:3]; print(ht,rt[:3,3],pts[1],tt[:3,3],axis,lv.min(0),lv.max(0)); print([(sign,np.unique(np.round(lv[(sign*lv[:,1]>.045),1],4))[:30], np.percentile(np.linalg.norm(lv[(sign*lv[:,1]>.045)][:,[0,2]],axis=1),[0,25,50,75,100])) for sign in [-1,1]])
print(str(world.integrate_rgb(obs['capture'],poses={tube:prop(tt,'z','xy'),rack:prop(rt,'z','xy')},expected_revision=s.revision,note='Calibrated initial top RGB: rounded tube end and open lip; rack yaw and center fit to six visible circles on existing mesh pitch. Heights inferred from table support and rack pitch, not independently measured.'))[:800]); ctx=world.read_context(); print(ctx['revision'])