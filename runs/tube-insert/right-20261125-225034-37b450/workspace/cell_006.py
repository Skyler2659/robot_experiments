def unit(p): return p/np.linalg.norm(p)
def mat(R,p):
 T=np.eye(4); T[:3,:3]=R; T[:3,3]=p; return T
def pose(T,ta=['x','y','z'],ra=['z']): return {'T_W_object':T.tolist(),'translation_observed_axes':ta,'orientation_observed_axes':ra}
up=np.array([0.,0.,1.]); rim=bp(454,806,-.0049); tip=bp(408,679,-.0049); axis=unit(rim-tip); TR=mat(np.column_stack([np.cross(axis,up),axis,up]),(rim+tip)/2-axis*.0032)
uvholes=[(823,642),(874,654),(928,665),(981,677),(1036,689),(1091,700)]; holes=np.array([bp(u,v,.1278) for u,v in uvholes]); along=unit(holes[-1]-holes[0]); RR=mat(np.column_stack([np.cross(along,up),along,up]),[*holes.mean(0)[:2],-.012]); print(TR[:3,3],holes[1]); print(str(world.integrate_rgb(obs['capture'],poses={tube:pose(TR),rack:pose(RR)},expected_revision=snap.revision,note='Calibrated top: back-projected visible tube rim and rounded end at mesh radius above table; fit rack yaw and center to six rim centers at table-anchored mesh plate height. Heights remain single-view estimates.'))[:1100])
def tcp(): return np.array(robo.state()['right']['T_W_tcp'])
def stat(): print('TCP',np.round(tcp()[:3,3],4),'jaw',robo.state()['right']['gripper_width_m'])
def move(p,R=None,path='auto',intent=None):
 world.read_context()
 if intent: world.expect_contacts(**intent)
 try: print(str(robo.move_to(p.tolist(),None if R is None else R.tolist(),arm='right',path=path))[:850])
 except RuntimeError as err: print(str(err)[:1250])
 stat()
def commit(note,**kwargs):
 ctx=world.read_context(); pending=ctx['interaction']['pending_update']; print('pending',bool(pending));
 if pending: print(str(world.commit_step(pending['capture'],expected_revision=ctx['revision'],note=note,**kwargs))[:550])
def grip(w,intent=None):
 world.read_context()
 if intent: world.expect_contacts(**intent)
 try: print(str(robo.set_gripper(w,arm='right'))[:650])
 except RuntimeError as err: print(str(err)[:1100])
 stat()
Rg=np.column_stack([-axis,np.cross(-up,-axis),-up]); gp=world.target(lambda s: s.local_point(tube,[0,.028,0])); move(np.array(gp)+up*.085,Rg)