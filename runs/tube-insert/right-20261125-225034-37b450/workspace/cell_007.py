def pose(T,ta=[True,True,False],ra=[False,False,True]): return {'T_W_object':T.tolist(),'translation_observed_axes':ta,'orientation_observed_axes':ra}
print(str(world.integrate_rgb(obs['capture'],poses={tube:pose(TR),rack:pose(RR)},expected_revision=snap.revision,note='Calibrated top: back-projected tube rim and rounded end on table-supported center plane; fit rack center and yaw to six rim centers using original mesh and table height.'))[:450])
def tcp(): return np.array(robo.state()['right']['T_W_tcp'])
def stat(): print('TCP',np.round(tcp()[:3,3],4),'jaw',robo.state()['right']['gripper_width_m'])
def move(p,R=None,path='auto',intent=None):
 world.read_context()
 if intent: world.expect_contacts(**intent)
 try: print(str(robo.move_to(np.asarray(p).tolist(),None if R is None else R.tolist(),arm='right',path=path))[:650])
 except RuntimeError as err: print(str(err)[:1250])
 stat()
def commit(note,**kwargs):
 ctx=world.read_context(); pending=ctx['interaction']['pending_update']; print('pending',bool(pending));
 if pending: print(str(world.commit_step(pending['capture'],expected_revision=ctx['revision'],note=note,**kwargs))[:600])
def grip(w,intent=None):
 world.read_context()
 if intent: world.expect_contacts(**intent)
 try: print(str(robo.set_gripper(w,arm='right'))[:500])
 except RuntimeError as err: print(str(err)[:1100])
 stat()
Rg=np.column_stack([-axis,np.cross(-up,-axis),-up]); gp=world.target(lambda s: s.local_point(tube,[0,.031,0])); move(np.array(gp)+up*.080,Rg); crop((305,550,515,850))