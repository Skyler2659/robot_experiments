def pp(t,axes=[True,True,False],ori=[False,False,True]): return {'T_W_object':t.tolist(),'translation_observed_axes':axes,'orientation_observed_axes':ori}
res=world.integrate_rgb(obs['capture'],poses={tube:pp(TT),rack:pp(RT,[True]*3)},expected_revision=world.snapshot().revision,note='Calibrated top RGB: tube end centers on tabletop plus radius; fit rack row yaw, center and height to six visible rim centers using existing mesh pitch.'); print(str(res)[:600]); print(np.asarray(world.snapshot().transform(tube)))
def tcp(): return np.asarray(robo.state()['right']['T_W_tcp'])
def review(note,**kw):
 ctx=world.read_context(); p=ctx['interaction']['pending_update'];
 if p:
  ans=world.commit_step(p['capture'],expected_revision=ctx['revision'],note=note,**kw); print(str(ans)[:680])
def mv(p,R=None,path='auto',grasp=False):
 world.read_context();
 if grasp: world.expect_contacts(grasp={'right':tube})
 try: print(str(robo.move_to(np.asarray(p).tolist(),None if R is None else R.tolist(),arm='right',path=path))[:800]); print('TCP',tcp()[:3,3])
 except RuntimeError as e: print(str(e)[:1100])
def jaw(w,grasp=False):
 world.read_context();
 if grasp: world.expect_contacts(grasp={'right':tube})
 try: print(str(robo.set_gripper(w,arm='right'))[:500]); print('jaw',robo.state()['right']['gripper_width_m'])
 except RuntimeError as e: print(str(e)[:1000])
jaw(.042)