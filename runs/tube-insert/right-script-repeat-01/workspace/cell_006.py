def prop(tf,ori='xyz',trans='xyz'):
 return {'T_W_object':tf.tolist(),'translation_observed_axes':[a in trans for a in 'xyz'],'orientation_observed_axes':[a in ori for a in 'xyz']}
ax=-axis; tt=pose(np.column_stack([np.cross(ax,up),ax,up]),mouth+.046*ax); print(str(world.integrate_rgb(obs['capture'],poses={tube:prop(tt,'z','xy'),rack:prop(rt,'z','xy')},expected_revision=world.snapshot().revision,note='Current calibrated top: tube mouth at (439,812), rounded end at (397,677); six rack holes fit to mesh pitch, feet at tabletop. Heights inferred from support and mesh.'))[:400]); print(world.snapshot().transform(rack)[:3,3]);
def tcp(): return np.array(robo.state()['right']['T_W_tcp'])
def review(note,**kw):
 c=world.read_context(); p=c['interaction']['pending_update'];
 if p: print('update:',str(world.commit_step(p['capture'],expected_revision=c['revision'],note=note,**kw))[:450]);
 return bool(p)
def mv(pos,rot,path='auto',grasp=False):
 world.read_context();
 if grasp: world.expect_contacts(grasp={'right':tube})
 try: print(str(robo.move_to(pos,rot,arm='right',path=path))[:1100]);
 except RuntimeError as e: print(str(e)[:1300]);
 print('TCP',tcp()[:3,3].round(4))
R=np.column_stack([axis,np.cross(-up,axis),-up]); grip=mouth+.014*ax; grip[2]=-.003; pre=grip+up*.065; mv(pre,R)