ctx = world.read_context()
commit1 = world.commit_step(ctx['interaction']['pending_update']['capture'], expected_revision=ctx['revision'], note='Lemon visibly lifted with the fingers and remains held; plate stationary. Translation is estimated from actual TCP displacement and RGB; rotation is not resolved.', poses={'lemon': {'T_W_object': lemon_lifted.tolist(), 'translation_observed_axes': [True, True, True], 'rotation_observed_axes': [False, False, False]}}, held={'right': 'lemon'}, unresolved={'lemon': 'Exact orientation partly hidden by fingers; original rotation retained only as a prior.'})
ctx = world.read_context()
assert ctx['interaction']['pending_update'] is None
world.expect_contacts(grasp={'right': 'lemon'})
transfer_receipt = robo.move_to([0.373, 0.102, 0.160], rotation=start_tcp[:3, :3].tolist(), arm='right')
robo.show('top')
state2 = robo.state()
log({'tcp_xyz': find_tcp(state2)[:3, 3].tolist(), 'jaw_m': state2['right']['gripper_width_m'], 'receipt': transfer_receipt})