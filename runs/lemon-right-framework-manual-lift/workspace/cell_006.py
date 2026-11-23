ctx = world.read_context()
commit1 = world.commit_step(ctx['interaction']['pending_update']['capture'], expected_revision=ctx['revision'], note='Top RGB confirms lemon lifted together with fingers and remains held; plate stationary. Translation estimated from actual TCP lift and RGB; orientation unresolved.', poses={'lemon': {'T_W_object': lemon_lifted.tolist(), 'translation_observed_axes': [True, True, True], 'orientation_observed_axes': [False, False, False]}}, held={'right': 'lemon'}, unresolved={'lemon': 'Exact orientation partly hidden; original rotation is only a prior.'})
ctx = world.read_context()
assert ctx['interaction']['pending_update'] is None
world.expect_contacts(grasp={'right': 'lemon'})
transfer_receipt = robo.move_to([0.373, 0.102, 0.160], rotation=start_tcp[:3, :3].tolist(), arm='right')
robo.show('top')
state2 = robo.state()
log({'tcp_xyz': find_tcp(state2)[:3, 3].tolist(), 'jaw_m': state2['right']['gripper_width_m'], 'receipt': transfer_receipt})