for object_id in proposals:
    proposals[object_id]['translation_observed_axes'] = [True,True,True]
    proposals[object_id]['rotation_observed_axes'] = [True,True,True]
result = world.integrate_rgb(obs['capture'], poses=proposals, expected_revision=world.snapshot().revision, note='Tube endpoints and rack six holes observed in this episode RGB; estimates use calibrated rays, tabletop support prior, and imported hole spacing. Tube closed end toward image upper-left and mouth lower-right; rack left-to-right direction fitted from holes. Depth is model-assisted, not a measured depth image.')
ctx = world.read_context()
initial_tcp = np.array(robo.state()['right']['T_W_tcp'])
initial_jaw = robo.state()['right']['gripper_width_m']
grasp_point = tube_pose[:3,3] + .022*tube_axis
pick_rotation = np.column_stack([tube_axis, np.cross([0,0,-1],tube_axis),[0,0,-1]])
print('integrated revision',ctx['revision'],'grasp',grasp_point)
comparison = world.compare_views(obs['capture'])