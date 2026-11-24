proposals = {'loose_tube': {'T_W_object':tube_pose.tolist()}, 'tube_rack': {'T_W_object':rack_pose.tolist()}}
result = world.integrate_rgb(obs['capture'], poses=proposals, expected_revision=world.snapshot().revision, note='Calibrated RGB: closed tube tip pixel (497,706), open rim (590,811), center backprojected at z=-0.0049. Rack fit to six observed hole centers with imported 25mm pitch; second from left local y=-0.0375. Dimensions and support height remain uncertain.')
ctx = world.read_context()
initial_tcp = np.array(robo.state()['right']['T_W_tcp'])
initial_jaw = robo.state()['right']['gripper_width_m']
grasp_point = tube_pose[:3,3] + .022*tube_axis
pick_rotation = np.column_stack([tube_axis, np.cross([0,0,-1],tube_axis),[0,0,-1]])
print('integrated revision',ctx['revision'],'grasp',grasp_point)
comparison = world.compare_views(obs['capture'])