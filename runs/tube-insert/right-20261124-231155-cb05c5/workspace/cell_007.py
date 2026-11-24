tube_pose[:3,1] = -tube_axis
tube_pose[:3,0] = np.cross(-tube_axis,[0,0,1])
proposals = {'loose_tube': tube_pose.tolist(), 'tube_rack': rack_pose.tolist()}
result = world.integrate_rgb(obs['capture'], poses=proposals, expected_revision=world.snapshot().revision, note='Current calibrated RGB: tube rounded closed end at (col497,row706), open rim at (590,811); backprojected at table plus 7.1mm radius. Rack six visible hole centers fitted with imported 25mm spacing, yaw 0.401rad; left-to-right hole 2 is local y=-0.0375. Dimensions remain prior estimates; top height and support geometry uncertain.')
ctx = world.read_context()
initial_tcp = np.array(robo.state()['right']['T_W_tcp'])
initial_jaw = robo.state()['right']['gripper_width_m']
grasp_point = tube_pose[:3,3] + .022*tube_axis
pick_rotation = np.column_stack([tube_axis, np.cross([0,0,-1],tube_axis),[0,0,-1]])
print('poses integrated',ctx['revision'],'grasp',grasp_point,'rotation',pick_rotation.tolist())
comparison = world.compare_views(obs['capture'])