for proposal in proposals.values():
    proposal['orientation_observed_axes'] = [True,True,True]
result = world.integrate_rgb(obs['capture'], poses=proposals, expected_revision=world.snapshot().revision, note='Current calibrated RGB endpoints and six rack hole centers fitted to unchanged imported geometry; depth inferred using table support and known hole pitch. Tube mouth is image lower-right; rack hole two is second from image left.')
ctx = world.read_context()
initial_tcp = np.array(robo.state()['right']['T_W_tcp'])
initial_jaw = robo.state()['right']['gripper_width_m']
grasp_point = tube_pose[:3,3] + .022*tube_axis
pick_rotation = np.column_stack([tube_axis, np.cross([0,0,-1],tube_axis),[0,0,-1]])
print('integrated revision',ctx['revision'],'grasp',grasp_point)
comparison = world.compare_views(obs['capture'])