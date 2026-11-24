tube_pose = np.array(robo.state()['right']['T_W_tcp']) @ attachment_local
print('projected ends',project((np.column_stack([vertical_endpoints,np.ones(2)]) @ tube_pose.T)[:,:3]))
review('Tube visibly aligned over second hole from image left, mouth up. Bottom near pixel (994,827), rim near (998,710); rack has not shifted. Grip and expected motion remain consistent; depth estimate remains approximate.',poses={'loose_tube':pose_record(tube_pose)},held={'right':'loose_tube'})
ctx = world.read_context()
next_tcp = np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy()
next_tcp[2] -= .015
print(robo.move_to(next_tcp.tolist(),rotation=upright_rotation.tolist(),arm='right'))
observe_review()
display(robo.image('top')[620:1010,830:1240])