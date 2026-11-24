tube_pose = np.array(robo.state()['right']['T_W_tcp']) @ attachment_local
review('Tube remains upright with mouth above gripper, lower end visibly entered second hole from image left; rack edges show no visible displacement. Depth estimated from retained geometry.', poses={'loose_tube':pose_record(tube_pose)}, held={'right':'loose_tube'})
ctx=world.read_context()
next_tcp=np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy()
next_tcp[2]-=.012
robo.move_to(next_tcp.tolist(), rotation=upright_rotation, arm='right')
observe_review()
display(robo.image('top')[650:1030,830:1260])