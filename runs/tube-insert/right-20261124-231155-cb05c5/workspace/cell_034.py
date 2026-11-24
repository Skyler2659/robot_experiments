tube_pose = np.array(robo.state()['right']['T_W_tcp']) @ attachment_local
review('Tube bottom reaches the second hole opening; mouth upright, rack edges stationary and no buckling visible. Continue controlled 20mm insertion.',poses={'loose_tube':pose_record(tube_pose)},held={'right':'loose_tube'})
ctx = world.read_context()
next_tcp = np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy()
next_tcp[2] -= .020
print(robo.move_to(next_tcp.tolist(),rotation=upright_rotation.tolist(),arm='right'))
observe_review()
display(robo.image('top')[650:1030,830:1260])