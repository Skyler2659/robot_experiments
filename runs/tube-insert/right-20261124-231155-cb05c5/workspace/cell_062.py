new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
review('Tube tip is centered over the second opening, now near its rim; no visible rack motion or tube tilt. Keep jaws closed during a final small descent.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
descend_tcp=np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy(); descend_tcp[2]-=.012
robo.move_to(descend_tcp.tolist(),rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[650:1030,830:1260])