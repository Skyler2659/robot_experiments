new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
review('Rounded tube tip is approaching the second-hole center, with a small visible gap; first hole on the left remains empty. Rack edges remain stationary.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
descend_tcp=np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy(); descend_tcp[2]-=.015
robo.move_to(descend_tcp.tolist(),rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[650:1030,830:1260])