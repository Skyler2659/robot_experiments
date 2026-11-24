new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
review('Tube remains upright and visibly above second hole, tip near (997,782); no rack displacement or contact. Continue short vertical descent.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
descend_tcp=np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy(); descend_tcp[2]-=.03
robo.move_to(descend_tcp.tolist(),rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[630:1030,830:1260])