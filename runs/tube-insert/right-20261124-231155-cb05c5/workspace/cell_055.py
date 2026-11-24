new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
review('Visible open rim moved with wrist during yaw change and lift; tube remains between closed jaws, corroborating attachment. The shaft is partly occluded; retain hand-consistent pose estimate.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
robo.move_to([.35,.15,.25],rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[350:850,350:800])
new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
print('pose',new_tube,'visual ends',project(visual_endpoints @ new_tube[:3,:3].T+new_tube[:3,3]))