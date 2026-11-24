ctx=world.read_context()
world.expect_contacts(grasp={'right':'loose_tube'})
ctx=world.read_context()
robo.move_to([.35,.15,.25],rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[350:850,400:850])
new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
print('tcp',np.array(robo.state()['right']['T_W_tcp']))
print('predicted ends',project(visual_endpoints @ new_tube[:3,:3].T+new_tube[:3,3]))