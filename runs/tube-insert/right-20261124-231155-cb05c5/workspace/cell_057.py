upright_pixels=np.array([[552,438],[607,576]])
upright_fit=least_squares(lambda position:(project(visual_endpoints @ new_tube[:3,:3].T+position)-upright_pixels).ravel(),new_tube[:3,3])
new_tube[:3,3]=upright_fit.x
new_attachment=np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ new_tube
print('Visible tube fit',new_tube,'residual pixels',upright_fit.fun)
review('Tube fully visible held upright, mouth at (552,438), rounded closed tip at (607,576). Re-estimate translation from both visible endpoints and prior measured horizontal length; previous kinematic estimate did not match current RGB.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
world.expect_contacts(grasp={'right':'loose_tube'})
ctx=world.read_context()
robo.move_to([.35,.15,.265],rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[330:780,400:800])