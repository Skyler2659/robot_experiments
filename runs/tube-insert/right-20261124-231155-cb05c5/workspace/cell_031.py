end_pixels = np.array([[552,439],[607,580]])
upright_fit = least_squares(upright_residual,upright_pose[:3,3]+[0,0,.0193])
upright_pose[:3,3] = upright_fit.x
review('Tube endpoints now (552,439) and (607,580), consistent with 19.3mm wrist lift; mouth remains up with stable grip. Earlier pose discrepancy came from depth refinement when tube became fully visible.',poses={'loose_tube':pose_record(upright_pose)},held={'right':'loose_tube'})
attachment_local = np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ upright_pose
ctx = world.read_context()
print('grasp review',ctx['grasp_review'])
print(robo.move_to(above_hole_tcp.tolist(),rotation=upright_rotation.tolist(),arm='right'))
observe_review()
display(robo.image('top')[400:1040,750:1330])