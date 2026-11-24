visual_length=float(np.linalg.norm(closed_world-open_world))
visual_endpoints=np.array([[0,-visual_length/2+.0035,0],[0,visual_length/2+.0035,0]])
visible_pixels=np.array([[398,633],[516,586]])
pose_fit=least_squares(lambda position:np.r_[(project(visual_endpoints @ new_tube[:3,:3].T+position)-visible_pixels).ravel(),(position-new_tube[:3,3])*300],new_tube[:3,3])
new_tube[:3,3]=pose_fit.x
new_attachment=np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ new_tube
print('Refined estimate',new_tube[:3,3],'residual',pose_fit.fun)
obs=world.observe()
world.integrate_rgb(obs['capture'],poses={'loose_tube':pose_record(new_tube)},expected_revision=world.snapshot().revision,note='Visible open mouth at (398,633), rounded end at (516,586), held between jaws. Refine projection using observed tabletop endpoint separation and hand-pose regularization; depth remains approximate.')
ctx=world.read_context()
world.expect_contacts(grasp={'right':'loose_tube'})
ctx=world.read_context()
robo.move_to([.35,.12,.235],rotation=mid_rotation,arm='right')
observe_review()