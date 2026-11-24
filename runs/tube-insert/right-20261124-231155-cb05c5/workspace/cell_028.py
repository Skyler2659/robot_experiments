predicted_tube = np.array(robo.state()['right']['T_W_tcp']) @ attachment_local
predicted_mouth = (predicted_tube @ np.array([0,-.0455,0,1]))[:3]
print('predicted mouth pixel',project(predicted_mouth).round(1),'height',predicted_mouth[2])
tube_pose = predicted_tube.copy()
tube_pose[:3,3] += backproject([529,598],predicted_mouth[2]) - predicted_mouth
review('Open circular rim clearly visible near (529,598), moving with wrist through lift/rotation; tube remains attached and rack stationary. Refined tube translation from visible rim at kinematic height.',poses={'loose_tube':pose_record(tube_pose)},held={'right':'loose_tube'})
attachment_local = np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ tube_pose
upright_rotation = np.column_stack([[0,0,-1],[-tube_axis[1],tube_axis[0],0],tube_axis])
ctx = world.read_context()
print(robo.move_to([.35,.15,.24],rotation=upright_rotation.tolist(),arm='right'))
observe_review()
display(robo.image('top')[300:800,350:900])