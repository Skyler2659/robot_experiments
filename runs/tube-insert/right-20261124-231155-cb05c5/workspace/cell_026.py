def pose_record(transform):
    return {'T_W_object':np.asarray(transform).tolist(),'translation_observed_axes':[True,True,True],'orientation_observed_axes':[True,True,True]}
try:
    review('Tube visibly followed jaws upward; original location empty and aperture stable. Pose estimated from mouth and measured lift; rack unchanged.', poses={'loose_tube':pose_record(tube_pose)}, held={'right':'loose_tube'})
    attachment_local = np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ tube_pose
    ctx = world.read_context()
    print(robo.move_to([grasp_point[0],grasp_point[1],.23], rotation=pick_rotation.tolist(), arm='right'))
    observe_review()
except Exception as error:
    print(type(error).__name__,str(error))