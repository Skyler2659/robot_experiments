try:
    review('Tube mouth visibly followed jaws upward 58mm and table location is empty; stable aperture confirms grasp over time. Pose estimated from visible rim and height change; rack unchanged.', poses={'loose_tube':{'T_W_object':tube_pose.tolist()}}, held={'right':'loose_tube'})
    attachment_local = np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ tube_pose
    ctx = world.read_context()
    print(robo.move_to([grasp_point[0],grasp_point[1],.23], rotation=pick_rotation.tolist(), arm='right'))
    observe_review()
except Exception as error:
    print(type(error).__name__,str(error))