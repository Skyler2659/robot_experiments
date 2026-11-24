tube_pose[:3,3] = backproject([585,746],.053) - .0455*tube_axis
review('Tube mouth moved upward with jaws during 58mm lift and original table location is empty; aperture stayed 17.6mm. Attachment confirmed by relative motion. Pose from visible mouth and lifted height estimate; rack unchanged.', poses={'loose_tube':tube_pose.tolist()}, held={'right':'loose_tube'})
ctx = world.read_context()
print(robo.move_to([grasp_point[0],grasp_point[1],.23], rotation=pick_rotation.tolist(), arm='right'))
observe_review()