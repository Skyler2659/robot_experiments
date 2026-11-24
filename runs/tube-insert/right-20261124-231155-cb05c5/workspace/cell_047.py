review('Downward approach orientation reached above table; tube remains visibly stationary and untouched.',poses={'loose_tube':pose_record(recovered_pose)},held={'right':None})
ctx=world.read_context()
robo.move_to([float(new_grasp[0]),float(new_grasp[1]),.06],rotation=new_pick.tolist(),arm='right')
observe_review()