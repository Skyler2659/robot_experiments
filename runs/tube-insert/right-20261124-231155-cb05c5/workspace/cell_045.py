review('Tube remains stationary on tabletop, clear of right arm; rack unchanged.',poses={'loose_tube':pose_record(recovered_pose)},held={'right':None})
ctx=world.read_context()
robo.move_to(new_pregrasp.tolist(),rotation=new_pick.tolist(),arm='right')
observe_review()