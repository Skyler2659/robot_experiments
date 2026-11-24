tube_pose=np.array(robo.state()['right']['T_W_tcp']) @ attachment_local
review('Paired RGB shows upright tube centered in second hole with its lower segment below the top plate; open mouth visible above. Rack has no visible displacement. Retained geometry estimates insertion depth.',poses={'loose_tube':pose_record(tube_pose)},held={'right':'loose_tube'})
ctx=world.read_context()
robo.set_gripper(.045,arm='right')
observe_review()
display(robo.image('top')[650:1030,830:1260])