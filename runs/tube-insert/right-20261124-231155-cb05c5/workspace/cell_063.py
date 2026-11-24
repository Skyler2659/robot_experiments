ctx=world.read_context()
robo.set_gripper(.075,arm='right')
observe_review()
display(robo.image('top')[650:1030,830:1260])