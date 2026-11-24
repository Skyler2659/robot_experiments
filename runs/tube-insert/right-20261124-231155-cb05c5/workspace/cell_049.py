ctx=world.read_context()
world.commit_step(ctx['interaction']['pending_update']['capture'],expected_revision=ctx['revision'],note='Reached grasp height without visible tabletop disturbance; tube remains occluded beneath gripper.',unresolved={'loose_tube':'At recent known tabletop pose beneath wrist; occluded.'},held={'right':None})
ctx=world.read_context()
world.expect_contacts(grasp={'right':'loose_tube'})
ctx=world.read_context()
robo.set_gripper(.012,arm='right')
observe_review()
new_attachment=np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ recovered_pose