review('Reached pregrasp about 12cm above tube; tube and rack unchanged, tube still on table.')
ctx = world.read_context()
print(robo.set_gripper(.045, arm='right'))
observe_review()