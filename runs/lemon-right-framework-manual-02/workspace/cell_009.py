ctx = world.read_context()
receipt = robo.set_gripper(0.0697, arm='right')
print('receipt', receipt)
ctx = world.read_context()
print('interaction', ctx['interaction'])
print('jaw_m', robo.state()['right']['gripper_width_m'])
if ctx['interaction']['pending_update']:
    world.compare_views(ctx['interaction']['pending_update']['capture'], object_ids=['lemon', 'plate'])