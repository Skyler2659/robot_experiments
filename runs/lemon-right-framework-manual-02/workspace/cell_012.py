ctx = world.read_context()
world.commit_step(ctx['interaction']['pending_update']['capture'], expected_revision=ctx['revision'], note='Blocked approach produced no arm or object displacement; paired top views show lemon and plate unchanged. Change approach orientation rather than repeat failed pose.', poses={}, held={}, relations={})
ctx = world.read_context()
pitch = 2.25
inclined_rotation = [[math.cos(yaw)*math.cos(pitch), -math.sin(yaw), math.cos(yaw)*math.sin(pitch)], [math.sin(yaw)*math.cos(pitch), math.cos(yaw), math.sin(yaw)*math.sin(pitch)], [-math.sin(pitch), 0.0, math.cos(pitch)]]
approach = world.target(lambda scene: scene.position('lemon') + np.array([-0.033, -0.158, 0.206]))
receipt = robo.move_to(approach, rotation=inclined_rotation, arm='right')
print('receipt', receipt)