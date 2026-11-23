ctx = world.read_context()
world.commit_step(ctx['interaction']['pending_update']['capture'], expected_revision=ctx['revision'], note='Vertical pregrasp planning failed with no motion. Paired RGB confirms lemon and plate unchanged. Use a modestly inclined pregrasp instead.', poses={}, held={}, relations={})
ctx = world.read_context()
pitch = 2.75
grasp_rotation = [[math.cos(yaw)*math.cos(pitch), -math.sin(yaw), math.cos(yaw)*math.sin(pitch)], [math.sin(yaw)*math.cos(pitch), math.cos(yaw), math.sin(yaw)*math.sin(pitch)], [-math.sin(pitch), 0.0, math.cos(pitch)]]
pregrasp = world.target(lambda scene: scene.position('lemon') + np.array([0.0, 0.0, 0.126]))
receipt = robo.move_to(pregrasp, rotation=grasp_rotation, arm='right')
print('receipt', receipt)