ctx = world.read_context()
approach = world.target(lambda scene: scene.position('lemon') + np.array([-0.063, -0.148, 0.186]))
yaw = 0.55
down_rotation = [[-math.cos(yaw), -math.sin(yaw), 0.0], [-math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, -1.0]]
receipt = robo.move_to(approach, rotation=down_rotation, arm='right')
print('receipt', receipt)
ctx = world.read_context()
print('tcp', robo.state()['right']['T_W_tcp'])
if ctx['interaction']['pending_update']:
    world.compare_views(ctx['interaction']['pending_update']['capture'], object_ids=['lemon', 'plate'])