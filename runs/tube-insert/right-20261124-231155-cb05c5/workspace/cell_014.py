ctx = world.read_context()
receipt = robo.move_to([.23,0,.23], rotation=initial_tcp[:3,:3].tolist(), arm='right')
print(receipt)
ctx = world.read_context()
if ctx['interaction']['pending_update']:
    comparison = world.compare_views(ctx['interaction']['pending_update']['capture'])