pick_rotation = np.column_stack([-tube_axis, np.cross([0,0,-1],-tube_axis),[0,0,-1]])
ctx = world.read_context()
receipt = robo.move_to([.28,.08,.24], rotation=pick_rotation.tolist(), arm='right')
print(receipt)
ctx = world.read_context()
if ctx['interaction']['pending_update']:
    comparison = world.compare_views(ctx['interaction']['pending_update']['capture'])