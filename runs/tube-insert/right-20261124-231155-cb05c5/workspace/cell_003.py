import inspect
print('signatures', [(name, str(inspect.signature(getattr(world, name)))) for name in ['integrate_rgb','commit_step','compare_views','target','points','bounds']])
print('views', obs['views'])
print('metadata', obs['metadata'])
print('tube nodes', [(node['path'],node['name'],node['matrix_local_column_major'][12:15]) for node in world.parts('loose_tube')['nodes']])
print('rack nodes', [(node['path'],node['name'],node['matrix_local_column_major'][12:15]) for node in world.parts('tube_rack')['nodes']])