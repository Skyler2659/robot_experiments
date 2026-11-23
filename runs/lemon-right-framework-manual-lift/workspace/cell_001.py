ctx = world.read_context()
state0 = robo.state()
log(ctx)
log(state0)
def find_tcp(value):
    if isinstance(value, dict):
        if 'T_W_tcp' in value:
            return np.asarray(value['T_W_tcp'], dtype=float)
        if 'right' in value:
            found = find_tcp(value['right'])
            if found is not None:
                return found
        for key, item in value.items():
            if key != 'left':
                found = find_tcp(item)
                if found is not None:
                    return found
    return None
start_tcp = find_tcp(state0)
if start_tcp is None:
    raise ValueError('No measured T_W_tcp found; no motion issued.')
assert start_tcp.shape == (4, 4) and np.isfinite(start_tcp).all()
assert abs(float(start_tcp[2, 3]) - 0.046) < 0.015
lift_xyz = start_tcp[:3, 3].copy()
lift_xyz[2] = 0.160
world.expect_contacts(grasp={'right': 'lemon'})
log(robo.move_to(lift_xyz.tolist(), rotation=start_tcp[:3, :3].tolist(), arm='right'))
robo.show('top')
log(robo.state())