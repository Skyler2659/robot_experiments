tube='loose_tube'; rack='tube_rack'; print(world.parts(rack)); print(world.parts(tube)); print(world.mesh(rack).keys()); cam=obs['views'][0]; K=np.array(cam['K']); C=np.array(cam['T_W_C']); defdummy=0
def back(u,v,z):
 d=C[:3,:3]@np.linalg.solve(K,[u,v,1]); return C[:3,3]+d*(z-C[2,3])/d[2]
print([back(*uv,-.0049) for uv in [(553,715),(650,593)]]); print([back(*uv,.10) for uv in [(801,807),(852,776),(902,747),(950,718),(998,689),(1044,662)]])