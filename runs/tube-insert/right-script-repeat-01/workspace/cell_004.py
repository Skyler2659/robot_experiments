K=np.array(v['K']); C=np.array(v['T_W_C']); invK=np.linalg.inv(K)
def back(uv,z):
 d=C[:3,:3]@invK@np.r_[uv,1]; return C[:3,3]+d*((z-C[2,3])/d[2])
def proj(p):
 q=K@(C[:3,:3].T@(np.array(p)-C[:3,3])); return q[:2]/q[2]
def unit(a): return a/np.linalg.norm(a)
for oid in ['loose_tube','tube_rack']:
 tf=s.transform(oid); m=world.mesh(oid); print(oid,m.bounds,tf); print([(n['path'],n['matrix_local_column_major'][12:15],n['vertices']) for n in world.parts(oid)['nodes'] if n['type']=='Mesh'])
print('tube', [back(uv,-.0049) for uv in [(438,813),(398,680)]]); print('holes', [back(uv,.086) for uv in [(850,677),(905,668),(959,660),(1014,651),(1067,643),(1121,635)]])