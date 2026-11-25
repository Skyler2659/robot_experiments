tube='loose_tube'; rack='tube_rack'; snap=world.snapshot()
for key in [tube,rack]:
 m=world.mesh(key); print(key,type(m), list(m) if isinstance(m,dict) else str(m)[:200]); print(str(world.parts(key))[:250])
view=obs['views'][0]; K=np.array(view['K']); C=np.array(view['T_W_C']); print(C)
def bp(u,v,z):
 d=C[:3,:3]@np.linalg.solve(K,[u,v,1]); return C[:3,3]+d*((z-C[2,3])/d[2])
def proj(p):
 q=K@(C[:3,:3].T@(np.array(p)-C[:3,3])); return q[:2]/q[2]
for z in [.126,.111,.100,.09,.08]: print(z,np.round([bp(u,v,z) for u,v in [(824,642),(874,654),(1091,701)]],4))
print('tube',bp(455,806,-.0049),bp(409,680,-.0049)); print('view',view.keys())