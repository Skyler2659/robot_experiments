for key in [tube,rack]:
 T=np.array(snap.transform(key)); m=world.mesh(key); print(key,T, m.bounds); parts=world.parts(key); print([(n.get('name'),n.get('path')) for n in parts['nodes']][:35]);
print('obs',obs['capture']); print('rack visual base samples', [bp(u,v,-.012) for u,v in [(779,788),(1108,865),(780,783)]]); print('end panel',[(z,bp(783,580,z)) for z in [.07,.09,.11,.126]])