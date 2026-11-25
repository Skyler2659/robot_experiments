cam=obs['views'][0]; K=np.array(cam['K']); C=np.array(cam['T_W_C']); up=np.array([0,0,1]); norm=lambda x:x/np.linalg.norm(x)
def back(u,v,z):
 d=C[:3,:3]@np.linalg.solve(K,[u,v,1]); return C[:3,3]+d*(z-C[2,3])/d[2]
def proj(p):
 q=K@(C[:3,:3].T@(p-C[:3,3])); return q[:2]/q[2]
s=world.snapshot(); orig={i:np.asarray(s.transform(i)) for i in (tube,rack)}
def localverts(i):
 t=orig[i]; return (np.asarray(world.mesh(i).vertices)-t[:3,3])@t[:3,:3]
for i in (tube,rack):
 v=localverts(i); print(i,len(v),v.min(0),v.max(0)); print([(np.round(np.array(n['matrix_local_column_major'])[12:15],4).tolist(),n['vertices']) for n in world.parts(i)['nodes'] if n['type']=='Mesh'])
print([back(*uv,-.0049) for uv in [(553,715),(650,593)]]); print([back(*uv,.10) for uv in [(801,807),(852,776),(902,747),(950,718),(998,689),(1044,662)]])