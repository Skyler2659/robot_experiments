B=tcp(); D=B[:3,3]-A[:3,3]
def ray(uv): return C[:3,:3]@np.linalg.solve(K,[*uv,1])
def par(old,new):
 d0,d1=ray(old),ray(new); fit=np.linalg.lstsq(np.column_stack([-d0,d1]),D,rcond=None)[0]; return C[:3,3]+d1*fit[1]
newR=(875,495); newB=(884,625); b3=par(uvB,newB); r3=par(uvR,newR); print('3d endpoints',b3,r3,'len',np.linalg.norm(r3-b3),'delta',D,'hole',np.array(hole))