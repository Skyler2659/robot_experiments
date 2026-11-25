from scipy.optimize import least_squares
uvholes=np.array([(801,807),(852,776),(902,747),(950,718),(998,689),(1044,662)]); ys=np.arange(6)*.025-.0625
def rackfit(p):
 x,y,theta,z=p; return np.array([proj(np.array([x-np.sin(theta)*d,y+np.cos(theta)*d,z])) for d in ys])
fit=least_squares(lambda p:(rackfit(p)-uvholes).ravel(),[.42,.32,.55,.135]).x; print('fit',fit,rackfit(fit)); print('near mesh height', [back(*uv,.127) for uv in [(852,776),(902,747)]]); print('feet',back(813,974,-.012),back(1104,784,-.012))
def pose(x,y,z):
 t=np.eye(4); t[:3,:3]=np.column_stack([np.cross(y,up),y,up]); t[:3,3]=[x[0],x[1],z]; return t
opened=back(554,716,-.0049); closed=back(650,592,-.0049); axis=norm(closed-opened); TT=pose(opened+.04607*axis,axis,-.0049); rr=np.array([-np.sin(fit[2]),np.cos(fit[2]),0]); RT=pose(fit,rr,fit[3]-.139); print('poses',TT[:3,3],RT[:3,3], axis)
def pp(t,axes=['x','y'],ori=['z']): return {'T_W_object':t.tolist(),'translation_observed_axes':axes,'orientation_observed_axes':ori}
res=world.integrate_rgb(obs['capture'],poses={tube:pp(TT),rack:pp(RT)},expected_revision=s.revision,note='Calibrated top RGB: tube open and rounded endpoints projected at tabletop+radius; fit rack row yaw and center to six visible rim centers at mesh pitch, height inferred from perspective and reused pitch.'); print(str(res)[:850])