def uprightfit(uvopen,uvcap,delta=.083):
 def f(p): return np.r_[proj(p)-uvopen,proj(p-delta*up)-uvcap]
 return least_squares(f,(pred@[0,-.04607,0,1])[:3]).x
mouth=uprightfit(np.array([650.5,418]),np.array([688,553])); print('mouth fit',mouth,proj(mouth),proj(mouth-.083*up)); print('sensible at z',back(650.5,418,.268),back(688,553,.185)); print('relative mouth',np.linalg.solve(tcp(),np.r_[mouth,1])); print('hypothesis near-point',np.linalg.solve(tcp(),np.r_[back(650.5,418,.266),1]))