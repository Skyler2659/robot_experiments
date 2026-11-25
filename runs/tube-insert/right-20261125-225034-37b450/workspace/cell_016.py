def point(T,local): return T[:3,:3]@local+T[:3,3]
# Use visible rim's projected center with the lift-height estimate; its depth remains provisional.
P[:3,3]+=(bp(412,750,point(P,[0,.0525,0])[2])-point(P,[0,.0525,0]))*.6
commit('Paired top shows rim lifted away from its table projection, wrist RGB confirms tube between fingers and jaw width remains 17.2 mm; update visible rim in XY, with depth extrapolated.',poses={tube:{**pose(P,ra=[False,False,False]),'time_basis':'image'}},held={'right':tube})
print('review',world.read_context()['grasp_review']); rel=np.linalg.inv(tcp())@np.array(world.snapshot().transform(tube)); print('relative',np.round(rel,4))
hloc=(holes[1]-RR[:3,3])@RR[:3,:3]; hole=world.target(lambda s:s.local_point(rack,hloc)); forward=unit(np.r_[np.array(hole)[:2],0.]); finaltool=np.column_stack([-up,np.cross(forward,-up),forward]); nom=finaltool@rel[:3,1]; cross=np.cross(nom,up); S=np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]]); Ri=(np.eye(3)+S+S@S/(1+nom@up))@finaltool
def itarget(z): return np.r_[np.array(hole)[:2],z]-Ri@rel[:3,3]
move(itarget(.210),Ri); display(robo.artifact('r_wrist.png')); bright((750,500,1060,800))