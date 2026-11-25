grabTCP=tcp(); rel0=np.linalg.inv(grabTCP)@TR
commit('Jaw stopped at 17.5 mm and the tube rim is visibly between finger pads in the top/geometry pair; attachment provisional until short lift.',held={'right':tube})
move(grabTCP[:3,3]+up*.070,Rg,'line'); display(robo.artifact('r_wrist.png')); crop((330,550,530,840)); print('review',world.read_context()['grasp_review'])