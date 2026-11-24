open_world=backproject([485,946],-.0049)
closed_world=backproject([562,788],-.0049)
closed_axis=(closed_world-open_world)/np.linalg.norm(closed_world-open_world)
recovered_pose=np.eye(4)
recovered_pose[:3,0]=[closed_axis[1],-closed_axis[0],0]
recovered_pose[:3,1]=closed_axis
recovered_pose[:3,3]=(open_world+closed_world)/2-closed_axis*.0035
print('ends',open_world,closed_world,'length',np.linalg.norm(closed_world-open_world),'closed axis',closed_axis)
review('Tube newly visible lying on tabletop clear of arm and rack. Rounded closed end at pixel (562,788), open rim at (485,946); calibrated table-plane backprojection estimates current pose. Rack unchanged.',poses={'loose_tube':pose_record(recovered_pose)},held={'right':None})
new_pick=np.column_stack([closed_axis,[closed_axis[1],-closed_axis[0],0],[0,0,-1]])
new_grasp=open_world+.025*closed_axis
new_pregrasp=new_grasp.copy(); new_pregrasp[2]=.13
print('grasp',new_grasp,'pick',new_pick)
ctx=world.read_context()
robo.move_to(new_pregrasp.tolist(),rotation=new_pick.tolist(),arm='right')
observe_review()