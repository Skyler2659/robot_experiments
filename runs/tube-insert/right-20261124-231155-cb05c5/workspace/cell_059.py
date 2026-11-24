new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
current_tip=new_tube[:3,:3] @ visual_endpoints[1]+new_tube[:3,3]
observed_tip=backproject([973,717],float(current_tip[2]))
new_tube[:2,3]+=observed_tip[:2]-current_tip[:2]
new_attachment=np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ new_tube
review('Held tube fully visible above rack with closed tip near (973,717), slightly left of intended second-hole vertical. Refine lateral pose at kinematically estimated tip height; rack unchanged.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
corrected_tcp=np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy()
corrected_tcp[:2]+=hole_two[:2]-observed_tip[:2]
corrected_tcp[2]-=.045
print('corrected tcp',corrected_tcp,'lateral delta',hole_two[:2]-observed_tip[:2])
robo.move_to(corrected_tcp.tolist(),rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[600:1030,830:1260])