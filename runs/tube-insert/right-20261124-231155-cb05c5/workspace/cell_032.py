tube_pose = np.array(robo.state()['right']['T_W_tcp']) @ attachment_local
tip = (tube_pose @ np.array([0,.0525,0,1]))[:3]
print('tip prediction',tip,'pixel',project(tip))
tube_pose[:3,3] += backproject([1001,761],tip[2])-tip
review('Tube held vertically above rack; open rim near (1007,629), closed end (1001,761). Rack unchanged. Refined lateral translation from visible bottom at hand-derived height.',poses={'loose_tube':pose_record(tube_pose)},held={'right':'loose_tube'})
attachment_local = np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ tube_pose
tip = (tube_pose @ np.array([0,.0525,0,1]))[:3]
align_tcp = np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy()
align_tcp[:2] += hole_two[:2]-tip[:2]
align_tcp[2] += (hole_two[2]+.020)-tip[2]
print('align tcp',align_tcp)
ctx = world.read_context()
print(robo.move_to(align_tcp.tolist(),rotation=upright_rotation.tolist(),arm='right'))
observe_review()
display(robo.image('top')[580:1040,800:1300])