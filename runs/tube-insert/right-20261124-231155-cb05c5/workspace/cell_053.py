new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
review('Tube is visibly lifted and held between the jaws after tilting; open rim points toward image lower-left and closed tip toward upper-right, consistent with expected rotation. Pose retained from grasp kinematics, with approximate dimensions.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
robo.move_to([.35,.15,.25],rotation=new_upright.tolist(),arm='right')
observe_review()
display(robo.image('top')[350:850,300:750])
new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
print('predicted ends',project(release_endpoints @ new_tube[:3,:3].T+new_tube[:3,3]),'pose',new_tube)