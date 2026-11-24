release_rotation=tube_pose[:3,:3].copy()
release_endpoints=np.array([[0,-.0455,0],[0,.0525,0]])
release_pixels=np.array([[975,786],[977,883]])
release_fit=least_squares(lambda position:(project(release_endpoints @ release_rotation.T + position)-release_pixels).ravel(),tube_pose[:3,3])
released_pose=tube_pose.copy()
released_pose[:3,3]=release_fit.x
print('Release pose estimate',release_fit.x,'pixel residual',release_fit.fun)
review('After opening to 44.5 mm, tube visibly slipped down and slightly image-left while remaining upright at the second hole. Mouth and rounded lower silhouette provide approximate release pose; lower contact partly obscured by acrylic. Jaws no longer clamp tube.',poses={'loose_tube':pose_record(released_pose)},held={'right':None})
ctx=world.read_context()
retreat_tcp=np.array(robo.state()['right']['T_W_tcp'])[:3,3].copy()-np.array([.7266693176672074,.6869874109189158,0])*.07
robo.move_to(retreat_tcp.tolist(),rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[650:1030,830:1260])