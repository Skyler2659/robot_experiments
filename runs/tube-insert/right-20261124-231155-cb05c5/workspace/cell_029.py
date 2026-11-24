upright_pose = np.array(robo.state()['right']['T_W_tcp']) @ attachment_local
vertical_rotation = upright_pose[:3,:3].copy()
vertical_endpoints = np.array([[0,-.0455,0],[0,.0525,0]])
end_pixels = np.array([[564,471],[618,605]])
def upright_residual(center):
    return (project(vertical_endpoints @ vertical_rotation.T + center)-end_pixels).ravel()
upright_fit = least_squares(upright_residual,upright_pose[:3,3])
upright_pose[:3,3] = upright_fit.x
print('upright fit',upright_fit.x,'residual',upright_residual(upright_fit.x),'tcp offset',upright_fit.x-np.array(robo.state()['right']['T_W_tcp'])[:3,3])
review('Entire tube visible upright: open rim pixel (564,471), rounded bottom (618,605). Fitted unchanged tube geometry to both endpoints; mouth up and attachment maintained through rotation.',poses={'loose_tube':pose_record(upright_pose)},held={'right':'loose_tube'})
attachment_local = np.linalg.inv(np.array(robo.state()['right']['T_W_tcp'])) @ upright_pose
hole_two = (rack_pose @ np.array([0,-.0375,.139,1]))[:3]
tube_tcp_offset = upright_pose[:3,3] - np.array(robo.state()['right']['T_W_tcp'])[:3,3]
above_hole_tcp = np.array([hole_two[0]-tube_tcp_offset[0],hole_two[1]-tube_tcp_offset[1],.28])
print('hole two',hole_two,'approach tcp',above_hole_tcp)
ctx = world.read_context()
print(robo.move_to(above_hole_tcp.tolist(),rotation=upright_rotation.tolist(),arm='right'))
observe_review()
display(robo.image('top')[350:1040,750:1300])