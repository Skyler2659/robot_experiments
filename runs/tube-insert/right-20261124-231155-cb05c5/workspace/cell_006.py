from scipy.optimize import least_squares
camera = obs['views'][0]
intrinsics = np.array(camera['K'])
camera_pose = np.array(camera['T_W_C'])
def project(points):
    points = np.atleast_2d(points)
    camera_points = (points-camera_pose[:3,3]) @ camera_pose[:3,:3]
    pixels = camera_points @ intrinsics.T
    return pixels[:,:2]/pixels[:,2:3]
def backproject(pixel, height):
    ray = camera_pose[:3,:3] @ np.linalg.solve(intrinsics, [pixel[0],pixel[1],1.0])
    return camera_pose[:3,3] + ray*((height-camera_pose[2,3])/ray[2])
closed_tip = backproject([497,706], -0.0049)
open_tip = backproject([590,811], -0.0049)
tube_axis = (open_tip-closed_tip)/np.linalg.norm(open_tip-closed_tip)
tube_pose = np.eye(4)
tube_pose[:3,1] = tube_axis
tube_pose[:3,0] = np.cross(tube_axis,[0,0,1])
tube_pose[:3,3] = (closed_tip+open_tip)/2
rack_holes_pixels = np.array([[939,858],[993,835],[1045,811],[1098,788],[1150,766],[1206,743]],float)
rack_holes_local = np.array([[0, offset, .139] for offset in [-.0625,-.0375,-.0125,.0125,.0375,.0625]])
def rack_transform(parameters):
    center_x,center_y,center_z,yaw = parameters
    transform = np.eye(4)
    transform[:3,:3] = [[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]]
    transform[:3,3] = [center_x,center_y,center_z]
    return transform

def rack_residual(parameters):
    transform = rack_transform(parameters)
    return (project(rack_holes_local @ transform[:3,:3].T + transform[:3,3])-rack_holes_pixels).ravel()
rack_fit = least_squares(rack_residual,[.40,.37,-.012,.4])
rack_pose = rack_transform(rack_fit.x)
print('tube pose',tube_pose.round(5).tolist(),'length',np.linalg.norm(open_tip-closed_tip))
print('rack fit',rack_fit.x,'pixel residual',rack_residual(rack_fit.x).round(2))
print('tube ends vertices',[(node['path'],node['vertices']) for node in world.parts('loose_tube')['nodes']])
display(robo.image('top')[650:1040,450:1300])