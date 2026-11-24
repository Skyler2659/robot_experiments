new_tube=np.array(robo.state()['right']['T_W_tcp']) @ new_attachment
review('Tube mouth and lower tip both moved upward with the 14 mm hand lift and remain steady relative to jaws; upright attachment now visually corroborated over consecutive observations.',poses={'loose_tube':pose_record(new_tube)},held={'right':'loose_tube'})
ctx=world.read_context()
new_offset=new_tube[:3,3]-np.array(robo.state()['right']['T_W_tcp'])[:3,3]
above_second=np.array([hole_two[0]-new_offset[0],hole_two[1]-new_offset[1],.265])
print('Above second TCP',above_second,'tube offset',new_offset)
robo.move_to(above_second.tolist(),rotation=upright_rotation,arm='right')
observe_review()
display(robo.image('top')[550:1030,830:1260])