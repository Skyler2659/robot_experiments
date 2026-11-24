ctx = world.read_context()
print(robo.move_to([grasp_point[0],grasp_point[1],.060], rotation=pick_rotation.tolist(), arm='right'))
observe_review()
display(robo.image('top')[600:880,400:740])