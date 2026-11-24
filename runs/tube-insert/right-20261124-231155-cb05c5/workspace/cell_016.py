review('Intermediate approach remains well above tabletop; visible tube and rack unchanged.')
ctx = world.read_context()
print(robo.move_to(pregrasp, rotation=pick_rotation.tolist(), arm='right'))
observe_review()