review('Paired top rack unchanged, tube occluded by jaws; wrist visually shows open jaws straddling tube near mouth.'); defdummy=0
def jaw(w,g=False):
 world.read_context()
 if g: world.expect_contacts(grasp={'right':tube})
 print(str(robo.set_gripper(w,arm='right'))[:850]); print('width',robo.state()['right']['gripper_width_m'])
jaw(0,True)