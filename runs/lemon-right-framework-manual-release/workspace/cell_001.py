ctx = world.read_context()
world.expect_contacts(grasp={"right": "lemon"})
robo.set_gripper(0.0697, arm="right")
robo.show("top")