ctx=world.read_context()
robo.move_to([.33,.17,.28],rotation=upright_rotation,arm='right')
observe_review()
display(np.clip(robo.image('top').astype(float)*2.7,0,255).astype(np.uint8))