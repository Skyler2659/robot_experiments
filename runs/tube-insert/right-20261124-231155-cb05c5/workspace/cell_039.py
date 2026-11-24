ctx=world.read_context()
print(robo.image('top').shape)
try:
    robo.show('r_wrist')
except Exception as error:
    print(type(error).__name__,str(error))
display(np.clip(robo.image('top')[730:1040,880:1270].astype(float)*3.5,0,255).astype(np.uint8))