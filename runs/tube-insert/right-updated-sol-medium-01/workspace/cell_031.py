try: review('Wrist and top confirm released upright tube remained in second hole after withdrawal.',relations={'inserted':[{ 'object':tube,'target':rack,'hole_from_left':2}]})
except ValueError as e:
 print(str(e)); review('Wrist and top confirm released upright tube remained in second hole after withdrawal.')
world.read_context(); print(str(robo.home(arm='right'))[:580]); print('joints',robo.state()['right']['joints'])