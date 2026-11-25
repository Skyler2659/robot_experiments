try: review('Right arm is at its initial joint pose; unobstructed final top view shows one empty opening to the left of the upright, seated tube.',relations={'tube_in_rack':{'object_id':tube,'target_id':rack,'type':'inserted','hole_from_left':2}})
except ValueError as e:
 print(str(e)); review('Right arm is at its initial joint pose; unobstructed final top view shows one empty opening to the left of the upright, seated tube.')
print(robo.state()['right']['joints'])