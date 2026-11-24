def review(note, poses=None, held=None, relations=None):
    context = world.read_context()
    pending = context['interaction']['pending_update']
    if pending:
        return world.commit_step(pending['capture'], expected_revision=context['revision'], note=note, poses=poses or {}, held=held or {}, relations=relations or {})
def observe_review():
    context = world.read_context()
    if context['interaction']['pending_update']:
        world.compare_views(context['interaction']['pending_update']['capture'])
    print('tcp',np.array(robo.state()['right']['T_W_tcp'])[:3,3].round(4),'jaw',robo.state()['right']['gripper_width_m'])
review('Arm moved freely above table; tube and rack visibly unchanged and no attachment.')
from scipy.spatial.transform import Rotation
mid_rotation = Rotation.from_euler('z',.35).as_matrix() @ Rotation.from_euler('y',2.35).as_matrix()
ctx = world.read_context()
print(robo.move_to([.28,.08,.22], rotation=mid_rotation.tolist(), arm='right'))
observe_review()