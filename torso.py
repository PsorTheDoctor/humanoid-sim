import pybullet as p


class Torso:
    def __init__(self):
        self.id = p.loadURDF(
          './torso/urdf/torso.urdf', basePosition=[-0.75, 1.3, -0.75], useFixedBase=True
        )
        self.n_joints = p.getNumJoints(self.id)
        self.end_effector = 5

    def get_joint_poses(self):
        return [p.getJointState(self.id, i)[0] for i in range(self.n_joints)]

    def get_end_effector_position_and_orientation(self):
        end_effector_state = p.getLinkState(self.id, self.end_effector)
        pos, orn = end_effector_state[:2]
        return pos, orn
