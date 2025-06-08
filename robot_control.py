import pybullet as p
import math
import json


def send_joint_poses(hubs, joint_poses: list):
    front_hub_data = {
        'shoulder1': joint_poses[0],
        'shoulder2': joint_poses[1],
        'arm': joint_poses[2]
    }

    rear_hub_data = {
        'elbow': joint_poses[3],
        'forearm': joint_poses[4],
        'wrist1': joint_poses[5],
        # 'wrist2': joint_poses[6]
    }

    hubs['front_hub'].send_message(json.dumps(front_hub_data) + '\n')
    hubs['rear_hub'].send_message(json.dumps(rear_hub_data) + '\n')


def draw_trajectory(torso, prev_pos):
    end_effector_state = p.getLinkState(torso.id, linkIndex=torso.end_effector)
    pos = end_effector_state[4]
    if prev_pos is not None:
        p.addUserDebugLine(prev_pos, pos, lineColorRGB=[1, 0, 0], lineWidth=2, lifeTime=10)

    prev_pos = pos
    return prev_pos


def joint_space_control(args, torso, hubs):
    joint_poses = torso.get_joint_poses()
    sliders = [
        p.addUserDebugParameter(f'Joint {i}', -math.pi, math.pi, joint_poses[i])
        for i in range(torso.n_joints)
    ]

    prev_pos = None
    try:
        while True:
            for i in range(torso.n_joints):
                joint_poses[i] = p.readUserDebugParameter(sliders[i])
                p.resetJointState(torso.id, i, joint_poses[i])

            if args.connect:
                send_joint_poses(hubs, joint_poses)

            if args.vis:
                prev_pos = draw_trajectory(torso, prev_pos)

            p.stepSimulation()

    except KeyboardInterrupt:
        print('')
    finally:
        if args.connect:
            hubs['front_hub'].close()
            hubs['rear_hub'].close()
        p.disconnect()


def cartesian_space_control(args, torso, hubs):
    pos, orn = torso.get_end_effector_position_and_orientation()
    sliders = [
        p.addUserDebugParameter('X', pos[0] - 1, pos[0] + 1, pos[0]),
        p.addUserDebugParameter('Y', pos[1] - 1, pos[1] + 1, pos[1]),
        p.addUserDebugParameter('Z', pos[2] - 1, pos[2] + 1, pos[2]),
        p.addUserDebugParameter('Roll', -math.pi, math.pi, orn[0]),
        p.addUserDebugParameter('Pitch', -math.pi, math.pi, orn[1]),
        p.addUserDebugParameter('Yaw', -math.pi, math.pi, orn[2])
    ]

    prev_pos = None
    try:
        while True:
            x = p.readUserDebugParameter(sliders[0])
            y = p.readUserDebugParameter(sliders[1])
            z = p.readUserDebugParameter(sliders[2])
            roll = p.readUserDebugParameter(sliders[3])
            pitch = p.readUserDebugParameter(sliders[4])
            yaw = p.readUserDebugParameter(sliders[5])

            target_orn = p.getQuaternionFromEuler([roll, pitch, yaw])
            joint_poses = p.calculateInverseKinematics(
                bodyIndex=torso.id,
                endEffectorLinkIndex=torso.end_effector,
                targetPosition=[x, y, z],
                targetOrientation=target_orn
            )

            for i in range(torso.n_joints):
                p.resetJointState(torso.id, i, joint_poses[i])

            if args.connect:
                send_joint_poses(hubs, joint_poses)

            if args.vis:
                prev_pos = draw_trajectory(torso, prev_pos)

            p.stepSimulation()

    except KeyboardInterrupt:
        print('')
    finally:
        if args.connect:
            hubs['front_hub'].close()
            hubs['rear_hub'].close()
        p.disconnect()


def robot_control(args, torso, hubs):
    if args.ctrl_space == 'joint':
        joint_space_control(args, torso, hubs)
    elif args.ctrl_space == 'cartesian':
        cartesian_space_control(args, torso, hubs)
    else:
        print(f'Argument {args.ctrl_space} is invalid')
