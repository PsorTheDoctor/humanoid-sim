import socket
import pybullet as p
import pybullet_data
import argparse
import math
import time
import json

from torso import Torso
from hub import Hub
from robot_control import robot_control


def main(args):
    hubs = {}
    if args.connect:
        hubs['front_hub'] = Hub('169.254.72.8', 1025)
        hubs['rear_hub'] = Hub('169.254.72.8', 1025)

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF('plane.urdf')
    torso = Torso()

    robot_control(args, torso, hubs)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--ctrl_space', type=str, default='cartesian',
        help='Control space. Choose either joint or cartesian. Default is cartesian.'
    )
    ap.add_argument('--connect', action='store_true',
        help='Connect with the EV3 hubs.'
    )
    ap.add_argument('--vis', action='store_true',
        help='Visualize trajectory.'
    )
    args = ap.parse_args()
    main(args)
