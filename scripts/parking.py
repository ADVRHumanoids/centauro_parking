#!/usr/bin/env python

from cartesian_interface.pyci_all import *
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
import numpy as np
import rospy
import rospkg
import argparse
import os, sys
import yaml

file_dir = os.path.dirname(os.path.abspath(__file__))

if sys.version_info[0] < 3:
    input = raw_input


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def get_xbot_cfg(urdf, srdf):
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_bool_parameter('is_model_floating_base', True)
    return cfg

def update_ik(ci, model, time, dt):
    ci.update(time, dt)
    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot * dt
    model.setJointPosition(q)
    model.update()
    return q, qdot

def quintic(alpha):
    if alpha < 0:
        return 0
    elif alpha > 1:
        return 1
    else:
        return ((6*alpha - 15)*alpha + 10)*alpha**3

def compute_h_frame(model, w_T_base, contacts):
    points = np.ones((3, len(contacts)))
    for i, c in enumerate(contacts):
        points[:, i] = model.getPose(c).translation
    points = points - np.mean(points, axis=1, keepdims=True)
    U, S, V = np.linalg.svd(points)
    print(points, U, S, V)
    n = U[:, -1]
    if n[2] < 0:
        n *= -1
    a = w_T_base.linear[:, 0]
    s = np.cross(n, a)  # x ^ z = y
    p = w_T_base.translation


    Th = Affine3()
    print(np.vstack([a, s, n]).T)
    Th.linear = np.vstack([a, s, n]).T
    Th.translation = p
    return Th

def cartesian_motion(qinit, qgoal, T, dt, ci, steering):
    # set steering state...
    if steering:
        print(bcolors.OKGREEN + 'Enabling steering tasks' + bcolors.ENDC)
        ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Enabled)
        ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Enabled)
        ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Enabled)
        ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Enabled)
    else:
        print(bcolors.OKGREEN + 'Disabling steering tasks' + bcolors.ENDC)
        ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Disabled)
        ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Disabled)
        ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Disabled)
        ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Disabled)

    time = 0
    while True:
        tau = time / T
        alpha = quintic(tau)
        qref = model.mapToEigen(qinit) * (1 - alpha) + model.mapToEigen(qgoal) * alpha
        qref = model.eigenToMap(qref)
        postural.setReferencePosture(qref)

        q, qdot = update_ik(ci, model, time, dt)
        rspub.publishTransforms('park')

        if robot is not None:
            robot.setPositionReference(q[6:])
            robot.setVelocityReference(model.eigenToMap(qdot))
            robot.move()

        time += dt

        # ...and change it before the action ends to avoid useless yaw rotations
        if time > T:
            if not steering and ci.getTask("steering_wheel_1").getActivationState() == pyci.ActivationState.Disabled:
                print(bcolors.OKGREEN + 'Enabling steering tasks' + bcolors.ENDC)
                ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Enabled)
                ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Enabled)
                ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Enabled)
                ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Enabled)
            # else:
            #     print(bcolors.OKGREEN + 'Enabling steering tasks' + bcolors.ENDC)
            #     ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Enabled)
            #     ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Enabled)
            #     ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Enabled)
            #     ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Enabled)
            if np.linalg.norm(qdot) < 0.01:
                break

        rate.sleep()

def rotate_wheels(qinit):
    q = qinit.copy()
    q['ankle_yaw_1'] = 0.0
    q['ankle_yaw_2'] = 0.0
    q['ankle_yaw_3'] = 0.0
    q['ankle_yaw_4'] = 0.0
    time = 0
    T = 1.0
    while time < T:
        tau = time / T
        alpha = quintic(tau)
        qref = model.mapToEigen(qinit) * (1 - alpha) + model.mapToEigen(q) * alpha
        model.setJointPosition(qref)
        model.update()
        rspub.publishTransforms('park')
        if robot is not None:
            robot.setPositionReference(qref[6:])
            robot.move()
        time += dt
        rate.sleep()
    ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Disabled)
    ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Disabled)
    ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Disabled)
    ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Disabled)

    return q

rospy.init_node('contact_homing')

# load configurations
with open(file_dir + '/../config/parking.yaml') as file:
    q_cfg = yaml.load(file, Loader=yaml.FullLoader)

parser = argparse.ArgumentParser()
parser.add_argument("--unattended", action="store_true", help='the script does not ask for user confirmation before executing the action')
parser.add_argument("--action", choices=q_cfg['actions'].keys())
args = parser.parse_args()

args.action = 'unparking'

# create XBot config object
urdf_path = rospkg.RosPack().get_path('centauro_urdf') + '/urdf/centauro.urdf'
urdf = open(urdf_path, 'r').read()

srdf_path = rospkg.RosPack().get_path('centauro_srdf') + '/srdf/centauro.srdf'
srdf = open(srdf_path, 'r').read()

cfg = get_xbot_cfg(urdf, srdf)

# create ModelInterface and RobotInterface (if xbot is running)
try:
    robot = xbot.RobotInterface(cfg)
    robot.sense()
    robot.setControlMode(xbot.ControlMode.Position())
    ctrlmode = {'j_wheel_{}'.format(i+1) : xbot.ControlMode.Velocity() for i in range(4)}
    ctrlmode['neck_velodyne'] = xbot.ControlMode.Idle()
    ctrlmode['d435_head_joint'] = xbot.ControlMode.Idle()
    robot.setControlMode(ctrlmode)
except:
    print(bcolors.FAIL + 'RobotInterface not created' + bcolors.ENDC)
    exit()

# sync ModelInterface to RobotInterface current position references to avoid discontinuities
model = xbot.ModelInterface(cfg)
qref = robot.getPositionReference()
qref = [0., 0., 0., 0., 0., 0.] + qref.tolist()
model.setJointPosition(qref)
model.update()

rspub = pyci.RobotStatePublisher(model)
rspub.publishTransforms('park')

dt = 0.01
ikpb = open(file_dir + '/../config/centauro_parking_stack.yaml', 'r').read()
ci = pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)
postural = ci.getTask('Postural')

rate = rospy.Rate(1./dt)
T = 5.0

# fill the action_list starting from ModelInterface current configuration
qinit = model.getJointPositionMap()
for index, action in enumerate(q_cfg['actions'][args.action]):
    if action == 'rotate_wheels':
        qinit = rotate_wheels(qinit)
    else:
        cartesian_motion(qinit, q_cfg[action['q']], T, dt, ci, action['steering'])
        qinit = q_cfg[action['q']]
