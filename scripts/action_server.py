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

from std_srvs.srv import Trigger
from xbot_msgs.msg import JointState

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

def cartesian_motion(qinit, qgoal, T, dt, ci, steering, change_steering):
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

        if robot is not None:
            robot.setPositionReference(q[6:])
            robot.setVelocityReference(model.eigenToMap(qdot))
            robot.move()

        time += dt

        if time > T:
            # if change_steering and ci.getTask("steering_wheel_1").getActivationState() == pyci.ActivationState.Disabled:
            #     print(bcolors.OKGREEN + 'Enabling steering tasks' + bcolors.ENDC)
            #     ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Enabled)
            #     ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Enabled)
            #     ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Enabled)
            #     ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Enabled)
            if np.linalg.norm(qdot) < 0.01 or time > 2*T:
                break

        rate.sleep()

    rotate_wheels(model.getJointPositionMap(), qgoal)

def rotate_wheels(qinit, qgoal=None):
    if qgoal is None:
        qgoal = qinit.copy()
        qgoal['ankle_yaw_1'] = 0.0
        qgoal['ankle_yaw_2'] = 0.0
        qgoal['ankle_yaw_3'] = 0.0
        qgoal['ankle_yaw_4'] = 0.0
    time = 0
    T = 1.0
    dt = 0.01
    rate = rospy.Rate(1./dt)
    while time < T:
        tau = time / T
        alpha = quintic(tau)
        qref = model.mapToEigen(qinit) * (1 - alpha) + model.mapToEigen(qgoal) * alpha
        model.setJointPosition(qref)
        model.update()
        robot.setPositionReference(qref[6:])
        robot.move()
        time += dt
        rate.sleep()

    return qgoal

def rotate_ankle_pitch(qinit, fold, box=False):
    q = qinit.copy()
    if fold:
        q['ankle_pitch_1'] = -2.4
        q['ankle_pitch_2'] = 2.4
        if box:
            q['ankle_pitch_3'] = 2.4 - np.pi
            q['ankle_pitch_4'] = -2.4 + np.pi
        else:
            q['ankle_pitch_3'] = 2.4
            q['ankle_pitch_4'] = -2.4
    else:
        q['ankle_pitch_1'] = -0.84
        q['ankle_pitch_2'] = 0.84
        q['ankle_pitch_3'] = 0.84
        q['ankle_pitch_4'] = -0.84
    time = 0
    T = 5.0
    while time < T:
        tau = time / T
        alpha = quintic(tau)
        qref = model.mapToEigen(qinit) * (1 - alpha) + model.mapToEigen(q) * alpha
        model.setJointPosition(qref)
        model.update()
        robot.setPositionReference(qref[6:])
        robot.move()
        time += dt
        rate.sleep()

    return args.action

def action_service(req):
    T = 5.0
    dt = 0.01

    # fill the action_list starting from ModelInterface current configuration
    qref = model.getJointPosition()
    qref = model.eigenToMap(qref)
    for jn, q in zip(last_js.name, last_js.position_reference):
        qref[jn] = q
    model.setJointPosition(qref)
    model.update()

    qinit = model.getJointPositionMap()
    for index, action in enumerate(q_cfg['actions']['homing_legs']):
        if type(action) == str:
            if action == 'rotate_wheels':
                qinit = rotate_wheels(qinit)

        if type(action) == dict:
            cartesian_motion(qinit, q_cfg[list(action.keys())[0]], T, dt, ci, action[list(action.keys())[0]]['steering'], action[list(action.keys())[0]]['change_steering'])
            qinit = q_cfg[list(action.keys())[0]]

    return True


rospy.init_node('homing_action_server')

# load configurations
with open(file_dir + '/../config/parking.yaml') as file:
    q_cfg = yaml.load(file, Loader=yaml.FullLoader)

parser = argparse.ArgumentParser()
parser.add_argument("--unattended", action="store_true", help='the script does not ask for user confirmation before executing the action')
parser.add_argument("--action", choices=q_cfg['actions'].keys())
args = parser.parse_args()

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
    robot.setControlMode(ctrlmode)
except:
    print(bcolors.FAIL + 'RobotInterface not created' + bcolors.ENDC)
    exit()

# sync ModelInterface to RobotInterface current position references to avoid discontinuities
model = xbot.ModelInterface(cfg)

dt = 0.01
ikpb = open(file_dir + '/../config/centauro_parking_stack.yaml', 'r').read()
ci = pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)
postural = ci.getTask('Postural')

rate = rospy.Rate(1./dt)

last_js: JointState = None

def on_js_recv(msg: JointState):
    global last_js
    last_js = msg

sub = rospy.Subscriber('/xbotcore/joint_states', JointState, on_js_recv, queue_size=1)

rospy.Service('homing_legs_service', Trigger, action_service)
rospy.spin()