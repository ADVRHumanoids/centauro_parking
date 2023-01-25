from cartesian_interface.pyci_all import *
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
import numpy as np
import rospy
import rospkg
import argcomplete, argparse
from argcomplete.completers import EnvironCompleter
from pathlib import Path

file_dir = str(Path(__file__).parent.absolute())

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

def getError(ci, model):
    error = 0
    for task in ci.getTaskList():
        t = ci.getTask(task)
        actual = model.getPose(task)
        try:
            reference = t.getPoseReference()
        except AttributeError:
            continue

        pos_error = actual.translation[2] - reference[0].translation[2]

        error += np.linalg.norm(pos_error)
    return error

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

def cartesian_motion(qinit, qgoal, T, dt, ci):
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
        if time > T and np.linalg.norm(qdot) < 0.1:
            break
        rate.sleep()

rospy.init_node('contact_homing')

parser = argparse.ArgumentParser()
parser.add_argument("--park", action="store_true")
parser.add_argument("--unpark", action="store_true")
args = parser.parse_args()

parking = bool
if args.park:
    parking = True
    input(bcolors.OKGREEN + 'Start Parking: click to confirm')
elif args.unpark:
    parking = False
    input(bcolors.OKGREEN + 'Start Unparking: click to confirm')
else:
    print(bcolors.FAIL + "Missing mandatory argument '--park' '--unpark'")
    exit()

urdf_path = rospkg.RosPack().get_path('centauro_urdf') + '/urdf/centauro.urdf'
urdf = open(urdf_path, 'r').read()
rospy.set_param('/robot_description', urdf)

srdf_path = rospkg.RosPack().get_path('centauro_srdf') + '/srdf/centauro.srdf'
srdf = open(srdf_path, 'r').read()
rospy.set_param('/robot_description_semantic', srdf)

cfg = get_xbot_cfg(urdf, srdf)

try:
    robot = xbot.RobotInterface(cfg)
    robot.sense()
    robot.setControlMode(xbot.ControlMode.Position())
    ctrlmode = {'j_wheel_{}'.format(i+1) : xbot.ControlMode.Velocity() for i in range(4)}
    ctrlmode['neck_velodyne'] = xbot.ControlMode.Idle()
    ctrlmode['d435_head_joint'] = xbot.ControlMode.Idle()
    robot.setControlMode(ctrlmode)
except:
    print(bcolors.WARNING + 'RobotInterface not created' + bcolors.ENDC)
    robot = None

model = xbot.ModelInterface(cfg)
if robot is not None:
    qref = robot.getPositionReference()
    qref = [0., 0., 0., 0., 0., 0.] + qref.tolist()
    model.setJointPosition(qref)
    model.update()
else:
    contacts = ['wheel_' + str(i + 1) for i in range(4)]
    w_T_base = model.getPose('base_link')
    w_T_h = compute_h_frame(model, w_T_base, contacts)
    model.setFloatingBasePose(w_T_h.inverse() * w_T_base)
    if parking:
        qhome = model.getRobotState("home")
        model.setJointPosition(qhome)
        model.update()
    else:
        q = model.getRobotState("home")
        q = model.eigenToMap(q)
        q["hip_pitch_1"] = -1.57
        q["hip_pitch_2"] = 1.57
        q["hip_pitch_3"] = 1.57
        q["hip_pitch_4"] = -1.57
        q["knee_pitch_1"] = -2.41
        q["knee_pitch_2"] = 2.41
        q["knee_pitch_3"] = 2.41
        q["knee_pitch_4"] = -2.41
        q['ankle_pitch_1'] = -2.41
        q['ankle_pitch_2'] = 2.41
        q['ankle_pitch_3'] = 2.41
        q['ankle_pitch_4'] = -2.41
        q["ankle_yaw_1"] = 0.0
        q["ankle_yaw_2"] = 0.0
        q["ankle_yaw_3"] = 0.0
        q["ankle_yaw_4"] = 0.0
        model.setJointPosition(q)
        model.update()

rspub = pyci.RobotStatePublisher(model)
rspub.publishTransforms('park')

dt = 0.01
ikpb = open(file_dir + '/../config/centauro_parking_stack.yaml', 'r').read()
ci = pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)
postural = ci.getTask('Postural')

q1 = model.getJointPosition()
q1 = model.eigenToMap(q1)

q2 = list()

if parking:
    q2 = q1.copy()
    q2["hip_yaw_1"] = -0.75
    q2["hip_yaw_2"] = 0.75
    q2["hip_yaw_3"] = 0.75
    q2["hip_yaw_4"] = -0.75
    q2["hip_pitch_1"] = -1.57
    q2["hip_pitch_2"] = 1.57
    q2["hip_pitch_3"] = 1.57
    q2["hip_pitch_4"] = -1.57
    q2["knee_pitch_1"] = -2.41
    q2["knee_pitch_2"] = 2.41
    q2["knee_pitch_3"] = 2.41
    q2["knee_pitch_4"] = -2.41
    q2["ankle_pitch_1"] = -0.84
    q2["ankle_pitch_2"] = 0.84
    q2["ankle_pitch_3"] = 0.84
    q2["ankle_pitch_4"] = -0.84
    q2["ankle_yaw_1"] = 0.0
    q2["ankle_yaw_2"] = 0.0
    q2["ankle_yaw_3"] = 0.0
    q2["ankle_yaw_4"] = 0.0

    q3 = q2.copy()
    q3['ankle_pitch_1'] = -2.41
    q3['ankle_pitch_2'] = 2.41
    q3['ankle_pitch_3'] = 2.41
    q3['ankle_pitch_4'] = -2.41

else:
    q2 = q1.copy()
    q2["hip_yaw_1"] = -0.75
    q2["hip_yaw_2"] = 0.75
    q2["hip_yaw_3"] = 0.75
    q2["hip_yaw_4"] = -0.75
    q2["hip_pitch_1"] = -1.57
    q2["hip_pitch_2"] = 1.57
    q2["hip_pitch_3"] = 1.57
    q2["hip_pitch_4"] = -1.57
    q2["knee_pitch_1"] = -2.41
    q2["knee_pitch_2"] = 2.41
    q2["knee_pitch_3"] = 2.41
    q2["knee_pitch_4"] = -2.41
    q2["ankle_pitch_1"] = -0.84
    q2["ankle_pitch_2"] = 0.84
    q2["ankle_pitch_3"] = 0.84
    q2["ankle_pitch_4"] = -0.84
    q2["ankle_yaw_1"] = 0.0
    q2["ankle_yaw_2"] = 0.0
    q2["ankle_yaw_3"] = 0.0
    q2["ankle_yaw_4"] = 0.0

    q3 = model.getRobotState("home")
    q3 = model.eigenToMap(q3)

T = 5.0
rate = rospy.Rate(1./dt)

qinit = q1
qgoal = q2

cartesian_motion(q1, q2, T, dt, ci)

if parking:
    ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Disabled)
    ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Disabled)
    ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Disabled)
    ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Disabled)

qinit = q2
qgoal = q3

cartesian_motion(q2, q3, T, dt, ci)

