from cartesian_interface.pyci_all import *
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
import numpy as np
import rospy
import rospkg
import os

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

rospy.init_node('contact_homing')

urdf_path = rospkg.RosPack().get_path('centauro_urdf') + '/urdf/centauro.urdf'
urdf = open(urdf_path, 'r').read()
rospy.set_param('/robot_description', urdf)

srdf_path = rospkg.RosPack().get_path('centauro_srdf') + '/srdf/centauro.srdf'
srdf = open(srdf_path, 'r').read()
rospy.set_param('/robot_description_semantic', srdf)

cfg = get_xbot_cfg(urdf,
                   srdf)

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
    model.syncFrom(robot)
    model.update()
rspub = pyci.RobotStatePublisher(model)
rspub.publishTransforms('ci')

contacts = ['wheel_' + str(i+1) for i in range(4)]
w_T_base = model.getPose('base_link')
w_T_h = compute_h_frame(model, w_T_base, contacts)

model.setFloatingBasePose(w_T_h.inverse() * w_T_base)
model.update()
rspub.publishTransforms('ci')
print(model.getPose('base_link'))

dt = 0.01
ikpb = open(file_dir + '/../config/centauro_parking_stack.yaml', 'r').read()
ci = pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)
postural = ci.getTask('Postural')

q1 = model.getRobotState('home')
model.setJointPosition(q1)
model.update()
q1 = model.eigenToMap(q1)

q2 = { "j_arm1_1": 0.520149,
       "j_arm1_2": 0.320865,
       "j_arm1_3": 0.274669,
       "j_arm1_4": -2.23604,
       "j_arm1_5": 0.0500815,
       "j_arm1_6": -0.781461,
       "j_arm2_1": 0.520149,
       "j_arm2_2": -0.320865,
       "j_arm2_3": -0.274669,
       "j_arm2_4": -2.23604,
       "j_arm2_5": -0.050081,
       "j_arm2_6": -0.781461,
       "hip_yaw_1": -0.75,
       "hip_yaw_2": 0.75,
       "hip_yaw_3": 0.75,
       "hip_yaw_4": -0.75,
       "hip_pitch_1": -1.57,
       "hip_pitch_2": 1.57,
       "hip_pitch_3": 1.57,
       "hip_pitch_4": -1.57,
       "knee_pitch_1": -2.41,
       "knee_pitch_2": 2.41,
       "knee_pitch_3": 2.41,
       "knee_pitch_4": -2.41,
       "ankle_pitch_1": -0.84,
       "ankle_pitch_2": 0.84,
       "ankle_pitch_3": 0.84,
       "ankle_pitch_4": -0.84,
       "ankle_yaw_1": 0.0,
       "ankle_yaw_2": 0.0,
       "ankle_yaw_3": 0.0,
       "ankle_yaw_4": 0.0,
       "d435_head_joint": 0.0,
       "torso_yaw": 0.0,
       "velodyne_joint": 0.0
    }

q3 = q2.copy()
q3['ankle_pitch_1'] = -2.41
q3['ankle_pitch_2'] = 2.41
q3['ankle_pitch_3'] = 2.41
q3['ankle_pitch_4'] = -2.41

T = 5.0
rate = rospy.Rate(1./dt)
time = 0

qinit = q1
qgoal = q2

for i in range(int(T/dt)):
    tau = time / T
    alpha = quintic(tau)
    qref = model.mapToEigen(qinit)*(1 - alpha) + model.mapToEigen(qgoal)*alpha
    qref = model.eigenToMap(qref)
    postural.setReferencePosture(qref)

    q, qdot = update_ik(ci, model, time, dt)
    rspub.publishTransforms('')

    if robot is not None:
        robot.setPositionReference(qref)
        robot.setVelocityReference(model.eigenToMap(qdot))
        robot.move()

    time += dt
    rate.sleep()

time = 0

qinit = q2
qgoal = q3
ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Disabled)
ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Disabled)
ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Disabled)
ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Disabled)

for i in range(int(T/dt)):
    tau = time / T
    alpha = quintic(tau)
    qref = model.mapToEigen(qinit)*(1 - alpha) + model.mapToEigen(qgoal)*alpha
    qref = model.eigenToMap(qref)
    postural.setReferencePosture(qref)

    q, qdot = update_ik(ci, model, time, dt)
    rspub.publishTransforms('')

    if robot is not None:
        robot.setPositionReference(qref)
        robot.setVelocityReference(model.eigenToMap(qdot))
        robot.move()

    time += dt
    rate.sleep()

input('click to init stand up')
time = 0
qinit = q3
qgoal = q2

for i in range(int(T/dt)):
    tau = time / T
    alpha = quintic(tau)
    qref = model.mapToEigen(qinit)*(1 - alpha) + model.mapToEigen(qgoal)*alpha
    qref = model.eigenToMap(qref)
    postural.setReferencePosture(qref)

    q, qdot = update_ik(ci, model, time, dt)
    rspub.publishTransforms('')

    if robot is not None:
        robot.setPositionReference(qref)
        robot.setVelocityReference(model.eigenToMap(qdot))
        robot.move()

    time += dt
    rate.sleep()

time = 0
qinit = q2
qgoal = q1
ci.getTask("steering_wheel_1").setActivationState(pyci.ActivationState.Enabled)
ci.getTask("steering_wheel_2").setActivationState(pyci.ActivationState.Enabled)
ci.getTask("steering_wheel_3").setActivationState(pyci.ActivationState.Enabled)
ci.getTask("steering_wheel_4").setActivationState(pyci.ActivationState.Enabled)

for i in range(int(T/dt)):
    tau = time / T
    alpha = quintic(tau)
    qref = model.mapToEigen(qinit)*(1 - alpha) + model.mapToEigen(qgoal)*alpha
    qref = model.eigenToMap(qref)
    postural.setReferencePosture(qref)

    q, qdot = update_ik(ci, model, time, dt)
    rspub.publishTransforms('')

    if robot is not None:
        robot.setPositionReference(qref)
        robot.setVelocityReference(model.eigenToMap(qdot))
        robot.move()

    time += dt
    rate.sleep()

