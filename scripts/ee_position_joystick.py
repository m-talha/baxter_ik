#!/usr/bin/env python
"""Baxter End Effector Controller Demo by Jackie Kay (jackie@osrfoundation.org)
Based on the Baxter RSDK Joint Position Example

Command the (x, y, z) position of the end effector using the joystick. Uses
Jacobian kinematics to determine required joint angles. 

"""

# TODO: refactor to use ik_command.py

import argparse
import rospy
import struct
import tf

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from baxter_pykdl import baxter_kinematics

import numpy as np
import math

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

try:
    import vrep
    import vrepConst
    from vrep_common.srv import *
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

# Convert the last 4 entries in q from quaternion form to Euler Angle form and copy into p
# Expect a 7x1 column vector and a preallocated 6x1 column vector (numpy)
# p and q are both pose vectors
def quaternion_to_euler(q, p):
    if q.shape != (7, 1) or p.shape != (6, 1):
        raise Exception("Got unexpected vector dimension in quaternion_to_euler")
    p[0:3] = q[0:3]
    p[3], p[4], p[5] = tf.transformations.euler_from_quaternion(q[3:7].flatten().tolist())

def rad_to_deg(r):
    deg = np.zeros(r.size)
    for i in range(r.size):
        deg[i] = math.degrees(r[i])
    return deg

# Get key commands from the user and move the end effector
def map_joystick(joystick):
# def map_joystick(joystick, pose_pub):
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)

    orient = False
    #abbreviations
    jhi = lambda s: joystick.stick_value(s) > 0
    jlo = lambda s: joystick.stick_value(s) < 0
    bdn = joystick.button_down
    bup = joystick.button_up
    ori = lambda v: orient if v=='up' else not orient
    zeros = [0]*4
    inc = 0.01
    syncErrThresh = 0.03
    syncErrFactor = 5
    syncErrCount = 0
    idleInput = True
    vrepZdelta = 1.08220972

    #These are from PyKDL and are needed for the Jacobian
    left_kin = baxter_kinematics('left')
    right_kin = baxter_kinematics('right')

    #Connect with IK service
    right_ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    right_iksvc = rospy.ServiceProxy(right_ns, SolvePositionIK)
    left_ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    left_iksvc = rospy.ServiceProxy(left_ns, SolvePositionIK)
    print('')

    # Service to subscribe to desired pose from V-rep
    # objHndCall = rospy.ServiceProxy('vrep/simRosGetObjectHandle', simRosGetObjectHandle)
    # resp = objHndCall.call('Baxter_leftArm_target')
    # ikTarget = resp.handle

    # Service for V-rep to subscribe to desired pose
    # poseSubCall = rospy.ServiceProxy('vrep/simRosEnableSubscriber', simRosEnableSubscriber)
    # resp = poseSubCall.call('/ik_baxter/d_pose',1,10243,ikTarget,-1,'')

    # Service to call functions in V-rep (e.g. set/get joint positions)
    vrepScriptFunc = rospy.ServiceProxy('vrep/simRosCallScriptFunction', simRosCallScriptFunction)

    def syncPos(limb,jntSrvCall):
        print('Synchronising pose...')
        while (True):
            # Set initial joint position in V-rep
            curr_jnts = np.array([limb.joint_angles()[name] for name in limb.joint_names()])
            # print curr_jnts
            resp = jntSrvCall.call('set_jnt_pos@Baxter_leftArm_target',1,[],curr_jnts,[],'')
            resp = jntSrvCall.call('get_jnt_pos@Baxter_leftArm_target',1,[],[],[],'')
            err = math.fabs(np.linalg.norm(np.array(resp.outputFloats)-curr_jnts))
            print 'Error: ',err
            vrep_deg = rad_to_deg(np.array(resp.outputFloats))
            print 'V-rep Joints: ',vrep_deg
            print '---------------------------------------------------------------------------------'
            curr_deg = rad_to_deg(curr_jnts)
            print 'Actual joints: ', curr_jnts
            print '---------------------------------------------------------------------------------'
            if err < abs(syncErrThresh):
                print('Pose synchronised')
                break

    def command_jacobian(side, direction):
        if side == 'left':
            limb = left
            kin = left_kin
        elif side == 'right':
            limb = right
            kin = right_kin
        else:
            raise Exception("Got wrong side in command_jacobian")

        # current is the current position of end effector
        # We need to reshape it from quaternion to Euler Angle
        current_q = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation'])
        current_q = current_q.reshape((7, 1))
        current = np.zeros((6, 1))
        quaternion_to_euler(current_q, current)

        # We need to convert the direction from the world frame to the hand frame
        # Rotate direction by the inverse of the rotation matrix from the angular pose in current
        R = tf.transformations.quaternion_matrix(current_q[3:7].flatten().tolist())
        R_inv = tf.transformations.inverse_matrix(R)
        print R_inv[0:3, 0:3]
        direction = np.array(direction).reshape((6, 1))
        print direction[0:3]
        dir_rot = np.dot(R_inv[0:3, 0:3], direction[0:3])
        print dir_rot
        direction[0:3] = dir_rot

        # Goal is the goal position, found by adding the requested direction from the user
        goal = current + direction
        current_angles = np.array([limb.joint_angles()[name] for name in limb.joint_names()])

        # Get the Jacobian inverse and solve for the necessary change in joint angles
        jacobian_inv = kin.jacobian_transpose()
        delta = jacobian_inv*(goal-current)
        commands = current_angles.flatten() + delta.flatten()

        # Command the joints
        command_list = commands.tolist()
        joint_command = dict(zip(limb.joint_names(), command_list[0]))
        limb.set_joint_positions(joint_command)

    def command_ik(side, direction):
        """Use the Rethink IK service to figure out a desired joint position
           This is way too slow to use in realtime. Unless I figure out why the
           IK service takes minutes to respond, not going to use it."""
        if side == 'left':
            limb = left
            iksvc = left_iksvc
            ns = left_ns
        else:
            limb = right
            iksvc = right_iksvc
            ns = right_ns

        # desired_p = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation'])
        resp = vrepScriptFunc.call('get_pose@Baxter_leftArm_target',1,[],[],[],'')
        curr = np.array(resp.outputFloats[3:])

        if orient == True:
            # print(temp[3:])
            direction = [x*500*math.pi/180 for x in direction]
            inc = tf.transformations.quaternion_from_euler(direction[1],direction[0],direction[2])
            print('Inc: ', inc)
            # curr_euler = tf.transformations.euler_from_quaternion(curr)
            # print('Current Euler: ',curr_euler)
            # print('Current Quarternion: ', curr)
            # mag = math.sqrt(np.sum(curr**2))
            # print('Current magnitude: ', mag)
            new_quar = tf.transformations.quaternion_multiply(inc,curr)
            # print('New quaternion: ',new_quar)
            # mag = math.sqrt(np.sum(new_quar**2))
            # print('New magnitude: ', mag)
            # new_euler = tf.transformations.euler_from_quaternion(new_quar)
            # print('New Euler: ',new_euler)
            direction = [0]*3 + new_quar.tolist()
            # print direction

        else:
            direction = direction+curr.tolist()

        # print('Current: ', temp)
        direction = np.array(direction)
        # print('Direction: ', direction)
        resp =vrepScriptFunc.call('set_pose@Baxter_leftArm_target',1,[],direction,[],'')
        desired_p = temp + direction
        # print('Desired: ', desired_p)

        return desired_p

    def pub_desired_pose(desired_p):
        # Publish desired position so V-rep can obtain it
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = PoseStamped(
                    header = hdr,
                    pose = Pose(position=Point(x=desired_p[0], y=desired_p[1], z=desired_p[2]+1.082),
                    orientation = Quaternion(x=desired_p[3], y=desired_p[4], z=desired_p[5], w=desired_p[6]))
                )
        # pose_pub.publish(pose)

    def pos_to_ori(s,ori):
        ori = s=='down'
        return ori

    def grip_l_open():
        grip_left.open()
        vrepScriptFunc.call('grip@BaxterGripper',1,[0],[],[],'')

    def grip_l_close():
        grip_left.close()
        vrepScriptFunc.call('grip@BaxterGripper',1,[1],[],[],'')

    def reset_pos():
        outputFloats = [-0.523599,-1.22173,0.0,2.44346,0.0,0.523599,0.0]
        limb_joints = dict(zip(left._joint_names['left'], outputFloats))
        left.move_to_joint_positions(limb_joints)

    def print_help(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s: %s" % (str(test[1][0]), doc))

    bindings_list = []
    bindings = (
        ((bdn, ['leftTrigger']),
         (grip_l_close,  []), "left gripper close"),
        ((bup, ['leftTrigger']),
         (grip_l_open,   []), "left gripper open"),
        ((bdn, ['leftBumper']),
         (pos_to_ori,['down', orient]), "Switched to orientation"),
        ((bup, ['leftBumper']),
         (pos_to_ori,['up', orient]), "Switched to position"),
        ((jlo, ['leftStickHorz']),
         (command_ik, ['left', [0, -inc, 0]]), lambda: "left y dec "),
        ((jhi, ['leftStickHorz']),
         (command_ik, ['left', [0, inc, 0]]), lambda: "left y inc "),
        ((jlo, ['leftStickVert']),
         (command_ik, ['left', [-inc, 0, 0]]), lambda: "left x dec "),
        ((jhi, ['leftStickVert']),
         (command_ik, ['left', [inc, 0, 0]]), lambda: "left x inc "),
        ((jlo, ['rightStickVert']),
         (command_ik, ['left', [0, 0, -inc]]), lambda: "left z dec "),
        ((jhi, ['rightStickVert']),
         (command_ik, ['left', [0, 0, inc]]), lambda: "left z inc "),
        # ((bdn, ['leftBumper']),
        #  (grip_left.calibrate, []), "left calibrate"),
        # ((bdn, ['rightBumper']),
        #  (grip_right.calibrate, []), "right calibrate"),
        ((bdn, ['function1']),
         (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']),
         (print_help, [bindings_list]), "help"),
        )
    bindings_list.append(bindings)

    reset_pos()
    # Initial synchronisation of Baxter and V-rep
    print('Synchronising pose...')
    syncPos(left,vrepScriptFunc)
    print('Pose synchronised')

    current_p = desired_p = np.array(left.endpoint_pose()['position']+left.endpoint_pose()['orientation'])

    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press Ctrl-C to stop. ")
    while not rospy.is_shutdown():
        temp = current_p
        idleInput = True
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                if cmd[0] == pos_to_ori:
                    orient = cmd[0](*cmd[1])
                elif cmd[0] == command_ik:
                    idleInput = False
                    temp = cmd[0](*cmd[1])
                else:
                    cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)

        # desired_p = temp
        # err = math.fabs(np.linalg.norm(desired_p-current_p))
        # print 'Error: ',err
        # if err > 0.001:
            # pub_desired_pose(current_p)
        # pub_desired_pose(desired_p)
            # print 'Current before: ', current_p
            # current_p = np.array(left.endpoint_pose()['position']+left.endpoint_pose()['orientation'])
            # print 'Current after: ', current_p

        resp = vrepScriptFunc.call('ikSuccess@Baxter_leftArm_target',1,[],[],[],'')
        print 'Ik Success: ', resp.outputInts

        if resp.outputInts[0] == -1:
            # Get resulting joint positions from V-rep
            resp = vrepScriptFunc.call('get_jnt_pos@Baxter_leftArm_target',1,[],[],[],'')

            # Get Baxter joints
            curr_jnts = np.array([left.joint_angles()[name] for name in left.joint_names()])

        # if err > syncErrThresh*syncErrFactor: # or syncErrCount > 4:
        #     print('Resynchronising....')
        #     syncPos(left, vrepScriptFunc)
        #     syncErrCount = 0
        # elif err > syncErrThresh:
        #     if idleInput == True:
        #         print('Incrementing sync error')
        #         syncErrCount += 1
        #     else:
        #         print('Resetting sync error')
        #         syncErrCount = 0
        #
        #     print('Sync error count: ',syncErrCount)
        #     print('Moving to position...')
        #     # Wrap and send joint positions to Baxter
        #     # limb_joints = dict(zip(left._joint_names['left'], resp.outputFloats))
        #     # left.set_joint_positions(limb_joints)
        #     current_p = np.array(left.endpoint_pose()['position']+left.endpoint_pose()['orientation'])






            # Compare joints
            norm = np.linalg.norm(curr_jnts)
            err = math.fabs(np.linalg.norm(np.array(resp.outputFloats)-curr_jnts))
            err = err/norm
            print('Error: ',err)

            # limb_joints = dict(zip(left._joint_names['left'], resp.outputFloats))
            # left.set_joint_positions(limb_joints)
            # syncPos(left, vrepScriptFunc)

            if err > abs(syncErrThresh):
                # Wrap and send joint positions to Baxter
                limb_joints = dict(zip(left._joint_names['left'], resp.outputFloats))
                left.set_joint_positions(limb_joints)
                # current_p = np.array(left.endpoint_pose()['position']+left.endpoint_pose()['orientation'])

                # print('V-rep Joints: ',limb_joints)
            # resp = vrepScriptFunc.call('get_pose@Baxter_leftArm_target',1,[],[],[],'')
            # print 'V-rep pose: ',resp.outputFloats
            # print('---------------------------------------------------------------------------------'/0)
            # print 'Finish pose: ',current_p
            # print('Actual joints: ', left.joint_angles())
            # print('---------------------------------------------------------------------------------')

        """if len(lcmd):
            left.set_joint_positions(lcmd)
            lcmd.clear()
        if len(rcmd):
            right.set_joint_positions(rcmd)
            rcmd.clear()"""
        rate.sleep()

    return False

def main():
    """RSDK Joint Position Example: Joystick Control

    Use a game controller to control the angular joint positions
    of Baxter's arms.

    Attach a game controller to your dev machine and run this
    example along with the ROS joy_node to control the position
    of each joint in Baxter's arms using the joysticks. Be sure to
    provide your *joystick* type to setup appropriate key mappings.

    Each stick axis maps to a joint angle; which joints are currently
    controlled can be incremented by using the mapped increment buttons.
    Ex:
      (x,y -> e0,e1) >>increment>> (x,y -> e1,e2)
    """
    epilog = """
See help inside the example with the "Start" button for controller
key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--joystick', required=True,
        choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    joystick = None
    if args.joystick == 'xbox':
        joystick = baxter_external_devices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = baxter_external_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = baxter_external_devices.joystick.PS3Controller()
    else:
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_joystick")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    vrepSimStart = rospy.ServiceProxy('vrep/simRosStartSimulation',simRosStartSimulation)
    vrepSimStop = rospy.ServiceProxy('vrep/simRosStopSimulation',simRosStopSimulation)

    def clean_shutdown():
        print("\nExiting example.")
        vrepSimStop.call()
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot... ")
    rs.enable()

    vrepSimStart.call()
    # pose_pub = rospy.Publisher("/ik_baxter/d_pose", PoseStamped,queue_size=1)
    # map_joystick(joystick,pose_pub)
    map_joystick(joystick)
    print("Done.")

if __name__ == '__main__':
    main()

