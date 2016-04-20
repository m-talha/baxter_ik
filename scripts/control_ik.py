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
import thread
import os, subprocess, signal

import baxter_interface
import baxter_external_devices
import joystick as jstick

from baxter_interface import CHECK_VERSION

import numpy as np
import math

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

import rosbag
from baxter_core_msgs.msg import (
    EndpointState,
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

def kb_bindings():
    kb_bindings = {
        'o': (pos_to_ori, ['down', orient], "Switched to orientation"),
        'p': (pos_to_ori, ['up', orient], "Switched to position"),
        'w': (command_ik, ['left', [inc, 0, 0]],  "increase left x"),
        's': (command_ik, ['left', [-inc, 0, 0]], "decrease left x"),
        'a': (command_ik, ['left', [0, inc, 0]],  "increase left y"),
        'd': (command_ik, ['left', [0, -inc, 0]], "decrease left y"),
        'q': (command_ik, ['left', [0, 0, inc]],  "increase left z"),
        'e': (command_ik, ['left', [0, 0, -inc]], "decrease left z"),
        'k': (grip_l_close, [], "left: gripper close"),
        'l': (grip_l_open, [], "left: gripper open"),
        # 'c': (grip_right.calibrate, [], "left: gripper calibrate")
    }

    return kb_bindings

def print_kb_bindings(kb_bindings):
    print("key bindings: ")
    print("  Esc: Quit")
    print("  ?: Help")
    for key, val in sorted(kb_bindings.items(),
                           key=lambda x: x[1][2]):
        print("  %s: %s" % (key, val[2]))
    print("Press Ctrl-C to stop. ")

def js_bindings():
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
        ((bdn, ['function1']),
         (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']),
         (print_help, [bindings_list]), "help"),
        )
    bindings_list.append(bindings)

    return bindings_list

def print_js_bindings(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s: %s" % (str(test[1][0]), doc))

class RobotArm:

    def __init__(self,vrepScriptFunc):
        self.tf = tf.TransformListener()
        self.left = baxter_interface.Limb('left')
        self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.grip_closed = False
        self.orient = False
        self.ee_mode = False
        self.vrepScriptFunc = vrepScriptFunc
        self.currPoseL = self.temp = np.array(self.left.endpoint_pose()['position']+self.left.endpoint_pose()['orientation'])

    def syncPos(self, side):
        if side == 'left':
            limb = self.left

        # Set initial joint position in V-rep
        curr_jnts = np.array([limb.joint_angles()[name] for name in limb.joint_names()])
        # print curr_jnts
        resp = self.vrepScriptFunc.call('set_jnt_pos@Baxter_leftArm_target',1,[],curr_jnts,[],'')
        resp = self.vrepScriptFunc.call('get_jnt_pos@Baxter_leftArm_target',1,[],[],[],'')
        err = math.fabs(np.linalg.norm(np.array(resp.outputFloats)-curr_jnts))
        print 'Error: ',err
        print 'V-rep Joints: ',resp.outputFloats
        print '---------------------------------------------------------------------------------'
        curr_deg = rad_to_deg(curr_jnts)
        print 'Actual joints: ', curr_jnts
        print '---------------------------------------------------------------------------------'

        return err

    def updPos(self,side):
        if side == 'left':
            limb = self.left
        else:
            limb = self.right

        self.currPoseL = self.temp = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation'])
        return self.currPoseL

    def command_ik(self, side, direction):
        """Use the Rethink IK service to figure out a desired joint position
           This is way too slow to use in realtime. Unless I figure out why the
           IK service takes minutes to respond, not going to use it."""
        if side == 'left':
            limb = self.left
        else:
            limb = self.right


        if self.ee_mode == True:
            resp = self.vrepScriptFunc.call('get_pose_ee@Baxter_leftArm_target',1,[],[],[],'')
            currV = np.array(resp.outputFloats)
            curr_ang = currV[3:]

            # print('V-rep pose: ', currV)

            t = self.tf.getLatestCommonTime("/left_gripper_base", "/base")
            position, quaternion = self.tf.lookupTransform("/left_gripper_base", "/base", t)
            currB = np.array(position).tolist() + np.array(quaternion).tolist()

            # print('Baxter pose: ', currB)

            diff = currB-currV
            # print('Difference :', diff)

            resp = self.vrepScriptFunc.call('get_target_ee@Baxter_leftArm_target',1,[],[],[],'')
            currV = np.array(resp.outputFloats)
            curr_ang = currV[3:]
            actV = currV # + diff

            # Transform from quaternion to Euler
            actV = actV.reshape((7,1))
            act_eul = np.zeros((6,1))
            quaternion_to_euler(actV,act_eul)

            # Get inverse rotation matrix from quaternion
            rot = tf.transformations.quaternion_matrix(actV[3:7].flatten().tolist())
            rot_inv = tf.transformations.inverse_matrix(rot)
            print('R_inv partial:', rot_inv[0:3,0:3])

            # Compute the direction desired
            if self.orient == True:
                direction = [x*50*math.pi/180 for x in direction]
                inc = tf.transformations.quaternion_from_euler(direction[1],direction[0],direction[2])
                print('Inc: ', inc)
                new_quar = tf.transformations.quaternion_multiply(inc,curr_ang)
                direction = [0]*3 + new_quar.tolist()

            else:
                direction = direction+curr_ang.tolist()

            # Convert direction from world to ee frame- currently just position
            # dir_ee = np.dot(rot_inv[0:3,0:3], direction[0:3])
            # print('Direction_EE: ', dir_ee)
            # direction[0:3] = dir_ee
            # print('Direction: ', direction)

            # Send the new direction to V-rep
            resp =self.vrepScriptFunc.call('set_target_ee@Baxter_leftArm_target',1,[],direction,[],'')
            desired_p = self.temp + direction
            self.temp = desired_p


        else:
            # desired_p = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation'])
            resp = self.vrepScriptFunc.call('get_pose@Baxter_leftArm_target',1,[],[],[],'')
            curr_ang = np.array(resp.outputFloats[3:])

            if self.orient == True:
                # print(temp[3:])
                direction = [x*500*math.pi/180 for x in direction]
                inc = tf.transformations.quaternion_from_euler(direction[1],direction[0],direction[2])
                print('Inc: ', inc)
                # curr_euler = tf.transformations.euler_from_quaternion(curr)
                # print('Current Euler: ',curr_euler)
                # print('Current Quarternion: ', curr)
                # mag = math.sqrt(np.sum(curr**2))
                # print('Current magnitude: ', mag)
                new_quar = tf.transformations.quaternion_multiply(inc,curr_ang)
                # print('New quaternion: ',new_quar)
                # mag = math.sqrt(np.sum(new_quar**2))
                # print('New magnitude: ', mag)
                # new_euler = tf.transformations.euler_from_quaternion(new_quar)
                # print('New Euler: ',new_euler)
                direction = [0]*3 + new_quar.tolist()
                # print direction

            else:
                direction = direction+curr_ang.tolist()

            # print('Current: ', self.temp)
            dirSection = np.array(direction)
            # print('Direction: ', direction)
            resp =self.vrepScriptFunc.call('set_pose@Baxter_leftArm_target',1,[],direction,[],'')
            desired_p = self.temp + direction
            self.temp = desired_p
            # print('Desired: ', desired_p)

    def grip_l_open(self):
        self.grip_left.open()
        self.vrepScriptFunc.call('grip@BaxterGripper',1,[0],[],[],'')

    def grip_l_close(self):
        if self.grip_closed == False:
            self.grip_left.close()
            self.vrepScriptFunc.call('grip@BaxterGripper',1,[1],[],[],'')
            self.grip_closed = True
            print('Gripper closed')
        else:
            self.grip_left.open()
            self.vrepScriptFunc.call('grip@BaxterGripper', 1, [0], [], [], '')
            self.grip_closed = False
            print('Gripper open')

    def pos_to_ori(self,s=None):
        if s:
            self.orient = s=='down'
        else:
            self.orient = not self.orient

        if self.orient == True:
            print('In orientation mode')
        else:
            print('In position mode')

    def world_to_ee(self,s=None):
        if s:
            self.ee_mode = s=='ee'
        else:
            self.ee_mode = not self.ee_mode

        if self.ee_mode == True:
            print('In tool frame')
        else:
            print('In world frame')

    def reset_pos(self):
        outputFloats = [-0.523599,-1.22173,0.0,2.44346,0.0,0.523599,0.0]
        # outputFloats = [0,0,0.0,0,0.0,0,0.0]
        limb_joints = dict(zip(self.left._joint_names['left'], outputFloats))
        self.left.move_to_joint_positions(limb_joints)

def move_to_start_pos(robot,syncErrThresh):
    print('Moving to start position and synchronising pose...')
    robot.reset_pos()
    # Initial synchronisation of Baxter and V-rep
    while (True):
        err = robot.syncPos('left')
        if err < abs(syncErrThresh):
            break


    print('Pose synchronised. Control of arm enabled.')

def inc_delta(inc, delta, incU, incL):
    print('Delta: ', delta)
    if delta > 0:
        if inc+delta <= incU:
            inc+= delta
        else:
            inc = incU
    if delta < 0:
        if inc+delta >= incL:
            inc+= delta
        else:
            inc = incL
    print('New speed: ', inc)

    return inc

def run_control(joystick):
    # Initialise parameters
    kbOnly = True if joystick==None else False
    print('kbOnly: ', kbOnly)

    #abbreviations
    if kbOnly == False:
        jhi = lambda s: joystick.stick_value(s) > 0
        jlo = lambda s: joystick.stick_value(s) < 0
        bdn = joystick.button_down
        bup = joystick.button_up

    zeros = [0]*4
    initial_inc = 0.03
    inc = initial_inc
    delta = 0.01
    incU = 0.05
    incL = 0.01
    syncErrThresh = 0.03
    vrepZdelta = 1.08220972

    # Service to call functions in V-rep (e.g. set/get joint positions)
    vrepScriptFunc = rospy.ServiceProxy('vrep/simRosCallScriptFunction', simRosCallScriptFunction)

    # Get robot interface
    robot = RobotArm(vrepScriptFunc)
    robot.updPos('left')

    # Get appropriate bindings
    kb_b= js_b = False

    def gen_js_bindings(inc):
        bindings = (
            ((bdn, ['btnUp']),
             (robot.grip_l_close, []), "left gripper"),
            ((jhi, ['rightTrigger']),
             (inc_delta, [inc, delta, incU, incL]), "Speed increased"),
            ((jhi, ['leftTrigger']),
             (inc_delta, [inc, -delta, incU, incL]), "Speed decreased"),
            ((bdn, ['leftBumper']),
             (robot.pos_to_ori, ['down']), "Switched to orientation"),
            ((bup, ['leftBumper']),
             (robot.pos_to_ori, ['up']), "Switched to position"),
            ((bdn, ['rightBumper']),
             (robot.world_to_ee, ['ee']), "Switched to tool frame"),
            ((bup, ['rightBumper']),
             (robot.world_to_ee, ['world']), "Switched to world frame"),
            ((jlo, ['leftStickHorz']),
             (robot.command_ik, ['left', [0, -inc, 0]]), lambda: "left y dec "),
            ((jhi, ['leftStickHorz']),
             (robot.command_ik, ['left', [0, inc, 0]]), lambda: "left y inc "),
            ((jlo, ['leftStickVert']),
             (robot.command_ik, ['left', [-inc, 0, 0]]), lambda: "left x dec "),
            ((jhi, ['leftStickVert']),
             (robot.command_ik, ['left', [inc, 0, 0]]), lambda: "left x inc "),
            ((jlo, ['rightStickVert']),
             (robot.command_ik, ['left', [0, 0, -inc]]), lambda: "left z dec "),
            ((jhi, ['rightStickVert']),
             (robot.command_ik, ['left', [0, 0, inc]]), lambda: "left z inc "),
            ((bdn, ['function1']),
             (print_js_bindings, [js_b]), "help"),
            ((bdn, ['function2']),
             (print_js_bindings, [js_b]), "help"),
        )

        return bindings
    def gen_kb_bindings(inc):
        bindings = {
            '9': (inc_delta, [inc, delta, incU, incL], "Speed increased"),
            '0': (inc_delta, [inc, -delta, incU, incL], "Speed decreased"),
            'i': (robot.pos_to_ori, [], "Switch position/orientation"),
            # 'p': (robot.pos_to_ori, ['up'], "Switched to position"),
            'o': (robot.world_to_ee, [], "Switch world/tool frame"),
            # 'i': (robot.world_to_ee, ['world'], "Switched to world frame"),
            'w': (robot.command_ik, ['left', [inc, 0, 0]], "increase left x"),
            's': (robot.command_ik, ['left', [-inc, 0, 0]], "decrease left x"),
            'a': (robot.command_ik, ['left', [0, inc, 0]], "increase left y"),
            'd': (robot.command_ik, ['left', [0, -inc, 0]], "decrease left y"),
            'q': (robot.command_ik, ['left', [0, 0, inc]], "increase left z"),
            'e': (robot.command_ik, ['left', [0, 0, -inc]], "decrease left z"),
            'p': (robot.grip_l_close, [], "Gripper open/close"),
            # 'l': (robot.grip_l_open, [], "left: gripper open"),
            # 'c': (grip_left.calibrate, [], "left: gripper calibrate")
        }

        return bindings

    if kbOnly == False:
        js_b = []
        bindings = gen_js_bindings(inc)
        js_b.append(bindings)

    kb_b = gen_kb_bindings(inc)

    # Print bindings
    if kbOnly == False:
        print_js_bindings(js_b)
    else:
        print_kb_bindings(kb_b)



    # Setup data recording
    endpoint_state=EndpointState()
    def on_endpoint_states(msg,endpoint_state):
        endpoint_state.header= msg.header
        endpoint_state.pose= msg.pose
        endpoint_state.twist= msg.twist
        endpoint_state.wrench= msg.wrench

    recording=False
    done_recording=False
    def record_data(filename):
        # Subscribe to endpoint state
        # _cartesian_state_sub = rospy.Subscriber(
        #         '/robot/limb/left/endpoint_state',
        #         EndpointState,
        #         on_endpoint_states,
        #         endpoint_state,
        #         queue_size=1,
        #         tcp_nodelay=True)

        # bag = rosbag.Bag(filename, 'w')

        def terminate_process_and_children(s):
            list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
            list_output = list_cmd.stdout.read()
            retcode = list_cmd.wait()
            assert retcode == 0, "List command returned %d" % retcode
            for str in list_output.split("\n"):
                if (str.startswith(s)):
                    os.system("rosnode kill " + str)

        # while not done_recording:
            # if recording==True:
                # print('Endpoint actual: ',endpoint_state)
                # bag.write('endpoint_state',endpoint_state)

        rt = rospy.Rate(100)
        while not recording:
            rt.sleep()

        p = subprocess.Popen('rosbag record -j -O test.bag /robot/limb/left/endpoint_state '
                                     '/cameras/head_camera/image',stdin=subprocess.PIPE,shell=True,
                                     cwd='/home/talha/test_ws/src/baxter_ik/scripts/')
        # print 'Finishing recording'
        # bag.close()
        while not done_recording:
            rt.sleep()

        print 'Finishing recording'
        terminate_process_and_children("/record")


    thread.start_new_thread(record_data, ('test.bag',))



    # Move to start position
    move_to_start_pos(robot,syncErrThresh)
    vrepScriptFunc.call('set_ik_mode@Baxter_leftArm_target',1,[],[],[],'')

    rate = rospy.Rate(100)
    # while loop
    while not rospy.is_shutdown():

        # inputControl()
        if kbOnly == True:
            c = baxter_external_devices.getch(-1)
            if c:
                print('Pressed: ',c)
                recording=True
                #catch Esc or ctrl-c
                if c in ['\x1b', '\x03']:
                    # terminate_process_and_children(p)
                    rospy.signal_shutdown("Example finished.")

                if c == '\r':
                    done_recording=True

                # Keyboard control
                if c in kb_b:
                    cmd = kb_b[c]
                    if cmd[0] == inc_delta:
                        inc = cmd[0](*cmd[1])
                        kb_b = gen_kb_bindings(inc)
                    elif cmd[0] == robot.pos_to_ori:
                        kb_b = gen_kb_bindings(initial_inc)
                        cmd[0](*cmd[1])
                    else:
                        cmd[0](*cmd[1])
                    print("command: %s" % (cmd[2],))

        else:
            c = baxter_external_devices.getch()
            if c == '\r':
                done_recording=True

            # Joystick control
            for (test, cmd, doc) in bindings:
                if test[0](*test[1]):
                    recording=True
                    if cmd[0] == inc_delta:
                        inc = cmd[0](*cmd[1])
                        bindings = gen_js_bindings(inc)
                        js_b[0] = bindings
                    elif cmd[0] == robot.pos_to_ori:
                        bindings = gen_js_bindings(initial_inc)
                        js_b[0] = bindings
                        cmd[0](*cmd[1])
                    else:
                        cmd[0](*cmd[1])
                    if callable(doc):
                        print(doc())
                    else:
                        print(doc)

        # Check if IK was a success in V-rep
        resp = vrepScriptFunc.call('ikSuccess@Baxter_leftArm_target',1,[],[],[],'')
        # print 'Ik Success: ', resp.outputInts

        # If successful, receive & send joints to Baxter
        if resp.outputInts[0] == -1:
            # Get resulting joint positions from V-rep
            resp = vrepScriptFunc.call('get_jnt_pos@Baxter_leftArm_target',1,[],[],[],'')

            # Get Baxter joints
            curr_jnts = np.array([robot.left.joint_angles()[name] for name in robot.left.joint_names()])

            # Compare joints
            norm = np.linalg.norm(curr_jnts)
            err = math.fabs(np.linalg.norm(np.array(resp.outputFloats)-curr_jnts))
            err = err/norm
            # print('Error: ',err)

            # limb_joints = dict(zip(robot.left._joint_names['left'], resp.outputFloats))
            # robot.left.set_joint_positions(limb_joints)
            # syncPos(left, vrepScriptFunc)

            if err > abs(syncErrThresh):
                # Wrap and send joint positions to Baxter
                limb_joints = dict(zip(robot.left._joint_names['left'], resp.outputFloats))
                robot.left.set_joint_positions(limb_joints)

                # print('V-rep Joints: ',limb_joints)
            # resp = vrepScriptFunc.call('get_pose@Baxter_leftArm_target',1,[],[],[],'')
            # print 'V-rep pose: ',resp.outputFloats
            # print('---------------------------------------------------------------------------------'/0)
            # print 'Finish pose: ',current_p
            # print('Actual joints: ', left.joint_angles())
            # print('---------------------------------------------------------------------------------')

        rate.sleep()



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
        '-j', '--joystick', required=False,
        choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    joystick = None
    if args.joystick == 'xbox':
        joystick = jstick.XboxController()
    elif args.joystick == 'logitech':
        joystick = baxter_external_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = baxter_external_devices.joystick.PS3Controller()
    # else:
    #     parser.error("Unsupported joystick type '%s'" % (args.joystick))

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
    # if joystick == None:
    #     map_keyboard()
    # else: map_joystick(joystick)
    run_control(joystick)
    print("Done.")

if __name__ == '__main__':
    main()