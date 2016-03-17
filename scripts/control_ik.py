#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy
import math

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from tf import transformations

import baxter_interface
import baxter_external_devices

import moveit_commander
import moveit_msgs.msg

from baxter_pykdl import baxter_kinematics
from moveit_commander.exception import MoveItCommanderException

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = "../models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def map_joystick(joystick,group,kin):
    """
    Maps joystick input to joint position commands.

    @param joystick: an instance of a Joystick
    """

    rcmd = {}
    changed = False
    delta = 0.01
    d_theta = 5
    ori = 'False'
    rate = rospy.Rate(100)
    waypoints = []
    joints = ['left_w0','left_w1','left_w2','left_e0','left_e1','left_s0','left_s1']
    ik_pose = group.get_current_pose().pose
    print("Press Ctrl-C to stop. ")
    while not rospy.is_shutdown():
        group.set_start_state_to_current_state()
        pos = [0.582583, -0.180819, 0.216003]

        # waypoints.append(copy.deepcopy(ik_pose))
        print "Before: /n", ik_pose
        # ik_pose.orientation.w = 1

        angles = transformations.euler_from_quaternion(ik_pose.orientation.__getstate__())

        if joystick.button_down('leftBumper'):
            ori = True
        elif joystick.button_up('leftBumper'):
            ori = False

        if joystick.button_down('rightTrigger'):
            pnp._gripper.close()
        elif joystick.button_up('rightTrigger'):
            pnp._gripper.open()

        x,y,z=angles[0],angles[1],angles[2]
        if joystick.stick_value('leftStickVert') > 0:
            print 'changed'
            changed = True
            if ori == True:
                y = y + d_theta*math.pi/180
            else:
                ik_pose.position.x = ik_pose.position.x + delta
        elif joystick.stick_value('leftStickVert') < 0:
            changed = True
            if ori == True:
                y = y - d_theta*math.pi/180
            else:
                ik_pose.position.x = ik_pose.position.x - delta
        if joystick.stick_value('leftStickHorz') > 0:
            changed = True
            if ori == True:
                x = x + d_theta*math.pi/180
            else:
                ik_pose.position.y = ik_pose.position.y + delta
        elif joystick.stick_value('leftStickHorz') < 0:
            changed = True
            if ori == True:
                x = x - d_theta*math.pi/180
            else:
                ik_pose.position.y = ik_pose.position.y - delta
        if joystick.stick_value('rightStickVert') > 0:
            changed = True
            ik_pose.position.z = ik_pose.position.z + delta
        elif joystick.stick_value('rightStickVert') < 0:
            changed = True
            ik_pose.position.z = ik_pose.position.z - delta
        if joystick.stick_value('rightStickHorz') > 0:
            changed = True
            if ori == True:
                z = z + d_theta*math.pi/180
        elif joystick.stick_value('rightStickHorz') < 0:
            changed = True
            if ori == True:
                z = z - d_theta*math.pi/180

        if ori==True:
            ik_pose.orientation.__setstate__(transformations.quaternion_from_euler(x,y,z))

        print "After: /n", ik_pose

        pos = [ik_pose.position.x,ik_pose.position.y,ik_pose.position.z]
        pos = [0.582583, -0.180819, 0.216003]
        ori = [ik_pose.orientation.x,ik_pose.orientation.y,ik_pose.orientation.z, ik_pose.orientation.w]

        if changed:
            if ori==True:
                angles = kin.inverse_kinematics(pos,oril)
            else:
                angles = kin.inverse_kinematics(pos)
            print 'Angles: ',angles
            if not angles == None:
                limb_joints = dict(zip(joints, angles))
                try:
                    group.set_joint_value_target(limb_joints)
                except MoveItCommanderException:
                    print'Unreachable position. Aborting'
                    continue
                group.go()

        # if changed:
        #     waypoints.append(copy.deepcopy(ik_pose))
        #     (plan3, fraction) = group.compute_cartesian_path(
        #                              waypoints,   # waypoints to follow
        #                              0.01,        # eef_step
        #                              0.0)         # jump_threshold
        #     group.execute(plan3)
        #     changed = False

        # group.set_pose_target(ik_pose,end_effector_link='left_gripper_base')
        # group.go(ik_pose, wait=False)


        rate.sleep()

    return False

def map_joystick_pnp(joystick,pnp):
    """
    Maps joystick input to joint position commands.

    @param joystick: an instance of a Joystick
    """
    # right = baxter_interface.Limb('right')
    # grip_right = baxter_interface.Gripper('right', baxter_interface.CHECK_VERSION)
    rcmd = {}
    changed = False
    delta = 0.01
    d_theta = 5
    ori = 'False'
    rate = rospy.Rate(100)

    print("Press Ctrl-C to stop. ")
    while not rospy.is_shutdown():
        current_pose = pnp._limb.endpoint_pose()
        ik_pose = Pose()

        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        angles = transformations.euler_from_quaternion(ik_pose.orientation.__getstate__())

        if joystick.button_down('leftBumper'):
            ori = True
        elif joystick.button_up('leftBumper'):
            ori = False

        if joystick.button_down('rightTrigger'):
            pnp._gripper.close()
        elif joystick.button_up('rightTrigger'):
            pnp._gripper.open()

        x,y,z=angles[0],angles[1],angles[2]
        if joystick.stick_value('leftStickVert') > 0:
            changed = True
            if ori == True:
                y = y + d_theta*math.pi/180
            else:
                ik_pose.position.x = current_pose['position'].x + delta
        elif joystick.stick_value('leftStickVert') < 0:
            changed = True
            if ori == True:
                y = y - d_theta*math.pi/180
            else:
                ik_pose.position.x = current_pose['position'].x - delta
        if joystick.stick_value('leftStickHorz') > 0:
            changed = True
            if ori == True:
                x = x + d_theta*math.pi/180
            else:
                ik_pose.position.y = current_pose['position'].y + delta
        elif joystick.stick_value('leftStickHorz') < 0:
            changed = True
            if ori == True:
                x = x - d_theta*math.pi/180
            else:
                ik_pose.position.y = current_pose['position'].y - delta
        if joystick.stick_value('rightStickVert') > 0:
            changed = True
            ik_pose.position.z = current_pose['position'].z + delta
        elif joystick.stick_value('rightStickVert') < 0:
            changed = True
            ik_pose.position.z = current_pose['position'].z - delta
        if joystick.stick_value('rightStickHorz') > 0:
            changed = True
            if ori == True:
                z = z + d_theta*math.pi/180
        elif joystick.stick_value('rightStickHorz') < 0:
            changed = True
            if ori == True:
                z = z - d_theta*math.pi/180

        if ori:
            ik_pose.orientation.__setstate__(transformations.quaternion_from_euler(x,y,z))

        # print('Calculating joint positions')
        if changed == True:
            rcmd = pnp.ik_request(ik_pose)
        # print('Calculation done')

        if rcmd:
            # print('rcmd: {0}'.format(rcmd))
            # print('current_pose: {0}'.format(current_pose))
            # print('ik_pose: {0}'.format(ik_pose))
            # print('Setting joint positions')
            new = False
            # for k,v in rcmd.items():
                # if abs(v-)
            #     print('New position')
            if changed == True:
                pnp._limb.set_joint_positions(rcmd)
            # pnp._guarded_move_to_joint_position(rcmd)
            rcmd.clear()
            changed = False
        rate.sleep()
    return False

def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    # rospy.init_node("ik_pick_and_place_demo")
    print("Initializing node... ")
    rospy.init_node("cartesian_control")
    print 'Initialised'
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    # load_gazebo_models()
    # Remove models from the scene on shutdown
    # rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    # rospy.wait_for_message("/robot/sim/started", Empty)

    # print("Getting robot state... ")
    # rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    # init_state = rs.state().enabled
    #
    # print("Enabling robot... ")
    # rs.enable()

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.9999952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    joints = ['left_w0',
                             'left_w1',
                             'left_w2',
                             'left_e0',
                             'left_e1',
                             'left_s0',
                             'left_s1']
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

    pnp = PickAndPlace(limb, hover_distance,verbose=False)
    pnp.move_to_start(starting_joint_angles)

    joystick = baxter_external_devices.joystick.XboxController()
    map_joystick(joystick, group, kin)
    print("Done.")

    return 0

if __name__ == '__main__':
    sys.exit(main())
