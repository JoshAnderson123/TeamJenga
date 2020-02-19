"""
Jenga - play
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import baxter_interface

import tf


'''
CLASS TO CREATE ARM/GRIPPER OBJECTS
'''
class PickAndPlace(object):
    # Creating pick and place object (for a single arm & gripper combo)
    def __init__(self, limb, hover_distance = 0.10, verbose=True):
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

    # Method for start state of pick and place object
    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    # Inverse Kinematics solver, returning joint angles for end-effector pose
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

    # Move joints of arm of object with given joint angles argument
    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    # Open gripper method for pick and place object
    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    # Close gripper of object
    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    # Move joints of arm of object to achieve inputted pose - but with a hover_distance applied to z
    def _approach(self, pose, num):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        if num == 1:
            approach.position.z = approach.position.z + self._hover_distance
            joint_angles = self.ik_request(approach)
            self._guarded_move_to_joint_position(joint_angles)
        elif num == 2:
            approach.position.y = approach.position.y + self._hover_distance
            joint_angles = self.ik_request(approach)
            self._guarded_move_to_joint_position(joint_angles)

    # Retracting arm by moving back to appropriated hover_distance
    # Here we are also retrieving the current pose of the end-effector
    def _retract(self,num):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        if num == 1:
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
        elif num == 2:
            ik_pose.position.x = current_pose['position'].x
            ik_pose.position.y = current_pose['position'].y + self._hover_distance
            ik_pose.position.z = current_pose['position'].z 
            ik_pose.orientation.x = current_pose['orientation'].x
            ik_pose.orientation.y = current_pose['orientation'].y
            ik_pose.orientation.z = current_pose['orientation'].z
            ik_pose.orientation.w = current_pose['orientation'].w
            joint_angles = self.ik_request(ik_pose)
            # servo up from current pose
            self._guarded_move_to_joint_position(joint_angles)

    # Find joint angles using IK solver and pose, then apply to Baxter
    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    # Pick method applying combination of other methods
    def pick(self, pose, num):
        # open the gripper
        self.gripper_open()
        # servo above pose
        print('Approaching')
        self._approach(pose, num)
        # servo to pose
        self._servo_to_pose(pose)
        print('Ready to grip')
        # close gripper
        self.gripper_close()
        print('grip')
        # retract to clear object
        self._retract(num)

    # Place method applying a combination of other methods
    def place(self, pose, num):
        # servo above pose
        self._approach(pose,num)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract(num)

    def move_to(self, joint_angles):
        """
        Send target joint position to the joint position controller
        """
        rospy.sleep(1.0)

        rospy.loginfo("Robot move to joint position started...")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(0.3)

        # Send joint angles command
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles, timeout=20.0, threshold=baxter_interface.settings.JOINT_ANGLE_TOLERANCE)
        else:
            rospy.logerr("No Joint Angles provided...")

'''
GAZEBO MODEL FUNCTIONS
'''

# Spawning table with predefined pose
def spawn_gazebo_table(table_pose=Pose(position=Point(x=0.95, y=0.0, z=0.0)),
                       table_reference_frame="world"):

    # Load Table SDF
    table_xml = ''
    with open ("models/cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

# Spawning brick with predefined pose
def spawn_gazebo_brick(brick_pose,
                       brick_reference_frame="world",
                       count=1):

    # Load Brick SDF
    brick_xml = ''
    with open ("models/brick/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

     # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:

        if (count == 1):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick1", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 2):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick2", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 3):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick3", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 4):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick4", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 5):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick5", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 6):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick6", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 7):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick7", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 8):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick8", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 9):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick9", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 10):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick10", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 11):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick11", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

        elif (count == 12):
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("brick12", brick_xml, "/",
                                 brick_pose, brick_reference_frame)

    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

# Deleting models from gazebo
def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("brick1")
        resp_delete = delete_model("brick2")
        resp_delete = delete_model("brick3")
        resp_delete = delete_model("brick4")
        resp_delete = delete_model("brick5")
        resp_delete = delete_model("brick6")
        resp_delete = delete_model("brick7")
        resp_delete = delete_model("brick8")
        resp_delete = delete_model("brick9")
        resp_delete = delete_model("brick10")
        resp_delete = delete_model("brick11")
        resp_delete = delete_model("brick12")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

'''
MAIN FUNCTION
'''
def main():
    """Simple pick and place example"""
    rospy.init_node("ik_pick_and_place_demo")

    '''
    OFFSET VARIABLES
    '''
    hover_distance = 0.1 # meters

    # Define offset - original table sdf was 0.82 for base(0.04) + column(0.04) + surface(0.74)
    off = 0.32

    # Trim for how close brick can go from gazebo to real world
    trim = 0

    '''
    ORIENTATITIONS
    '''
    # This orientation matches that of 'tuck_arms.py -u'
    roll1 = 3.14159265359
    pitch1 = 0
    yaw1 = -3.14159265359
    quat1 = tf.transformations.quaternion_from_euler(roll1, pitch1, yaw1)

    # This orientation matches that of layers 1 and 3 in the jenga tower
    brick_roll1 = 3.14159265359
    brick_pitch1 = 0
    brick_yaw1 = -1.57079632679

    brick_quat1 = tf.transformations.quaternion_from_euler(brick_roll1, brick_pitch1, brick_yaw1)

    # rotate end-effector 90 degrees about z
    roll2 = 3.14159265359
    pitch2 = 0
    yaw2 = -1.57079632679
    quat2 = tf.transformations.quaternion_from_euler(roll2, pitch2, yaw2)

    #This orientation matches that of layers 2 and 4 in the jenga tower
    brick_roll2 = 3.14159265359
    brick_pitch2 = 0
    brick_yaw2 = 0

    brick_quat2 = tf.transformations.quaternion_from_euler(brick_roll2, brick_pitch2, brick_yaw2)

    # rotate end-effector -90 degrees about x
    roll3 =  -1.57079632679 #3.14159265359
    pitch3 = 0 #-1.57079632679
    yaw3 = -3.14159265359
    quat3 = tf.transformations.quaternion_from_euler(roll3, pitch3, yaw3)

    roll3b = -1.57079632679 #3.14159265359
    pitch3b = -1.57079632679
    yaw3b = -3.14159265359
    quat3b = tf.transformations.quaternion_from_euler(roll3b, pitch3b, yaw3b)    

    # rotate end-effector 90 degrees about z (from quat3b)
    roll4 = 3.14159265359
    pitch4 = -1.57079632679
    yaw4 = 0
    quat4 = tf.transformations.quaternion_from_euler(roll4, pitch4, yaw4)

    # rotate end-effector 90 degrees about y
    #roll5 = 3.14159265359 #change this angle
    #pitch5 = -1.57079632679
    #yaw5 = -1.57079632679
    #quat5 = tf.transformations.quaternion_from_euler(roll5, pitch5, yaw5)

    '''
    INITIALISER POSES
    '''
    # Start Pose for left arm
    left_pose = Pose()
    left_pose.position.x = 0.589679836383
    left_pose.position.y = 0.3333
    left_pose.position.z = 0.3 - off
    left_pose.orientation.x = quat1[0]
    left_pose.orientation.y = quat1[1]
    left_pose.orientation.z = quat1[2]
    left_pose.orientation.w = quat1[3]

    # Start Pose for right arm
    right_pose = Pose()
    right_pose.position.x = 0.579679836383
    right_pose.position.y = -0.383311769707
    right_pose.position.z = 0.213676720426 - off
    right_pose.orientation.x = quat1[0]
    right_pose.orientation.y = quat1[1]
    right_pose.orientation.z = quat1[2]
    right_pose.orientation.w = quat1[3]

    # Brick Pose
    brick_pose = Pose()
    brick_pose.position.x = 0.589679836383
    brick_pose.position.y = 0.3333
    brick_pose.position.z = 0.817893 - off

    '''
    PLACING POSES
    '''
    # Place pose 1
    brick_pose1 = Pose()
    brick_pose1.position.x = 0.589679836383
    brick_pose1.position.y = 0.0
    brick_pose1.position.z = 0.183676720426 + off
    brick_pose1.orientation.x = brick_quat1[0]
    brick_pose1.orientation.y = brick_quat1[1]
    brick_pose1.orientation.z = brick_quat1[2]
    brick_pose1.orientation.w = brick_quat1[3]

    # Place pose 2 (x + 0.07)
    brick_pose2 = Pose()
    brick_pose2.position.x = 0.659679836383 - trim
    brick_pose2.position.y = 0.0
    brick_pose2.position.z = 0.183676720426 + off
    brick_pose2.orientation.x = brick_quat1[0]
    brick_pose2.orientation.y = brick_quat1[1]
    brick_pose2.orientation.z = brick_quat1[2]
    brick_pose2.orientation.w = brick_quat1[3]

    # Place pose 3
    brick_pose3 = Pose()
    brick_pose3.position.x = 0.729679836383 - 2*trim
    brick_pose3.position.y = 0.0
    brick_pose3.position.z = 0.183676720426 + off
    brick_pose3.orientation.x = brick_quat1[0]
    brick_pose3.orientation.y = brick_quat1[1]
    brick_pose3.orientation.z = brick_quat1[2]
    brick_pose3.orientation.w = brick_quat1[3]

    # Place pose 4
    brick_pose4 = Pose()
    brick_pose4.position.x = 0.659679836383
    brick_pose4.position.y = -0.06
    brick_pose4.position.z = 0.27 + off
    brick_pose4.orientation.x = brick_quat2[0]
    brick_pose4.orientation.y = brick_quat2[1]
    brick_pose4.orientation.z = brick_quat2[2]
    brick_pose4.orientation.w = brick_quat2[3]

    # Place pose 5 (y + 0.065)
    brick_pose5 = Pose()
    brick_pose5.position.x = 0.659679836383
    brick_pose5.position.y = 0.005 - trim
    brick_pose5.position.z = 0.27 + off
    brick_pose5.orientation.x = brick_quat2[0]
    brick_pose5.orientation.y = brick_quat2[1]
    brick_pose5.orientation.z = brick_quat2[2]
    brick_pose5.orientation.w = brick_quat2[3]

    # Place pose 6
    brick_pose6 = Pose()
    brick_pose6.position.x = 0.659679836383
    brick_pose6.position.y = 0.07 - 2*trim
    brick_pose6.position.z = 0.27 + off
    brick_pose6.orientation.x = brick_quat2[0]
    brick_pose6.orientation.y = brick_quat2[1]
    brick_pose6.orientation.z = brick_quat2[2]
    brick_pose6.orientation.w = brick_quat2[3]


    # Place pose 7
    brick_pose7 = Pose()
    brick_pose7.position.x = 0.589679836383
    brick_pose7.position.y = 0.0
    brick_pose7.position.z = 0.357 + off
    brick_pose7.orientation.x = brick_quat1[0]
    brick_pose7.orientation.y = brick_quat1[1]
    brick_pose7.orientation.z = brick_quat1[2]
    brick_pose7.orientation.w = brick_quat1[3]

    # Place pose 8 (x + 0.07)
    brick_pose8 = Pose()
    brick_pose8.position.x = 0.659679836383 - trim
    brick_pose8.position.y = 0.0
    brick_pose8.position.z = 0.357 + off
    brick_pose8.orientation.x = brick_quat1[0]
    brick_pose8.orientation.y = brick_quat1[1]
    brick_pose8.orientation.z = brick_quat1[2]
    brick_pose8.orientation.w = brick_quat1[3]


    # Place pose 9
    brick_pose9 = Pose()
    brick_pose9.position.x = 0.729679836383  - 2*trim
    brick_pose9.position.y = 0.0
    brick_pose9.position.z = 0.357 + off
    brick_pose9.orientation.x = brick_quat1[0]
    brick_pose9.orientation.y = brick_quat1[1]
    brick_pose9.orientation.z = brick_quat1[2]
    brick_pose9.orientation.w = brick_quat1[3]

    # Place pose 10
    brick_pose10 = Pose()
    brick_pose10.position.x = 0.659679836383
    brick_pose10.position.y = -0.06
    brick_pose10.position.z = 0.444 + off
    brick_pose10.orientation.x = brick_quat2[0]
    brick_pose10.orientation.y = brick_quat2[1]
    brick_pose10.orientation.z = brick_quat2[2]
    brick_pose10.orientation.w = brick_quat2[3]

    # Place pose 11 (y + 0.065)
    brick_pose11 = Pose()
    brick_pose11.position.x = 0.659679836383 
    brick_pose11.position.y = 0.005 - trim
    brick_pose11.position.z = 0.444 + off
    brick_pose11.orientation.x = brick_quat2[0]
    brick_pose11.orientation.y = brick_quat2[1]
    brick_pose11.orientation.z = brick_quat2[2]
    brick_pose11.orientation.w = brick_quat2[3]

    # Place pose 12
    brick_pose12 = Pose()
    brick_pose12.position.x = 0.659679836383
    brick_pose12.position.y = 0.07 - 2*trim 
    brick_pose12.position.z = 0.444 + off
    brick_pose12.orientation.x = brick_quat2[0]
    brick_pose12.orientation.y = brick_quat2[1]
    brick_pose12.orientation.z = brick_quat2[2]
    brick_pose12.orientation.w = brick_quat2[3]

    '''
    PLACING POSES
    '''
    # Place pose 1
    place_pose1 = Pose()
    place_pose1.position.x = 0.589679836383
    place_pose1.position.y = 0.0
    place_pose1.position.z = 0.183676720426 + off
    place_pose1.orientation.x = quat1[0]
    place_pose1.orientation.y = quat1[1]
    place_pose1.orientation.z = quat1[2]
    place_pose1.orientation.w = quat1[3]

    # Place pose 2 (x + 0.07)
    place_pose2 = Pose()
    place_pose2.position.x = 0.659679836383 + trim
    place_pose2.position.y = 0.0
    place_pose2.position.z = 0.183676720426 + off
    place_pose2.orientation.x = quat1[0]
    place_pose2.orientation.y = quat1[1]
    place_pose2.orientation.z = quat1[2]
    place_pose2.orientation.w = quat1[3]

    # Place pose 3
    place_pose3 = Pose()
    place_pose3.position.x = 0.729679836383 + 2*trim
    place_pose3.position.y = 0.0
    place_pose3.position.z = 0.183676720426 + off
    place_pose3.orientation.x = quat1[0]
    place_pose3.orientation.y = quat1[1]
    place_pose3.orientation.z = quat1[2]
    place_pose3.orientation.w = quat1[3]

    # Place pose 4
    place_pose4 = Pose()
    place_pose4.position.x = 0.659679836383
    place_pose4.position.y = -0.06
    place_pose4.position.z = 0.27 + off
    place_pose4.orientation.x = quat2[0]
    place_pose4.orientation.y = quat2[1]
    place_pose4.orientation.z = quat2[2]
    place_pose4.orientation.w = quat2[3]

    # Place pose 5 (y + 0.065)
    place_pose5 = Pose()
    place_pose5.position.x = 0.659679836383
    place_pose5.position.y = 0.005 + trim
    place_pose5.position.z = 0.27 + off
    place_pose5.orientation.x = quat2[0]
    place_pose5.orientation.y = quat2[1]
    place_pose5.orientation.z = quat2[2]
    place_pose5.orientation.w = quat2[3]

    # Place pose 6
    place_pose6 = Pose()
    place_pose6.position.x = 0.659679836383
    place_pose6.position.y = 0.07 + 2*trim
    place_pose6.position.z = 0.27 + off
    place_pose6.orientation.x = quat2[0]
    place_pose6.orientation.y = quat2[1]
    place_pose6.orientation.z = quat2[2]
    place_pose6.orientation.w = quat2[3]

    # Place pose 7
    place_pose7 = Pose()
    place_pose7.position.x = 0.589679836383
    place_pose7.position.y = 0.0
    place_pose7.position.z = 0.357 + off
    place_pose7.orientation.x = quat1[0]
    place_pose7.orientation.y = quat1[1]
    place_pose7.orientation.z = quat1[2]
    place_pose7.orientation.w = quat1[3]

    # Place pose 8 (x + 0.07)
    place_pose8 = Pose()
    place_pose8.position.x = 0.659679836383 + trim
    place_pose8.position.y = 0.0
    place_pose8.position.z = 0.357 + off
    place_pose8.orientation.x = quat1[0]
    place_pose8.orientation.y = quat1[1]
    place_pose8.orientation.z = quat1[2]
    place_pose8.orientation.w = quat1[3]

    # Place pose 9
    place_pose9 = Pose()
    place_pose9.position.x = 0.729679836383 + 2*trim
    place_pose9.position.y = 0.0
    place_pose9.position.z = 0.357 + off
    place_pose9.orientation.x = quat1[0]
    place_pose9.orientation.y = quat1[1]
    place_pose9.orientation.z = quat1[2]
    place_pose9.orientation.w = quat1[3]

    # Place pose 10
    place_pose10 = Pose()
    place_pose10.position.x = 0.659679836383
    place_pose10.position.y = -0.06
    place_pose10.position.z = 0.444 + off
    place_pose10.orientation.x = quat2[0]
    place_pose10.orientation.y = quat2[1]
    place_pose10.orientation.z = quat2[2]
    place_pose10.orientation.w = quat2[3]

    # Place pose 11 (y + 0.065)
    place_pose11 = Pose()
    place_pose11.position.x = 0.659679836383
    place_pose11.position.y = 0.005 + trim
    place_pose11.position.z = 0.444 + off
    place_pose11.orientation.x = quat2[0]
    place_pose11.orientation.y = quat2[1]
    place_pose11.orientation.z = quat2[2]
    place_pose11.orientation.w = quat2[3]

    # Place pose 12
    place_pose12 = Pose()
    place_pose12.position.x = 0.659679836383
    place_pose12.position.y = 0.07 + 2*trim
    place_pose12.position.z = 0.444 + off
    place_pose12.orientation.x = quat2[0]
    place_pose12.orientation.y = quat2[1]
    place_pose12.orientation.z = quat2[2]
    place_pose12.orientation.w = quat2[3]

    '''
    ADDITIONAL POSES FOR STABILITY
    '''
    # clear pose level 1
    clear_pose1 = Pose()
    clear_pose1.position.x = 0.859679836383
    clear_pose1.position.y = 0.3333
    clear_pose1.position.z = 0.283676720426 - off
    clear_pose1.orientation.x = quat2[0]
    clear_pose1.orientation.y = quat2[1]
    clear_pose1.orientation.z = quat2[2]
    clear_pose1.orientation.w = quat2[3]

    # clear pose level 2
    clear_pose2 = Pose()
    clear_pose2.position.x = 0.859679836383
    clear_pose2.position.y = 0.3333
    clear_pose2.position.z = 0.37 - off
    clear_pose2.orientation.x = quat2[0]
    clear_pose2.orientation.y = quat2[1]
    clear_pose2.orientation.z = quat2[2]
    clear_pose2.orientation.w = quat2[3]

    # clear pose level 3
    clear_pose3 = Pose()
    clear_pose3.position.x = 0.859679836383
    clear_pose3.position.y = 0.3333
    clear_pose3.position.z = 0.456 - off
    clear_pose3.orientation.x = quat2[0]
    clear_pose3.orientation.y = quat2[1]
    clear_pose3.orientation.z = quat2[2]
    clear_pose3.orientation.w = quat2[3]

    # clear pose level 4
    clear_pose4 = Pose()
    clear_pose4.position.x = 0.589679836383
    clear_pose4.position.y = 0.3333
    clear_pose4.position.z = 0.542 - off
    clear_pose4.orientation.x = quat2[0]
    clear_pose4.orientation.y = quat2[1]
    clear_pose4.orientation.z = quat2[2]
    clear_pose4.orientation.w = quat2[3]

    # prepare pose level 3
    prep_pose1 = Pose()
    prep_pose1.position.x = 0.589679836383
    prep_pose1.position.y = 0.3333
    prep_pose1.position.z = 0.357 - off
    prep_pose1.orientation.x = quat2[0]
    prep_pose1.orientation.y = quat2[1]
    prep_pose1.orientation.z = quat2[2]
    prep_pose1.orientation.w = quat2[3]

    # prepare pose level 4
    prep_pose2 = Pose()
    prep_pose2.position.x = 0.589679836383
    prep_pose2.position.y = 0.3333
    prep_pose2.position.z = 0.444 - off
    prep_pose2.orientation.x = quat2[0]
    prep_pose2.orientation.y = quat2[1]
    prep_pose2.orientation.z = quat2[2]
    prep_pose2.orientation.w = quat2[3]

    # clear play 1
    clear_1 = Pose()
    clear_1.position.x = 0.589679836383 #0.659679836383
    clear_1.position.y = 0.3333
    clear_1.position.z = 0.357 - off #0.27 - off
    clear_1.orientation.x = quat3[0]
    clear_1.orientation.y = quat3[1]
    clear_1.orientation.z = quat3[2]
    clear_1.orientation.w = quat3[3]

    # clear play 1 part 2
    clear_1b = Pose()
    clear_1b.position.x = 0.589679836383 #0.659679836383
    clear_1b.position.y = 0.3333
    clear_1b.position.z = 0.357 - off #0.27 - off
    clear_1b.orientation.x = quat3b[0]
    clear_1b.orientation.y = quat3b[1]
    clear_1b.orientation.z = quat3b[2]
    clear_1b.orientation.w = quat3b[3]

    clear_1c = Pose()
    clear_1c.position.x = 0.589679836383 #0.659679836383
    clear_1c.position.y = 0.3333
    clear_1c.position.z = 0.357 - off #0.27 - off
    clear_1c.orientation.x = quat4[0]
    clear_1c.orientation.y = quat4[1]
    clear_1c.orientation.z = quat4[2]
    clear_1c.orientation.w = quat4[3]

    clear_1d = Pose()
    clear_1d.position.x = 0.589679836383 #0.659679836383
    clear_1d.position.y = 0.3333
    clear_1d.position.z = 0.531 - off
    clear_1d.orientation.x = quat4[0]
    clear_1d.orientation.y = quat4[1]
    clear_1d.orientation.z = quat4[2]
    clear_1d.orientation.w = quat4[3]


    # prepare pose level 3
    prep_pose1 = Pose()
    prep_pose1.position.x = 0.589679836383
    prep_pose1.position.y = 0.3333
    prep_pose1.position.z = 0.357 - off
    prep_pose1.orientation.x = quat2[0]
    prep_pose1.orientation.y = quat2[1]
    prep_pose1.orientation.z = quat2[2]
    prep_pose1.orientation.w = quat2[3]



    #clear play 2
    clear_2 = Pose()
    clear_2.position.x = 0.489679836383
    clear_2.position.y = 0.0
    clear_2.position.z = 0.531 - off
    clear_2.orientation.x = quat4[0]
    clear_2.orientation.y = quat4[1]
    clear_2.orientation.z = quat4[2]
    clear_2.orientation.w = quat4[3]

    '''
    PLAYING POSES
    '''
    #Pick up brick 6 from the side
    play_pick6_pose = Pose()
    play_pick6_pose.position.x = 0.659679836383
    play_pick6_pose.position.y = 0.07 - 2*trim
    play_pick6_pose.position.z = 0.27 - off
    play_pick6_pose.orientation.x = quat3b[0]
    play_pick6_pose.orientation.y = quat3b[1]
    play_pick6_pose.orientation.z = quat3b[2]
    play_pick6_pose.orientation.w = quat3b[3]

    #Place brick 6 on top of tower (pose14) (z + 0.065)
    play_place_pose14 = Pose()
    play_place_pose14.position.x = 0.659679836383 - trim #0.589679836383 #0.729679836383 + 2*trim
    play_place_pose14.position.y = 0.0
    play_place_pose14.position.z = 0.531 - off #0.531 - off
    play_place_pose14.orientation.x = quat4[0]
    play_place_pose14.orientation.y = quat4[1]
    play_place_pose14.orientation.z = quat4[2]
    play_place_pose14.orientation.w = quat4[3]

    #Push out brick 5
    #NEEDS POSITION AND ANGLES FIXING
    #play_push5_pose = Pose()
    #play_push5_pose.position.x = 0.589679836383 #check positions
    #play_push5_pose.position.y = 0.0
    #play_push5_pose.position.z = 0.531 - off
    #play_push5_pose.orientation.x = quat5[0]
    #play_push5_pose.orientation.y = quat5[1]
    #play_push5_pose.orientation.z = quat5[2]
    #play_push5_pose.orientation.w = quat5[3]

    '''
    ACTION SEQUENCE
    '''
    # Initialise pick and place
    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))
    right_pnp.move_to_start(right_pnp.ik_request(right_pose))

    # Spawn table
    spawn_gazebo_table()

    # Spawn brick1
    spawn_gazebo_brick(brick_pose=brick_pose1, count=1)

    rospy.sleep(0.5)

    # Spawn brick2
    spawn_gazebo_brick(brick_pose=brick_pose2, count=2)

    rospy.sleep(0.5) 

    # Spawn brick3
    spawn_gazebo_brick(brick_pose=brick_pose3, count=3)

    rospy.sleep(0.5)

    # Spawn brick4
    spawn_gazebo_brick(brick_pose=brick_pose4, count=4)

    rospy.sleep(0.5)

    # Spawn brick5
    spawn_gazebo_brick(brick_pose=brick_pose5, count=5)

    rospy.sleep(0.5)

    # Spawn brick6
    spawn_gazebo_brick(brick_pose=brick_pose6, count=6)

    rospy.sleep(0.5) 

    # Spawn brick7
    spawn_gazebo_brick(brick_pose=brick_pose7, count=7)

    rospy.sleep(0.5)

    # Spawn brick8
    spawn_gazebo_brick(brick_pose=brick_pose8, count=8)

    rospy.sleep(0.5)

    # Spawn brick9
    spawn_gazebo_brick(brick_pose=brick_pose9, count=9)

    rospy.sleep(0.5)

    # Spawn brick10
    spawn_gazebo_brick(brick_pose=brick_pose10, count=10)

    rospy.sleep(0.5) 

    # Spawn brick11
    spawn_gazebo_brick(brick_pose=brick_pose11, count=11)

    rospy.sleep(0.5)

    # Spawn brick12
    spawn_gazebo_brick(brick_pose=brick_pose12, count=12)

    rospy.sleep(0.5)

    #Play - pick and place brick 6
    print("\nPicking...")
    left_pnp.move_to(left_pnp.ik_request(clear_1))

    left_pnp.move_to(left_pnp.ik_request(clear_1b)) 

    #left_pnp.move_to(left_pnp.ik_request(clear_1c))   

    left_pnp.pick(play_pick6_pose,2)

    rospy.sleep(2.0)

    left_pnp.move_to(left_pnp.ik_request(clear_1b))

    left_pnp.move_to(left_pnp.ik_request(clear_1c))

    print("\nPlacing...")
    left_pnp.move_to(left_pnp.ik_request(clear_1d))

    left_pnp.place(play_place_pose14,1)

    rospy.sleep(2.0)

    left_pnp.move_to(left_pnp.ik_request(clear_1d))

    rospy.sleep(2.0)

    # Delete all models
    delete_gazebo_models()

    #rospy.on_shutdown(delete_gazebo_models)


if __name__ == '__main__':
    sys.exit(main())
    #delete_gazebo_models()
