"""
Jenga - First Layer Pick and Place
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
    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    # Retracting arm by moving back to appropriated hover_distance
    # Here we are also retrieving the current pose of the end-effector
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

    # Find joint angles using IK solver and pose, then apply to Baxter
    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    # Pick method applying combination of other methods
    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        print('Approaching')
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        print('Ready to grip')
        # close gripper
        self.gripper_close()
        print('grip')
        # retract to clear object
        self._retract()

    # Place method applying a combination of other methods
    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()


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
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


'''
MAIN FUNCTION
'''
def main():
    """Simple pick and place example"""
    rospy.init_node("ik_pick_and_place_demo")

    hover_distance = 0.1 # meters

    # Define offset - original table sdf was 0.82 for base(0.04) + column(0.04) + surface(0.74)
    off = 0.095

    # Start Pose for left arm
    left_pose = Pose()
    left_pose.position.x = 0.589679836383
    left_pose.position.y = 0.183311769707
    left_pose.position.z = 0.3 - off
    left_pose.orientation.x = -0.0249590815779
    left_pose.orientation.y = 0.999649402929
    left_pose.orientation.z = 0.00737916180073
    left_pose.orientation.w = 0.00486450832011

    # Start Pose for right arm
    right_pose = Pose()
    right_pose.position.x = 0.579679836383
    right_pose.position.y = -0.283311769707
    right_pose.position.z = 0.213676720426 - off
    right_pose.orientation.x = -0.0249590815779
    right_pose.orientation.y = 0.999649402929
    right_pose.orientation.z = -0.00737916180073
    right_pose.orientation.w = 0.00486450832011

    # Brick Pose
    brick_pose = Pose()
    brick_pose.position.x = 0.589679836383
    brick_pose.position.y = 0.183311769707
    brick_pose.position.z = 0.817893 - off

    # Pick pose
    pick_pose = Pose()
    pick_pose.position.x = 0.589679836383
    pick_pose.position.y = 0.183311769707
    pick_pose.position.z = 0.183676720426 - off
    pick_pose.orientation.x = -0.99
    pick_pose.orientation.y = 0.999649402929
    pick_pose.orientation.z = 0.00737916180073
    pick_pose.orientation.w = 0.00486450832011

    # Place pose 1
    place_pose1 = Pose()
    place_pose1.position.x = 0.589679836383
    place_pose1.position.y = 0.0
    place_pose1.position.z = 0.183676720426 - off
    place_pose1.orientation.x = -0.0249590815779
    place_pose1.orientation.y = 0.999649402929
    place_pose1.orientation.z = 0.00737916180073
    place_pose1.orientation.w = 0.00486450832011

    # Place pose 2 (x + 0.07)
    place_pose2 = Pose()
    place_pose2.position.x = 0.659679836383 
    place_pose2.position.y = 0.0
    place_pose2.position.z = 0.183676720426 - off
    place_pose2.orientation.x = -0.0249590815779
    place_pose2.orientation.y = 0.999649402929
    place_pose2.orientation.z = 0.00737916180073
    place_pose2.orientation.w = 0.00486450832011


    # Place pose 3
    place_pose3 = Pose()
    place_pose3.position.x = 0.729679836383
    place_pose3.position.y = 0.0
    place_pose3.position.z = 0.183676720426 - off
    place_pose3.orientation.x = -0.0249590815779
    place_pose3.orientation.y = 0.999649402929
    place_pose3.orientation.z = 0.00737916180073
    place_pose3.orientation.w = 0.00486450832011

    # Place pose 4
    place_pose4 = Pose()
    place_pose4.position.x = 0.659679836383
    place_pose4.position.y = -0.06
    place_pose4.position.z = 0.27 - off
    place_pose4.orientation.x = -0.99
    place_pose4.orientation.y = 0.999649402929
    place_pose4.orientation.z = 0.00737916180073
    place_pose4.orientation.w = 0.00486450832011


    # Initialise pick and place
    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))
    right_pnp.move_to_start(right_pnp.ik_request(right_pose))

    # Spawn table
    spawn_gazebo_table()

    # Spawn brick1
    spawn_gazebo_brick(brick_pose=brick_pose, count=1)

    # Pick and Place brick1
    print("\nPicking...")
    left_pnp.pick(pick_pose)
    print("\nPlacing...")
    left_pnp.place(place_pose1)

    rospy.sleep(2.0)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))

    # Spawn brick2
    spawn_gazebo_brick(brick_pose=brick_pose, count=2)

    # Pick and Place brick2
    print("\nPicking...")
    left_pnp.pick(pick_pose)
    print("\nPlacing...")
    left_pnp.place(place_pose2)

    rospy.sleep(2.0)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))

    # Spawn brick3
    spawn_gazebo_brick(brick_pose=brick_pose, count=3)

    # Pick and Place brick3
    print("\nPicking...")
    left_pnp.pick(pick_pose)
    print("\nPlacing...")
    left_pnp.place(place_pose3)

    rospy.sleep(2.0)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))

    # Spawn brick4
    spawn_gazebo_brick(brick_pose=brick_pose, count=4)

    # Pick and Place brick4
    print("\nPicking...")
    left_pnp.pick(pick_pose)
    print("\nPlacing...")
    left_pnp.place(place_pose4)

    rospy.sleep(10.0)


    '''

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))

    # Spawn brick3
    spawn_gazebo_brick()

    # Pick and Place brick3
    print("\nPicking...")
    left_pnp.pick(pick_pose)
    print("\nPlacing...")
    left_pnp.place(place_pose2)

    rospy.sleep(5.0)

    '''

    # Delete all models
    delete_gazebo_models()

    #rospy.on_shutdown(delete_gazebo_models)

    


if __name__ == '__main__':
    sys.exit(main())
    #delete_gazebo_models()
