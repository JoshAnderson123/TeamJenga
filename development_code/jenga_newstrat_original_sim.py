"""
Jenga - Full Tower - MOVE ONE
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
            return "err1"
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
            return "err2"
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
        #num variable changes if the hover distance is in z or y
        if num == 1:
            approach.position.z = approach.position.z + self._hover_distance
            joint_angles = self.ik_request(approach)
            self._guarded_move_to_joint_position(joint_angles)
        elif num == 2:
            approach.position.y = approach.position.y + self._hover_distance
            joint_angles = self.ik_request(approach)
            self._guarded_move_to_joint_position(joint_angles)
        elif num == 3:
            approach.position.z = approach.position.z + 0.357
            joint_angles = self.ik_request(approach)
            self._guarded_move_to_joint_position(joint_angles)
        elif num == 4:
            approach.position.z = approach.position.z + 0.444
            joint_angles = self.ik_request(approach)
            self._guarded_move_to_joint_position(joint_angles)
        elif num == 5:
            approach.position.z = approach.position.z + 0.2
            joint_angles = self.ik_request(approach)
            self._guarded_move_to_joint_position(joint_angles)

    # Retracting arm by moving back to appropriated hover_distance
    # Here we are also retrieving the current pose of the end-effector
    def _retract(self, num):
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
        # Used in real-life running for play move 2
        elif num == 3: 
            ik_pose.position.x = current_pose['position'].x
            ik_pose.position.y = current_pose['position'].y + self._hover_distance + 0.05
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
    def pick(self, pose, num=1, toggle=1):
        if toggle == 1:
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
        if toggle == 2:       
            self._approach(pose, num)
            # servo to pose
            self._servo_to_pose(pose)
            # retract to clear object
            self._retract(num)

    # Place method applying a combination of other methods
    def place(self, pose, num=1):
        # servo above pose
        self._approach(pose, num)
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


### GAZEBO MODEL FUNCTIONS ###

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
def spawn_gazebo_brick(brick_pose, brick_id):

    brick_reference_frame="world"

    # Load Brick SDF
    brick_xml = ''
    with open ("models/brick/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

     # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick" + brick_id, brick_xml, "/", brick_pose, brick_reference_frame)

    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


# Deleting models from gazebo
def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")

        for i in range(12):
            resp_delete = delete_model("brick" + str(i+1))

    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


# Creates a unique identifier for each brick based of it's layer and brick position in the layer
def calc_brick_id(layer, brick):

    return (3 * (layer - 1)) + brick

# Creates and returns a ROS Pose based on input parameters
def create_pose(pos_x, pos_y, pos_z, quat):
    pose = Pose()
    pose.position.x = pos_x
    pose.position.y = pos_y
    pose.position.z = pos_z
    if quat[0] != None:
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

    return pose

# Robust ik_solver, allows multiple attemps to get a solution
def ik_robust(pnp_object, pose):

    number_of_attempts = 5

    for i in range(number_of_attempts):
        result = pnp_object.ik_request(pose)
        if result == "err1": # err1 = Service call failed
            return False
        if result == "err2": # err2 = Invalid Pose
            continue
        else: # ik_request was successful
            return result
    print("IK request tried {} times to find a solution but was unsuccessful".format(number_of_attempts))
    return False # Reached threshold attemps


### MAIN FUNCTION ###

def main():
    """Simple pick and place example"""
    rospy.init_node("ik_pick_and_place_demo")

    pi = 3.14159

    ### OFFSET VARIABLES ###

    hover_distance = 0.1 # meters
    off = 0.32 # Vertical offset - original table sdf was 0.82 for base(0.04) + column(0.04) + surface(0.74)
    gap = 0 # Gap - Distance between placed bricks (AT 0 GAP IS 0.08)
    z_shift = 0 # Adjust z-height for difference in brick width from sim to real world
    x_shift_l1 = 0
    x_shift_l2 = 0
    x_shift_l3 = 0
    x_shift_l4 = 0
    y_shift_l1 = 0
    y_shift_l2 = 0
    y_shift_l3 = 0
    y_shift_l4 = 0

    ### ORIENTATIONS ###

    #----------------------------------------------- roll --- pitch -- yaw

    # This orientation matches that of 'tuck_arms.py -u'
    quat1 = tf.transformations.quaternion_from_euler(pi,      0,     -pi)
    # rotate end-effector 90 degrees about z
    quat2 = tf.transformations.quaternion_from_euler(pi,      0,     -pi/2)
    # rotate end-effector -90 degrees about x
    quat3 = tf.transformations.quaternion_from_euler(-pi/2,   0,     -pi)
    # rotate end-effector -90 degrees about y
    quat4 = tf.transformations.quaternion_from_euler(-pi/2,  -pi/2,  -pi)
    # rotate end-effector 90 degrees about z (from quat4)
    quat5 = tf.transformations.quaternion_from_euler(pi,     -pi/2,   0)


    ### POSES ###

    #----------------------------- x position ----- y position - z position ------------ quaternion

    left_pose        = create_pose(0.5897,          0.3333,      0.3000 - off,           quat2)
    right_pose       = create_pose(0.5797,         -0.3833,      0.2137 - off,           quat1)
    brick_pose       = create_pose(0.5897,          0.3333,      0.8179 - off,          [None])
    pick_pose        = create_pose(0.5896,          0.3333,      0.1836 - off,           quat2)

    calib_pose = {} # Calibration Poses for physical DE NIRO

    calib_pose["1"] = create_pose(0.5896,           0.3333,      0.1836 - off,           quat2)
    calib_pose["2"] = create_pose(0.6596,          -0.07,        0.1836 - off + z_shift, quat2)

    place_pose = {} # Poses for placing each brick (1-12)

    place_pose["1"]  = create_pose(0.6596 + x_shift_l1,        -0.07 + y_shift_l1,         0.1836 - off + z_shift, quat2)
    place_pose["2"]  = create_pose(0.6596 + x_shift_l1,         0.00 + 1*gap + y_shift_l1, 0.1836 - off + z_shift, quat2)
    place_pose["3"]  = create_pose(0.6596 + x_shift_l1,         0.07 + 2*gap + y_shift_l1, 0.1836 - off + z_shift, quat2)

    place_pose["4"]  = create_pose(0.5896 + x_shift_l2,         0.00 + y_shift_l2,         0.2696 - off + 2*z_shift, quat1)
    place_pose["5"]  = create_pose(0.6596 + 1*gap + x_shift_l2, 0.00 + y_shift_l2,         0.2696 - off + 2*z_shift, quat1)
    place_pose["6"]  = create_pose(0.7296 + 2*gap + x_shift_l2, 0.00 + y_shift_l2,         0.2696 - off + 2*z_shift, quat1)

    place_pose["7"] = create_pose(0.6596 + x_shift_l3,        -0.07 + y_shift_l3,         0.3556 - off + 3*z_shift, quat2)
    place_pose["8"] = create_pose(0.6596 + x_shift_l3,         0.00 + 1*gap + y_shift_l3, 0.3556 - off + 3*z_shift, quat2)
    place_pose["9"] = create_pose(0.6596 + x_shift_l3,         0.07 + 2*gap + y_shift_l3, 0.3556 - off + 3*z_shift, quat2)

    place_pose["10"]  = create_pose(0.5896 + x_shift_l4,         0.0 + y_shift_l4,          0.4416 - off + 4*z_shift, quat1)
    place_pose["11"]  = create_pose(0.6596 + 1*gap + x_shift_l4, 0.0 + y_shift_l4,          0.4416 - off + 4*z_shift, quat1)
    place_pose["12"]  = create_pose(0.7296 + 2*gap + x_shift_l4, 0.0 + y_shift_l4,          0.4416 - off + 4*z_shift, quat1)

    clear_pose = {} # Poses used to clear the tower for each layer (1-4)
    
    clear_pose["3"]  = create_pose(0.5897,         0.3333,       0.456 - off,            quat2)
    clear_pose["4"]  = create_pose(0.5897,         0.3333,       0.542 - off,            quat2)

    prep_pose = {} # Preparation poses used to help guide DE NIRO for the upper layers (3-4)

    prep_pose["3"]   = create_pose(0.5897,         0.3333,       0.357 - off,            quat2)
    prep_pose["4"]   = create_pose(0.5896,         0.3333,       0.444 - off,            quat2)

    play_pose = {} # Poses used to pick and place bricks in play

    play_pose["1"]  = create_pose(0.6596 + x_shift_l3,          0.07 + 2*gap + y_shift_l3,    0.3556 - off + z_shift,  quat4) # Pick up brick 6  #0.07 - 2*trim
    play_pose["2"]  = create_pose(0.6596 + x_shift_l3,          0.07 + 2*gap + y_shift_l3,    0.5276 - off + 4*z_shift,  quat4) # Place brick 6 in position 14
    play_pose["3"]  = create_pose(0.5436 + x_shift_l2,          0.05 + 2*gap + y_shift_l2,    0.2696 - off + z_shift,  quat4)

    clear_play = {} # Poses for clearing layer 2 in play

    clear_play["1"]  = create_pose(0.6598,            0.4,         0.5276 - off,    quat3)
    clear_play["1b"]  = create_pose(0.6598,           0.4,         0.5276 - off,    quat4)
    clear_play["1c"]  = create_pose(0.6598,           0.17 + 2*gap,         0.6276 - off,    quat4)
    #clear_play["1d"]  = create_pose(0.5898,           0.4,         0.6276 - off,    quat5)


    ### ACTION SEQUENCE ###

    world_mode = "sim" # sim = gazebo simulation | real = real world
    calibration = False
    build_tower = True
    play_jenga_1 = True
    play_jenga_2 = True

    # Initialise pick and place
    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    # Go to initial position
    left_pnp.move_to_start(ik_robust(left_pnp, left_pose))
    right_pnp.move_to_start(ik_robust(right_pnp, right_pose))

    # Spawn table
    if world_mode == "sim": spawn_gazebo_table()


    if calibration == True:
        left_pnp.move_to(ik_robust(left_pnp, calib_pose["1"]))
        rospy.sleep(100.0)

        '''
        left_pnp.move_to(ik_robust(left_pnp, calib_pose["2"]))
        rospy.sleep(100.0)
        '''


    if build_tower == True:
        # Tower pick and place sequence
        for layer in [1, 2, 3, 4]: # 4 vertical layers in the tower
            for brick in [1, 2, 3]: # 3 bricks in each layer

                brick_id = calc_brick_id(layer, brick)
                brick_id = str(brick_id)
                print("Pick and place for brick {}/12 (layer {}/4, brick {}/3)".format(brick_id, layer, brick))
                
                if world_mode == "sim": spawn_gazebo_brick(brick_pose, brick_id) # Spawns brick and creates a unique id

                print("Picking...")
                left_pnp.pick(pick_pose) # Pick brick up from collection site
                if layer in [3, 4]: left_pnp.move_to(ik_robust(left_pnp, prep_pose[str(layer)])) # Intermediate preparation pose
                print("Placing...")

                left_pnp.place(place_pose[brick_id]) # Place brick on tower
                rospy.sleep(0.01) # For stability

                if layer in [3, 4]: left_pnp.move_to(ik_robust(left_pnp, clear_pose[str(layer)]))  # Intermediate clear pose

                #left_pnp.move_to_start(ik_robust(left_pnp, left_pose)) # Return to initial position

            print("layer {} complete".format(layer))

    
    if play_jenga_1 == True:
        ### MOVE ONE

        rospy.sleep(2.0)

        print("\nPicking...")
        left_pnp.move_to(ik_robust(left_pnp, clear_play["1"])) # Clearance from tower
        left_pnp.move_to(ik_robust(left_pnp, clear_play["1b"])) # Clearance from tower
        left_pnp.pick(play_pose["1"], 2) # Pick up brick 6 from tower
        
        print("\nPlacing...")
        left_pnp.move_to(ik_robust(left_pnp, clear_play["1c"])) # Clearance from tower
        left_pnp.place(play_pose["2"], 1) # Pick up brick 6 from tower

        left_pnp.move_to(ik_robust(left_pnp, clear_play["1c"])) # Clearance from tower

  
    if play_jenga_2 == True:

        rospy.sleep(5.0)

        ### MOVE TWO

        rospy.sleep(5.0)

        left_pnp.move_to(ik_robust(left_pnp, clear_play["1"])) # Clearance from tower
        left_pnp.move_to(ik_robust(left_pnp, clear_play["1b"])) # Clearance from tower
        left_pnp.pick(play_pose["3"], 2, 2)

    
    rospy.sleep(10.0)


    # Delete all models
    if world_mode == "sim": delete_gazebo_models()


if __name__ == '__main__':
    sys.exit(main())
    #delete_gazebo_models()
