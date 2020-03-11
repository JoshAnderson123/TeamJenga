# Team Jenga

Design Engineering 2019/2020 Robotics Group Project. Our mission was to use the DE NIRO robot to build a Jenga tower and play two moves against a human player. This documentation provides class and function descriptions for the code we used in this project, which can be found in _jenga_newstrat_original_sim_3.py_:

https://github.com/JoshAnderson123/TeamJenga/blob/master/jenga_newstrat_original_sim_3.py

#### Project Members:
 * Marcus Melconian
 * Ben Collis
 * Hannah Qureshi
 * Puja Soneji
 * Joshua Anderson
 * Bettina Sosa

<br>

## class PickAndPlace(object):
    
The PickAndPlace class enables DE NIRO to move his limbs. Each PickAndPlace object is initialised with a single limb that it can control.

##### Methods:

* `__init__()`
* `move_to_start()`
* `ik_request()`
* `_guarded_move_to_joint_position()`
* `gripper_open()`
* `gripper_close()`
* `_approach()`
* `_retract()`
* `_servo_to_pose()`
* `pick()`
* `place()`
* `move_to()`

<br>

### PickAndPlace.\_\_init\_\_(self, limb, hover_distance, verbose):
        
Initialises a PickAndPlace object. Each PickAndPlace object can control one limb. 

##### Parameters:

* `limb` _String_ - which limb you want to control. 'left' is for the left arm and 'right' is for the right arm
* `hover_distance` _Float_ - the distance (in meters) at which the end effector hovers above the desired position. 
* `verbose` _Bool_ - provides additional information to the console if set to True

<br>

### PickAndPlace.move_to_start(self, start_angles=None):
    
Moves the PickAndPlace object's limb to the starting pose.

##### Parameters:

* `start_angles` _[Int]_ - the set of joint angles that make the starting pose.
 
<br>
    
### PickAndPlace.ik_request(self, pose):

Uses Inverse Kinematics to return the joint angles for a specified end-effector pose

##### Parameters:

* `pose` _geometry_msgs.msg.Pose_ - the desired end-effector pose

##### Returns:

[Float] - a list of joint angles to achieve the desired end-effector pose

##### Exceptions:

* `Service call failed` - could not communicate with ROS service
* `Inavlid pose` - pose outside the robots dexterous workspace

<br>

### PickAndPlace._guarded_move_to_joint_position(self, joint_angles):
        
Moves the PickAndPlace object's limb to the pose with the specified joint_angles

##### Parameters:

* `joint_angles` _[Float]_ - the joint angles for the desired end-effector pose

<br>

### PickAndPlace.gripper_open(self):
        
Opens the PickAndPlace object's gripper
        
<br>

### PickAndPlace.gripper_close(self):
        
Closes the PickAndPlace object's gripper
        
<br>

### PickAndPlace._approach(self, pose, num):
        
Moves the PickAndPlace object's limb to the specified pose, with the hover_distance offset applied to the z-axis.

##### Parameters:

* `pose` _geometry_msgs.msg.Pose_ - the desired end-effector pose
* `num` _Int_ - the approach configuration number (From 1-5). This changes what axis the hover_distance is applied to
  
<br>

### PickAndPlace._retract(self, num):
        
Retracts the PickAndPlace object's limb by the objects global hover distance

Parameters:

* `num` _Int_ - the approach configuration number (From 1-5). This changes what axis the hover_distance is applied to
        
<br>

### PickAndPlace._servo_to_pose(self, pose):
        
Moves the PickAndPlace object's limb to the specified pose

##### Parameters:

* `pose` _geometry_msgs.msg.Pose_ - the desired end-effector pose
        
<br>

### PickAndPlace.pick(self, pose, num=1, toggle=1):
        
Picks up a brick in the specified location

Parameters:

* `pose` _geometry_msgs.msg.Pose_ - the end-effector location to pick the brick
* `num` _Int_ - the approach configuration number (From 1-5). This changes what axis the hover_distance is applied to
* `toggle` _Bool_ - toggles if the gripper initially opens or not
        
<br>

### PickAndPlace.place(self, pose, num=1):
        
Places a brick in the specified location

Parameters:

* `pose` _geometry_msgs.msg.Pose_ - the end-effector location to pick the brick
* `num` _Int_ - the approach configuration number (From 1-5). This changes what axis the hover_distance is applied to
  
<br>
  
### PickAndPlace.move_to(self, joint_angles):
        
Moves the PickAndPlace objects limb to the location specified by the joint angles

Parameters:

* `joint_angles` _[Float]_ - the set of joint angles for the requested pose
       
<br>

### spawn_gazebo_table(table_pose, table_reference_frame):
   
Loads and Spawns table in the Gazebo siulator

Parameters:

* `table_pose` _geometry_msgs.msg.Pose_ - the position of the table
* `table_reference_frame` _String_ - the reference frame for the table

Exceptions:

* `ServiceException` - unable to call service

<br>

### spawn_gazebo_brick(brick_pose, brick_id):
    
Spawns a brick with a predefined pose

Parameters:

* `brick_pose` _geometry_msgs.msg.Quaternion_ - the pose to spawn the brick
* `brick_id` _Int_ - the unique identifier for the brick

<br>

### delete_gazebo_models():
   
Deletes all models from the Gazebo simulator.

<br>

### calc_brick_id(layer, brick):
    
Creates a unique identifier for each brick based of it's layer and brick position in that layer

##### Parameters:

* `layer` _Int_ - the layer that the brick resides in
* `brick` _Int_ - the position of the brick within the layer

<br>

### create_pose(pos_x, pos_y, pos_z, quat):
   
Creates and returns a ROS Pose based on position and rotation parameters.

Parameters:

* `pos_x`, `pos_y`, `pos_z` _Int_ - x, y, z components of requested position (in meters)
* `quat` _geometry_msgs.msg.Pose_ - quaternion representation of requested orientation

<br>

### ik_robust(pnp_object, pose):
    
Robust wrapper for PickAndPlace.ik_request(). Allows multiple attemps to find an IK solution, since ik_request() has a 1.5% chance of returning an Invalid Pose error even though the pose is valid.

Parameters:

* `pnp_object` _PickAndPlace_ - the PickAndPlace object to apply robust wrapper to
* `pose` _geometry_msgs.msg.Pose_ - the desired end-effector pose
    
<br>

### main():
    
DE NIRO plays Jenga! There are four sequences which can be toggled on and off:

1 - Calibration <br>
2 - Building Jenga Tower <br>
3 - DE NIRO move 1 <br>
4 - DE NIRO move 2 <br>
