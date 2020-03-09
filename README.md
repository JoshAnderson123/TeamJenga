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



### PickAndPlace.__init__(self, limb, hover_distance, verbose):
        
Initialises a PickAndPlace object. Each PickAndPlace object can control one limb. 

##### Parameters:

* `limb` _(String)_ - which limb you want to control. 'left' is for the left arm and 'right' is for the right arm
* `hover_distance` _(float)_ - the distance (in meters) at which the end effector hovers above the desired position. 
* `verbose` _(Bool)_ - provides additional information to the console if set to True



#### PickAndPlace.move_to_start(self, start_angles=None):
    
Moves the PickAndPlace object's limb to the starting pose.

    Parameters:
        start_angles - the set of joint angles that make the starting pose.
 
 
    
#### PickAndPlace.ik_request(self, pose):

Uses Inverse Kinematics to return the joint angles for a specified end-effector pose

    Parameters:

        pose - the desired end-effector pose

    Returns:

        [float] - a list of joint angles to achieve the desired end-effector pose

    Exceptions:

        Service call failed - could not communicate with ROS service
        Inavlid pose - pose outside the robots dexterous workspace
  

#### PickAndPlace._guarded_move_to_joint_position(self, joint_angles):
        
            Moves the PickAndPlace object's limb to the pose with the specified joint_angles
        

#### PickAndPlace.gripper_open(self):
        
            Opens the PickAndPlace object's gripper
        

#### PickAndPlace.gripper_close(self):
        
            Closes the PickAndPlace object's gripper
        

#### PickAndPlace._approach(self, pose, num):
        
            Moves the PickAndPlace object's limb to the specified pose, with the
            hover_distance offset applied to the z-axis.

            Parameters:

                pose - the desired end-effector pose
                num - the approach configuration number (From 1-5). This changes what axis the
                    hover_distance is applied to
        


### PickAndPlace._retract(self, num):
        
            Retracts the PickAndPlace object's limb by the objects global hover distance

            Parameters:

                num - the approach configuration number (From 1-3). This changes what axis the
                    hover_distance is applied to
        


### PickAndPlace._servo_to_pose(self, pose):
        
            Moves the PickAndPlace object's limb to the specified pose
        


### PickAndPlace.pick(self, pose, num=1, toggle=1):
        
            Picks up a brick in the specified location

            Parameters:

                pose - the end-effector location to pick the brick
                num - the approach configuration number (From 1-5). This changes what axis the
                    hover_distance is applied to
                toggle - toggles if the gripper initially opens or not
        

### PickAndPlace.place(self, pose, num=1):
        
            Places a brick in the specified location

            Parameters:

                pose - the end-effector location to place the brick
                num - the approach configuration number (From 1-5). This changes what axis the
                    hover_distance is applied to
        


### PickAndPlace.move_to(self, joint_angles):
        
            Moves the PickAndPlace objects limb to the location specified by the joint angles

            Parameters:

                joint_angles - the set of joint angles for the requested pose
       


### GAZEBO MODEL FUNCTIONS ###


### def spawn_gazebo_table(table_pose=Pose(position=Point(x=0.95, y=0.0, z=0.0)),
                       table_reference_frame="world"):
   
        Loads and Spawns table in the Gazebo siulator

        Parameters:

            table_pose - the position of the table
            table_reference_frame - the reference frame for the table

        Exceptions:

            ServiceException - unable to call service
    


### def spawn_gazebo_brick(brick_pose, brick_id):
    
        Spawns a brick with a predefined pose

        Parameters:

            brick_pose - the pose to spawn the brick
            brick_id - the unique identifier for the brick
    


### def delete_gazebo_models():
   
        Deletes all models from the Gazebo simulator.
    


### def calc_brick_id(layer, brick):
    
        Creates a unique identifier for each brick based of it's layer and brick position in that layer
    


### def create_pose(pos_x, pos_y, pos_z, quat):
   
        Creates and returns a ROS Pose based on position and rotation parameters.

        Parameters:

            pos_x, pos_y, pos_z - x, y, z components of requested position (in meters)
            quat - quaternion representation of requested orientation
    


### def ik_robust(pnp_object, pose):
    
        Robust wrapper for PickAndPlace.ik_request(). Allows multiple attemps to find an IK solution,
        since ik_request() has a 1.5% chance of returning an Invalid Pose error even though the pose
        is valid.

        Parameters:

            pnp_object - the PickAndPlace object to apply robust wrapper to
            pose - the desired end-effector pose
    


### MAIN FUNCTION ###

### def main():
    
        DE NIRO plays Jenga! There are four sequences which can be toggled on and off:

        1 - Calibration
        2 - Building Jenga Tower
        3 - DE NIRO move 1
        4 - DE NIRO move 2
