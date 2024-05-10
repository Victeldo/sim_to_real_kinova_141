#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import math
import os
import time
import threading
import numpy as np
import robosuite as suite
from stable_baselines3 import SAC
from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

import matplotlib.pyplot as plt

controller_config = load_controller_config(default_controller="JOINT_POSITION")

# create environment instance
env = suite.make(
    env_name="Stack", # try with other tasks like "Stack" and "Door"
    robots="Kinova3",  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=False,
    controller_configs=controller_config,
    has_offscreen_renderer=False,
    use_camera_obs=False,
)
wrapped_env = GymWrapper(env)

def mean_square_error(arr1, arr2):
    if arr1.shape != arr2.shape:
        raise ValueError("Mismatched arrays")
    
    mse = np.mean(np.square(arr1-arr2), axis=1)
    return mse

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 100

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def populateAngularPose(jointPose,durationFactor):
    waypoint = Base_pb2.AngularWaypoint()
    waypoint.angles.extend(jointPose)
    waypoint.duration = durationFactor*5.0    
    
    return waypoint

def goto_position(base, traj):

    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    waypoints = Base_pb2.WaypointList()    
    waypoints.duration = 5.0
    waypoints.use_optimal_blending = False

    traje = []
    for i in range(7):
        traje.append(math.degrees(math.asin(traj[i])))
    traje = tuple(traje)

    waypoint = waypoints.waypoints.add()
    waypoint.name = "startpos"
    durationFactor = 5
    
    waypoint.angular_waypoint.CopyFrom(populateAngularPose(traje, durationFactor))
    
   # Verify validity of waypoint
    result = base.ValidateWaypointList(waypoints);
    if(len(result.trajectory_error_report.trajectory_error_elements) == 0):

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Reaching angular pose trajectory...")
        
        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished
    else:
        print("Error found in trajectory") 
        print(result.trajectory_error_report)
        return finished

def angular_action_movement(base, actions):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()
    i = 0
    
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value +=  (math.degrees(actions[i]))
        i += 1

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def SendGripperCommand(base, move):
        
        # fix the move (from (-1,1) where -1 is open and 1 is closed
        #                   to (0,1) where 1 is closed and 0 is open

        normalized_move = -move
        # sign = (move < 0)
        # if sign:
            

        # get current gripper position via request
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        gripper_m_pos = gripper_measure.finger[0].value

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION

        tempval = gripper_m_pos + normalized_move
        tempval = min(tempval, 1)
        finger.value = max(tempval, 0)
        base.SendGripperCommand(gripper_command)

def dSendGripperCommand(base):

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.value = 0.5
        base.SendGripperCommand(gripper_command)      

def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Move the arm back to the home position and gripper
        success = True
        success &= move_to_home_position(base)
        dSendGripperCommand(base)
        
        # Train the model (Sample Model for demonstration purposes)
        model = SAC("MlpPolicy", wrapped_env, verbose=1)
        model.learn(total_timesteps=100, log_interval=4)

        # Move the arm to the starting point in the simulated environment
        obs = wrapped_env.reset()
        obs_unwrapped = wrapped_env.unwrapped
        obs_unwrapped = obs_unwrapped.reset()
        success &= goto_position(base, obs_unwrapped["robot0_joint_pos_sin"])

        input_joint_angles = base.GetMeasuredJointAngles()

        # for evaluation purposes
        measured_angles = []
        virtual_angles = []

        # Sim to real for making the real robot mimic the simulated robot (if want to use real robot and not simulation
        # there is a need for using the real robot's observations, especially for the objects of interest)
        

        for i in range(1000):
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = wrapped_env.step(action)  # take action in the environment
            
            # # Get representation of robot
            obs_unwrapped = wrapped_env.unwrapped
            obs_unwrapped = obs_unwrapped.reset()

            # Angular joint position movement doesn't account for current position otherwise
            passed_Action = []
            tmp_Action = []

            for j in range(7):
                passed_Action.append(action[j] + math.asin(obs_unwrapped["robot0_joint_pos_sin"][j]))
                tmp_Action.append(math.degrees(action[j] + math.asin(obs_unwrapped["robot0_joint_pos_sin"][j])))
            
            virtual_angles.append(tmp_Action)
            
            input_joint_angles = base.GetMeasuredJointAngles()
            temparr = []
            for joint_angle in input_joint_angles.joint_angles:
                temparr.append(joint_angle.value)
            measured_angles.append(temparr)

            success &= angular_action_movement(base, passed_Action)
            print(i)
            if done:
                success &= move_to_home_position(base)
                obs = wrapped_env.reset()
                obs_unwrapped = wrapped_env.unwrapped
                obs_unwrapped = obs_unwrapped.reset()
                success &= goto_position(base, obs_unwrapped["robot0_joint_pos_sin"])
        
        measured_angles_np = np.array(measured_angles)
        virtual_angles_np = np.array(virtual_angles)
        mse = mean_square_error(measured_angles_np, virtual_angles_np)
        
        plt.plot(mse)
        plt.title('Mean Square Error between real and simulated joint angles')
        plt.xlabel('Time Step')
        plt.ylabel('Mean Square Error')
        plt.show()

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())

