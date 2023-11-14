#!/usr/bin/env python

#This Python file is used to go through the RobotStates





#_____________________IMPORTS_____________________
#Importing needed libraries, msg and srv types
import rospy, actionlib, time
from moveit_python import MoveGroupInterface
from ibo1_ros360.msg import StateMachine, ObjectPosition, DoneWithState
from dynamic_reconfigure.msg import Config, IntParameter
from moveit_msgs.msg import MoveItErrorCodes
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal, 
							  FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ibo1_ros360.srv import StateMachineService
from dynamic_reconfigure.srv import Reconfigure





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
robotState = -1
previousRobotState = -1
stateMachine_service = None
rgb_service = None
objectArea = 0
doneApproachingTarget = False
donePickingUp = False
doneDropping = False
doneSearchingNextTarget = False
joint_names = ["shoulder_pan_joint",
                "shoulder_lift_joint", "upperarm_roll_joint",
                "elbow_flex_joint", "forearm_roll_joint",
                "wrist_flex_joint", "wrist_roll_joint"]
head_joint_names = ["head_pan_joint", "head_tilt_joint"]
torso_joint_names = ["torso_lift_joint"]
originalPose = [1.3200007983036732, 1.3999813261835135, -0.19999156896589554, 1.7200002648456447,
				0, 1.65997797158587, 0]
move_group = None
cubes = {
	"numberOfCubesAlreadyPickedUp": 0,
	"maxCubesToPickUp": 2
}
timeRun = {
	"start": 0,
	"finish": 0
}





#_____________________CALLBACK FUNCTIONS_____________________

#callback function for StateMachine
def callbackStateMachine(receivedMsg):
	global robotState
	
	robotState = receivedMsg.currentState





#callback function for foundObject
def callbackObjectPosition(receivedMsg):
	global objectArea

	objectArea = receivedMsg.area





#callback function for approachingTarget
def callbackApproachTarget(receivedMsg):
	global doneApproachingTarget
	
	doneApproachingTarget = receivedMsg.doneWithState





#callback function for pickUpObject
def callbackPickUpObject(receivedMsg):
	global donePickingUp
	
	donePickingUp = receivedMsg.doneWithState





#callback function for droppingIntoBin
def callbackDroppingIntoBin(receivedMsg):
	global doneDropping
	
	doneDropping = receivedMsg.doneWithState





#callback function for searchingForNextTarget
def callbackSearchingForNextTarget(receivedMsg):
	global doneSearchingNextTarget
	
	doneSearchingNextTarget = receivedMsg.doneWithState





#_____________________METHODS_____________________

#send request to StateMachineService to change robotState
def changeRobotState(changeRobotStateTo):
	global stateMachine_service

	response = stateMachine_service(int(changeRobotStateTo))
	if not response:
		rospy.logerr("ERR: RobotState wasnt set correctly")





#Tilting head
def resetCamera():
	global head_joint_names

	#rospy.loginfo("Waiting for head_controller...")
	head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	head_client.wait_for_server()
	#rospy.loginfo("...connected.")

	if rospy.is_shutdown():
		return

	trajectory = JointTrajectory()
	trajectory.joint_names = head_joint_names
	trajectory.points.append(JointTrajectoryPoint())

	pose = [0, 0]
	trajectory.points[0].positions = pose
	trajectory.points[0].velocities = [0.0] * len(pose)
	trajectory.points[0].accelerations = [0.0] * len(pose)
	trajectory.points[0].time_from_start = rospy.Duration(0.0)

	head_goal = FollowJointTrajectoryGoal()
	head_goal.trajectory = trajectory
	head_goal.goal_time_tolerance = rospy.Duration(0.0)

	#rospy.loginfo("Setting positions...")
	head_client.send_goal(head_goal)

	head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
	return True





#function to move only the torso
def resetTorso():
	global torso_joint_names


	#rospy.loginfo("Waiting for torso_controller...")
	torso_client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	torso_client.wait_for_server()

	#Setting up the initialization for moving
	trajectory = JointTrajectory()
	trajectory.joint_names = torso_joint_names
	trajectory.points.append(JointTrajectoryPoint())
	trajectory.points[0].positions = [0.0]
	trajectory.points[0].velocities = [0.06]
	trajectory.points[0].accelerations = [0.01]
	trajectory.points[0].time_from_start = rospy.Duration(0.0)

	#Creating the goal and its initialization
	torso_goal = FollowJointTrajectoryGoal()
	torso_goal.trajectory = trajectory
	torso_goal.goal_time_tolerance = rospy.Duration(0.0)

	#Moving to the end_point
	torso_client.send_goal(torso_goal) #optioanly add callbacks, note fixed order: goalDone_cb, active_cb, feedback_cb)

	#Waiting till I reached the end point
	torso_client.wait_for_result(rospy.Duration(1.0))  # specify timeout on waiting
	return True





#Reset Arm
def resetArm():
	global move_group, joint_names, originalPose

	joint_names, originalPose

	arrivedAtPosition = False
	
	move_group.moveToJointPosition(joint_names, originalPose, wait=True)

	move_group.get_move_action().wait_for_result()
	result = move_group.get_move_action().get_result()

	if result:
        # Checking the MoveItErrorCode
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			arrivedAtPosition = True
		else:
			# If you get to this point please search for:
			# moveit_msgs/MoveItErrorCodes.msg
			rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
			arrivedAtPosition = False
	else:
		rospy.logerr("MoveIt! failure no result returned.")
	
	return arrivedAtPosition





#Function to close grippers
def resetGripper():

	gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
	gripper_client.wait_for_server()

	gripper_goal = GripperCommandGoal()
	gripper_goal.command.max_effort = 10.0
	gripper_goal.command.position = 1.0

	#rospy.loginfo("Setting positions open...")
	gripper_client.send_goal(gripper_goal)
	gripper_client.wait_for_result(rospy.Duration(5.0))

	return True





#Reset Robot
def resetRobot():
	rospy.logwarn("Resetting Robot!")
	finishedResettingArm = False
	finishedResettingCamera = False
	finishedResettingTorso = False
	finishedResettingGripper = False

	rospy.logwarn("Resetting Arm")
	finishedResettingArm = resetArm()
	rospy.logwarn("Resetting Camera")
	finishedResettingCamera = resetCamera()
	rospy.logwarn("Resetting Torso")
	finishedResettingTorso = resetTorso()
	rospy.logwarn("Resetting Gripper")
	finishedResettingGripper = resetGripper()

	if finishedResettingArm and finishedResettingCamera and finishedResettingTorso and finishedResettingGripper:
		return True
	else:
		return False





#Changing color filter
def changeColorFilter(color):
	global rgb_service

	config = Config()
	colorValue = [0,0,0,0]

	#If we want to switch to green
	if color == "green":
		colorValue[0] = 0
		colorValue[1] = 10
		colorValue[2] = 90
		colorValue[3] = 255

	#If we want to switch to red
	elif color == "red":
		colorValue[0] = 90
		colorValue[1] = 255
		colorValue[2] = 0
		colorValue[3] = 10

	#Updating red limit min
	int_param = IntParameter()
	int_param.name = "r_limit_min"
	int_param.value = colorValue[0]
	config.ints.append(int_param)

	#Updating red limit max
	int_param = IntParameter()
	int_param.name = "r_limit_max"
	int_param.value = colorValue[1]
	config.ints.append(int_param)

	#Updating green limit min
	int_param = IntParameter()
	int_param.name = "g_limit_min"
	int_param.value = colorValue[2]
	config.ints.append(int_param)

	#Updating green limit max
	int_param = IntParameter()
	int_param.name = "g_limit_max"
	int_param.value = colorValue[3]
	config.ints.append(int_param)


	response = rgb_service(config)

	return True





#main logic to go through the stateMachine
def stateMachineLogic():
	global previousRobotState, robotState, objectArea, doneApproachingTarget
	global donePickingUp, doneDropping, doneSearchingNextTarget, cubes, timeRun

	#rospy.loginfo("robotState: %s - previous: %s",robotState, previousRobotState)

	#Checking if robot is in state -1
	if robotState == -1 and previousRobotState == -1:
		if resetRobot():
			if changeColorFilter("red"):
				rospy.logwarn("Drive till object found")
				changeRobotState(0)
				previousRobotState = 0
	
	#Checking if robot is in state 0 and prevRobotState was 0 and objectArea is above or equal to 30,
	#is a value decided by myself
	elif robotState == 0 and previousRobotState == 0 and objectArea > 30:
		rospy.logdebug("Approach the target")
		changeRobotState(1)
		previousRobotState = 1

	#Checking if robot is in state 1 and prevRobotState was 1 and done Approaching
	elif robotState == 1 and previousRobotState == 1 and doneApproachingTarget:
		rospy.logwarn("Pickup the target")
		doneApproachingTarget = False
		changeRobotState(2)
		previousRobotState = 2

	#Checking if robot is in state 2 and prevRobotState was 2 and done pickingUp
	elif robotState == 2 and previousRobotState == 2 and donePickingUp:
		rospy.logwarn("Go drop the cube")
		#Changing Color Filter for green
		if changeColorFilter("green"):
			donePickingUp = False
			changeRobotState(3)
			previousRobotState = 3
	
	#Checking if robot is in state 2 and prevRobotState was 2 and done dropping
	elif robotState == 3 and previousRobotState == 3 and doneDropping:
		
		#Increase the dropped cubes
		cubes["numberOfCubesAlreadyPickedUp"] += 1

		#Checking if all cubes have been picked up
		if cubes["numberOfCubesAlreadyPickedUp"] < cubes["maxCubesToPickUp"]:

			rospy.logwarn("Find next target")
			#Changing Color Filter for red
			if changeColorFilter("red"):
				#Sleep one second to give the cube time to drop
				time.sleep(1)
				
				doneDropping = False
				changeRobotState(4)
				previousRobotState = 4
		
		#Otherwise say we finished with the time and set state to a nonStart state
		else:
			timeRun["finish"] = time.time()

			#Calculating time and converting to minutes and seconds to be printed
			finishedTime = (timeRun["finish"] - timeRun["start"])/60
			finishedTimeMinutes = int(finishedTime) 
			finishedTimeSecondsPart = int((finishedTime - finishedTimeMinutes)*60)
			if finishedTimeSecondsPart < 10:
				finishedTimeSecondsPart = "0" +str(finishedTimeSecondsPart)
			rospy.logwarn("Done picking up all cubes with the Time[min]: %s:%s"
						  ,finishedTimeMinutes, finishedTimeSecondsPart)
						  
			changeRobotState(-2)
			previousRobotState = -2

	#Checking if robot is in state 2 and prevRobotState was 2 and done serachingNextTarget
	elif robotState == 4 and previousRobotState == 4 and doneSearchingNextTarget:
		rospy.logwarn("Found next target!")
		doneSearchingNextTarget = False
		changeRobotState(1)
		previousRobotState = 1
		

	



#Function used to setup all publisher subscribers and so on
def setup():
	#Accessing global variables
	global stateMachine_service, rgb_service, robotState
	global previousRobotState, move_group, timeRun

	timeRun["start"] = time.time()

	#Initiating node
	rospy.init_node('main', anonymous=True)

	#Subscribing to /stateMachine to get the current state of the Robot
	rospy.Subscriber('/stateMachine', StateMachine, callbackStateMachine)

	#Subscribing to /objectPosition to get the current position of the Object
	rospy.Subscriber('/objectPosition', ObjectPosition, callbackObjectPosition)

	#Subscribing to /objectPosition to get the current position of the Object
	rospy.Subscriber('/approachTargetStatus', DoneWithState, callbackApproachTarget)

	#Subscribing to /objectPosition to get the current position of the Object
	rospy.Subscriber('/pickUpObjectStatus', DoneWithState, callbackPickUpObject)

	#Subscribing to /droppingIntoBinStatus to get the current position of the Object
	rospy.Subscriber('/droppingIntoBinStatus', DoneWithState, callbackDroppingIntoBin)

	#Subscribing to /searchingForNextTargetStatus to get the current position of the Object
	rospy.Subscriber('/searchingForNextTargetStatus', DoneWithState, callbackSearchingForNextTarget)

	#Adding the stateMachine_setState service, to change the state the robot is in
	rospy.wait_for_service("/stateMachine_setState")
	stateMachine_service = rospy.ServiceProxy('/stateMachine_setState', StateMachineService)

	#Adding color filter service, to change the color filter
	rospy.wait_for_service('rgb_color_filter/set_parameters')
	rgb_service = rospy.ServiceProxy('/rgb_color_filter/set_parameters', Reconfigure)

	changeRobotState(-1)
	previousRobotState = -1

	#Setting move_group
	move_group = MoveGroupInterface("arm", "base_link")

	#Setting refresh rate
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		stateMachineLogic()
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass