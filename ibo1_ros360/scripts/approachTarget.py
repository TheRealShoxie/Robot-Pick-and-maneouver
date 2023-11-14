#!/usr/bin/env python

#This Python file is used for approaching a target





#_____________________IMPORTS_____________________
#Importing needed libraries, msg and srv types
import rospy, actionlib, time
from geometry_msgs.msg import Twist
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ibo1_ros360.msg import SensorDirection, StateMachine, ObjectPosition, DoneWithState
from ibo1_ros360.srv import SensorDirectionService





#_____________________GLOBAL VARIABLES_____________________
#Defining global variables
robotState = -1
target = {
	"xPos": -1.0,
	"yPos": -1.0,
	"area": -1.0,
	"length": -1.0,
	"distanceToCenterX": -1.0,
	"distanceToCenterY": -1.0,
	"distanceToBottomY": -1.0,
	"depth": -1.0
}
imageDimensions = {
	"xMax": 640,
	"yMax": 480
}
previousStates = {
	"velocity": 0.0,
	"angular": 0.0,
	"torso": [0.0],
	"tiltLvl": 0.0,
	"doneTrsoAlignment": False
}
approachState = {
	"current": -1
}
sensorData = {
	"left": 25,
	"forward": 25,
	"right": 25
}
sensorThreshold = {
	"left": 300,
	"forwardMin": 300,
	"forwardMax": 360,
	"right": 360
}
head_joint_names = ["head_pan_joint", "head_tilt_joint"]
torso_joint_names = ["torso_lift_joint"]
sensorDirection_service = None
showDebug = False





#_____________________CALLBACK FUNCTIONS_____________________

# callback function for contour_moments
#callback function for StateMachine
def callbackStateMachine(receivedMsg):
	global robotState
	
	robotState = receivedMsg.currentState





#callback function for LaserScan
def callbackSensorData(receivedMsg):
	#Making global variables usable
	global sensorData

	sensorData["left"] = receivedMsg.thresholdLeft
	sensorData["forward"] = receivedMsg.thresholdForward
	sensorData["right"] = receivedMsg.thresholdRight





#callback function for foundObject
def callbackObjectPosition(receivedMsg):
	global target

	target["xPos"] = receivedMsg.xPos
	target["yPos"] = receivedMsg.yPos
	target["area"] = receivedMsg.area
	target["length"] = receivedMsg.length
	target["distanceToCenterX"] = receivedMsg.distanceToCenterX
	target["distanceToCenterY"] = receivedMsg.distanceToCenterY
	target["distanceToBottomY"] = receivedMsg.distanceToBottomY
	target["depth"] = receivedMsg.depth





#_____________________METHODS_____________________

#Send done msg
def sendDone(publishDone, isDone):
	#Create ApproachTarget msg and send that we are done
	sendingMsg = DoneWithState()
	sendingMsg.doneWithState = isDone
	publishDone.publish(sendingMsg)





#send request to SensorDirectionService to change sensorThresholds
def changeSensorValues(sensorDirection_service):
	global sensorThreshold

	try:
		response = sensorDirection_service(int(sensorThreshold["left"]), int(sensorThreshold["forwardMin"]), 
										   int(sensorThreshold["forwardMax"]), int(sensorThreshold["right"]))
		if str(response.response[1]).lower() == "true":
			return True
		else:
			rospy.logerr("Couldnt update Thresholds with following msg:\n %s", response.response[0])
			return False
	except rospy.ServiceException as e:
		print("Service call failed: %s", e)






#Tilting head
def tiltHead(tiltLvl):
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

	pose = [0, tiltLvl]
	trajectory.points[0].positions = pose
	trajectory.points[0].velocities = [0.0] * len(pose)
	trajectory.points[0].accelerations = [0.0] * len(pose)
	trajectory.points[0].time_from_start = rospy.Duration(0.0)

	head_goal = FollowJointTrajectoryGoal()
	head_goal.trajectory = trajectory
	head_goal.goal_time_tolerance = rospy.Duration(0.0)

	#rospy.loginfo("Setting positions...")
	head_client.send_goal(head_goal)

	head_client.wait_for_result(rospy.Duration(0.1))  # specify timeout on waiting
	#rospy.loginfo("...done")





#function to move only the torso
def moveTorso(pose):
	global torso_joint_names

	torso_client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	torso_client.wait_for_server()

	#Setting up the initialization for moving
	trajectory = JointTrajectory()
	trajectory.joint_names = torso_joint_names
	trajectory.points.append(JointTrajectoryPoint())
	trajectory.points[0].positions = pose
	trajectory.points[0].velocities = [0.0] * len(pose)
	trajectory.points[0].accelerations = [0.0] * len(pose)
	trajectory.points[0].time_from_start = rospy.Duration(0.0)

	#Creating the goal and its initialization
	torso_goal = FollowJointTrajectoryGoal()
	torso_goal.trajectory = trajectory
	torso_goal.goal_time_tolerance = rospy.Duration(0.0)

	#Moving to the end_point
	#rospy.loginfo("Setting positions...")
	torso_client.send_goal(torso_goal) #optioanly add callbacks, note fixed order: goalDone_cb, active_cb, feedback_cb)

	#Waiting till I reached the end point
	torso_client.wait_for_result(rospy.Duration(1.0))





#Function for Angular Alignment
def angularAlignment(sendingMsg):
	global target, imageDimensions, previousStates

	doneAngularAlignment = False

	#Checking if the target is still outside distance to center in x
	if abs(target["distanceToCenterX"]) >= 60:

		#Calculating angular. Angular = (distanceCenterX / (imageHeight/2))*maxSpeed
		angular = (target["distanceToCenterX"] / (imageDimensions["xMax"]/2)) * 1

		#Checking if difference between old and new angular are bigger than certain threshhold
		if abs(abs(angular) - previousStates["angular"]) >= 0.1:
			sendingMsg.angular.z = angular
			previousStates["angular"] = angular
		else:
			sendingMsg.angular.z = previousStates["angular"]

	#If within center do not move angularly
	else:
		angular = 0.0
		previousStates["angular"] = 0.0
		sendingMsg.angular.z = angular
		doneAngularAlignment = True
	
	return [sendingMsg, doneAngularAlignment]





#Function fo Camera Alignment
def cameraAlignment():
	global target, previousStates

	doneCameraAlignment = False

	#Checking if the target is still outside distance to the center of y
	if abs(target["distanceToCenterY"]) >= 40:


		#Checking if distance to center Y greater 0 then move tiltLvl down
		if target["distanceToCenterY"] >= 0 and previousStates["tiltLvl"] >= -0.74:
			previousStates["tiltLvl"] -= 0.02
		
		elif previousStates["tiltLvl"] <= 1.43:
			previousStates["tiltLvl"] += 0.02

		tiltHead(previousStates["tiltLvl"])

	#Otherwise set CameraAlignment to done = True
	else:
		doneCameraAlignment = True

	return doneCameraAlignment





#Torso alignment
def torsoAlignment(threshold):
	global target, previousStates

	doneTorsoAlignment = False

	#Checking if the target is still too high or too low
	if target["distanceToCenterY"] >= threshold:

		#Checking if distance to Object bigger than value then increase height of torso
		if target["distanceToCenterY"] >= threshold and previousStates["torso"][0] <= 0.58:
			previousStates["torso"][0] = previousStates["torso"][0] + 0.02
		
		#Else if smaller than value decrease height of torso
		elif target["distanceToCenterY"] < threshold and previousStates["torso"][0] >= 0.02:
			previousStates["torso"][0] = previousStates["torso"][0] - 0.02

		#Checking if we reached threhold if yes return true
		elif previousStates["torso"] == 0.6 or previousStates["torso"] == 0:
			doneTorsoAlignment = True

		moveTorso(previousStates["torso"])

	#Otherwise set done Torso Alignment
	else:
		if previousStates["torso"][0] + 0.1 < 0.6:
			previousStates["torso"][0] = previousStates["torso"][0] +0.1
			moveTorso(previousStates["torso"])
		doneTorsoAlignment = True
	
	return doneTorsoAlignment





#Driving closer to target
def moveCloserToTarget(pubCmdVel):
	global target, imageDimensions, previousStates, sensorData, approachState

	#Creating the msg to be send
	sendingMsg = Twist()
	doneAngularAlignment = False
	doneDistanceAlignment = False
	doneCameraAlignment = False

	#Calling align angular and assigning returned values
	returnOfFunctionAngular = angularAlignment(sendingMsg)
	sendingMsg = returnOfFunctionAngular[0]
	doneAngularAlignment = returnOfFunctionAngular[1]

	#Checking if the target is still outside of 2m
	if sensorData["forward"] >= 2.5:

		#Calculate velocity with following formular: (distance/maxRange)*maxSpeed
		velocity = (sensorData["forward"]/6)*1

		#Checking if difference between old and new velocity are bigger than certain threshhold
		if abs(abs(velocity) - previousStates["velocity"]) >= 0.03:
			sendingMsg.linear.x = velocity
			previousStates["velocity"] = velocity
		else:
			sendingMsg.linear.x = previousStates["velocity"]
			
	#If within the range do not move velocity
	else:
		velocity = 0.0
		previousStates["velocity"] = 0.0
		sendingMsg.linear.x = velocity
		doneDistanceAlignment = True

	#Checking if the camera tiltLvl is above a threshold to ensure that we also go in height
	doneCameraAlignment = cameraAlignment()

	#We send move command, when done we send 0
	pubCmdVel.publish(sendingMsg)

	#Checking if all alignments have been done if yes switch to next stage
	if doneAngularAlignment and doneDistanceAlignment and doneCameraAlignment:
		approachState["current"] += 1

	



#Approach Target while adjusting camera and using depth
def moveToFinalPosition(pubCmdVel):
	global target, approachState

	sendingMsg = Twist()
	doneCameraAlignment = False
	doneDepthAlignment = False
	doneAngularAlignment = False

	#Calling align angular and assigning returned values
	returnOfFunctionAngular = angularAlignment(sendingMsg)
	sendingMsg = returnOfFunctionAngular[0]
	doneAngularAlignment = returnOfFunctionAngular[1]

	#Calling align camera
	doneCameraAlignment = cameraAlignment()

	#Checking if we did a torso alignment of more than 0.1, then We can move closer to the target
	if previousStates["torso"][0] >= 0.2:
		depthDistance = 0.5
	else:
		depthDistance = 0.6

	#Checking if the target is still further away than 60cm
	if abs(target["depth"]) >= depthDistance:

		#Calculate velocity with following formular: (distance/maxRange)*maxSpeed
		velocity = (target["depth"]/4.2)*0.8

		#Checking if difference between old and new velocity are bigger than certain threshhold
		if abs(abs(velocity) - previousStates["velocity"]) >= 0.03:
			sendingMsg.linear.x = velocity
			previousStates["velocity"] = velocity
		else:
			sendingMsg.linear.x = velocity
	
	#Otherwise within the depth do not move forward
	else:
		velocity = 0.0
		previousStates["velocity"] = velocity
		sendingMsg.linear.x = velocity
		doneDepthAlignment = True
	
	#Sending the command to move if needed otherwise 0 should be passed
	pubCmdVel.publish(sendingMsg)

	#Checking if all alignments have been done
	if doneCameraAlignment and doneDepthAlignment and doneAngularAlignment and approachState["current"] == 1:
		approachState["current"] = 2

	elif doneCameraAlignment and doneDepthAlignment and doneAngularAlignment and approachState["current"] == 3:
		approachState["current"] = 4

	# elif not doneDepthAlignment or not doneAngularAlignment:
	# 	pubCmdVel.publish(sendingMsg)





#Readjusting height incase camera wasnt tilted down
def readjustHeight():
	global target, previousStates, approachState

	doneTorsoAlignment = previousStates["doneTrsoAlignment"]
	doneCameraAlignment = False

	#Checking if the tiltLvl is over a certain threshold if yes then readjust trso otherwise check if camera is aligned
	if previousStates["tiltLvl"] <= 0.3 and not doneTorsoAlignment:
		doneTorsoAlignment = torsoAlignment(-220)
		previousStates["doneTrsoAlignment"] = doneTorsoAlignment
	else:
		doneTorsoAlignment = True
		previousStates["doneTrsoAlignment"] = doneTorsoAlignment

	if doneTorsoAlignment:
		doneCameraAlignment = cameraAlignment()

	if doneCameraAlignment: 
		approachState["current"] = 3





#Initialize all data
def initialize():
	global previousStates, target, sensorDirection_service, sensorData

	#Updating sensorThresholds if not updated retry after a second
	while not changeSensorValues(sensorDirection_service):
		time.sleep(1)

	#Resetting global variables
	sensorData = {
		"left": 25,
		"forward": 25,
		"right": 25
	}

	previousStates = {
		"velocity": 0.0,
		"angular": 0.0,
		"torso": [0.0],
		"tiltLvl": 0.0,
		"doneTrsoAlignment": False
	}

	target = {
		"xPos": -1.0,
		"yPos": -1.0,
		"area": -1.0,
		"length": -1.0,
		"distanceToCenterX": -1.0,
		"distanceToCenterY": -1.0,
		"distanceToBottomY": -1.0,
		"depth": -1.0
	}





#function to approach the target
def approachingTarget(pubCmdVel, pubApproachTarget):
	global approachState, showDebug

	if showDebug:
		rospy.loginfo("ApproachState: %s", approachState["current"])

	if approachState["current"] == -1:
		#call initialize
		initialize()
		approachState["current"] += 1

	#Checking if xPos, yPos and area have values otherwise don't publish
	if target["xPos"] != -1 and target["yPos"] != -1 and target["area"] != -1:

		#Checking if the approachState == 2, which means we are done approaching
		if approachState["current"] == 4:
			sendDone(pubApproachTarget, True)

		#Checking if the approachState == 3, which means height was adjusted thus we readjust final position
		elif approachState["current"] == 3:

			#Call move to final position
			moveToFinalPosition(pubCmdVel)

		#Checking if the approachState == 2, which means readjusting height
		elif approachState["current"] == 2:

			#Call move to final position
			readjustHeight()

		#Checking if the approachState == 1, which means moving to final position
		elif approachState["current"] == 1:

			#Call move to final position
			moveToFinalPosition(pubCmdVel)
		
		#Checking if the approachState = 0, which means move closer to target and adjust height
		if approachState["current"] == 0:

			#Call move closer to target
			moveCloserToTarget(pubCmdVel)





#Function used to setup all publisher subscribers and so on
def setup():
	global robotState, approachState, sensorDirection_service

	#Initiating node
	rospy.init_node('approachTarget', anonymous=True)

	#Subscribing to /stateMachine to get the current state of the Robot
	rospy.Subscriber('/stateMachine', StateMachine, callbackStateMachine)

	#Subscribing to /objectPosition to get the current position of the Object
	rospy.Subscriber('/objectPosition', ObjectPosition, callbackObjectPosition)

	#Subscribing to /base_scan to get the Laserscanner information
	rospy.Subscriber('/sensorData', SensorDirection, callbackSensorData)

	#Creating the publisher to publish to /cmd_vel
	pubCmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	#Creating the publisher to publish to /approachTargetStatus
	pubApproachTarget = rospy.Publisher("approachTargetStatus", DoneWithState, queue_size=10)

	#Adding the stateMachine_setState service, to change the state the robot is in
	rospy.wait_for_service("/sensorThreshold_setThreshold")
	sensorDirection_service = rospy.ServiceProxy('/sensorThreshold_setThreshold', SensorDirectionService)


	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		if robotState == 1:
			approachingTarget(pubCmdVel,pubApproachTarget)
		else:
			approachState["current"] = -1
			sendDone(pubApproachTarget, False)
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass