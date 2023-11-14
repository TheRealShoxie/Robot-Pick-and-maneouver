#!/usr/bin/env python

#This Python file is used to search for the next Target





#_____________________IMPORTS_____________________

#Importing needed libraries, msg and srv types
import rospy
from moveit_python import MoveGroupInterface
from ibo1_ros360.msg import SensorDirection, StateMachine, ObjectPosition, DoneWithState
from geometry_msgs.msg import Twist
from moveit_msgs.msg import MoveItErrorCodes
from ibo1_ros360.srv import ResetTargetService





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
sensorData = {
	"left": 0.0,
	"forward": 0.0,
	"right": 0.0
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
previousStates = {
	"angular": 0.0,
	"turningSide": "left"
}
joint_names = ["shoulder_pan_joint",
                "shoulder_lift_joint", "upperarm_roll_joint",
                "elbow_flex_joint", "forearm_roll_joint",
                "wrist_flex_joint", "wrist_roll_joint"]
originalPose = [1.3200007983036732, 1.3999813261835135, -0.19999156896589554, 1.7200002648456447,
				0, 1.65997797158587, 0]
resetTarget_service = None
nextTargetState = -1
move_group = None
robotState = -1
showDebug = False





#_____________________CALLBACK FUNCTIONS_____________________

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





#send request to StateMachineService to change robotState
def resetTarget():
	global resetTarget_service

	response = resetTarget_service(True)
	if response:
		#rospy.loginfo("Finished reset!")
		return
	if not response:
		rospy.logerr("ERR: resetTarget wasnt successfull")
		return





#Turn till Object found
def turnTillObjectFound(pubCmdVel):
	global target, previousStates

	sendingMsg = Twist()
	foundObject = False

	#Turn till target found
	if target["xPos"] == -1:
		if previousStates["turningSide"] == "left":
			sendingMsg.angular.z = 0.6
			previousStates["angular"] = 0.6
		else:
			sendingMsg.angular.z = -0.6
			previousStates["angular"] = -0.6

	#If target found keep turning till you are within 180px this is to ensure a more smoother transition
	#between the two speed values
	elif abs(target["distanceToCenterX"]) >= 200:
			sendingMsg.angular.z = previousStates["angular"]

	else:
		sendingMsg.angular.z = 0.0
		previousStates["angular"] = 0.0
		foundObject = True

	pubCmdVel.publish(sendingMsg)

	return foundObject





#Function to move the gripper to the target
def moveToTarget():
	global move_group, joint_names, originalPose

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





#Function to reset all needed values
def initialize():
	global target, move_group, previousStates

	previousStates["turningSide"] = "left"
	previousStates["angular"] = 0.0

	#Resetting Target due to earlier calls it wont have correct values
	target["xPos"] = -1
	target["yPos"] = -1
	target["area"] = -1
	target["length"] = -1
	target["distanceToCenterX"] = -1
	target["distanceToCenterY"] = -1
	target["distanceToBottomY"] = -1
	target["depth"] = -1

	#Reset Target
	resetTarget()

	#Setting move_group
	move_group = MoveGroupInterface("arm", "base_link")





#Function used for moving
def searchForNextTarget(pubCmdVel,pubNextTargetStatus):
	#Making global variables usable
	global nextTargetState, sensorData, previousStates, showDebug

	if showDebug:
		rospy.loginfo("NextTargetState: %s", nextTargetState)

	#done with dropping
	if nextTargetState == 2:
		sendDone(pubNextTargetStatus, True)

	#moveToDropTarget
	elif nextTargetState == 1:
		if moveToTarget():
			nextTargetState += 1

	#Turning till target is within certain amount of pixel
	elif nextTargetState == 0:
		if turnTillObjectFound(pubCmdVel):
			nextTargetState += 1
	
	#Settin the direction to turn to
	elif nextTargetState == -1:

		initialize()

		#Checking if sensorData has data
		if sensorData["left"] != 0 :

			#Finding out which side to turn to
			if sensorData["left"] > sensorData["right"]:
				previousStates["turningSide"] = "left"
			else:
				previousStates["turningSide"] = "right"

			nextTargetState += 1





#Function used to setup all publisher subscribers and so on
def setup():
	#Accessing global variables
	global robotState, resetTarget_service, nextTargetState

	#Initiating node
	rospy.init_node('searchingForNextTarget', anonymous=True)

	#Subscribing to /base_scan to get the Laserscanner information
	rospy.Subscriber('/sensorData', SensorDirection, callbackSensorData)
	
	#Subscribing to /stateMachine to get the current state of the Robot
	rospy.Subscriber('/stateMachine', StateMachine, callbackStateMachine)

	#Subscribing to /objectPosition to get the current position of the Object
	rospy.Subscriber('/objectPosition', ObjectPosition, callbackObjectPosition)


	#Creating the publisher to publish to /cmd_vel
	pubCmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	#Creating the publisher to publish to /searchingForNextTargetStatus
	pubNextTargetStatus = rospy.Publisher("searchingForNextTargetStatus", DoneWithState, queue_size=10)


	#Adding the targetAllocation_reset service, to reset Target
	rospy.wait_for_service("/targetAllocation_reset")
	resetTarget_service = rospy.ServiceProxy('/targetAllocation_reset', ResetTargetService)


	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		if robotState == 4:
			searchForNextTarget(pubCmdVel, pubNextTargetStatus)
		else:
			nextTargetState = -1
			sendDone(pubNextTargetStatus, False)
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass