#!/usr/bin/env python

#This Python file is used to drop a cube in the bin





#_____________________IMPORTS_____________________

#Importing needed libraries, msg and srv types
import rospy, actionlib, tf2_ros, tf_conversions, math
from moveit_python import MoveGroupInterface
from sensor_msgs.msg import JointState
from ibo1_ros360.msg import SensorDirection, StateMachine, ObjectPosition, DoneWithState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Twist
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal, 
							  FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes
from ibo1_ros360.srv import SensorDirectionService, ResetTargetService





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
sensorData = {
	"left": 0.0,
	"forward": 0.0,
	"right": 0.0
}
sensorThreshold = {
	"left": 260,
	"forwardMin": 260,
	"forwardMax": 400,
	"right": 400
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
imageDimensions = {
	"xMax": 640,
	"yMax": 480
}
previousStates = {
	"velocity": 0.0,
	"angular": 0.0,
	"tiltLvl": 0.0,
	"turningSide": "left",
	"amountOfTries": 0
}
pose_offsets = {
	"firstOffset": [0.1, 0, 0.15, -math.pi/2, 0, -math.pi/2]
	}
head_joint_names = ["head_pan_joint", "head_tilt_joint"]
gripper_pose_stamped = None
gripper_frame = 'gripper_link'
gripper = {
	"closed": 0.024,
	"open": 0.08
}
joint_names = ["shoulder_pan_joint",
                "shoulder_lift_joint", "upperarm_roll_joint",
                "elbow_flex_joint", "forearm_roll_joint",
                "wrist_flex_joint", "wrist_roll_joint"]
currentJointPos = [0, 0, 0, 0, 0, 0, 0]
robotState = -1
resetTarget_service = None
droppingState = -1
tfBuffer = None
listener = None
trans = None
move_group = None
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




#callback function for joint_states
def callbackJointStates(receivedMsg):
	global currentJointPos
	
	currentJointPos[0] = receivedMsg.position[6]
	currentJointPos[1] = receivedMsg.position[7]
	currentJointPos[2] = receivedMsg.position[8]
	currentJointPos[3] = receivedMsg.position[9]
	currentJointPos[4] = receivedMsg.position[10]
	currentJointPos[5] = receivedMsg.position[11]
	currentJointPos[6] = receivedMsg.position[12]





#_____________________METHODS_____________________

#send request to SensorDirectionService to change sensorThresholds
def changeSensorValues():
	global sensorDirection_service, sensorThreshold

	try: 
		response = sensorDirection_service(int(sensorThreshold["left"]), int(sensorThreshold["forwardMin"]), 
										   int(sensorThreshold["forwardMax"]), int(sensorThreshold["right"]))
		#rospy.logerr(response)
	except rospy.ServiceException as e:
		print("Service call failed: %s", e)	






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
		return
	if not response:
		#rospy.logerr("ERR: resetTarget wasnt successfull")
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
	elif (target["distanceToCenterX"]) >= 200:
			sendingMsg.angular.z = previousStates["angular"]

	else:
		sendingMsg.angular.z = 0.0
		previousStates["angular"] = 0.0
		foundObject = True

	pubCmdVel.publish(sendingMsg)

	return foundObject





#function used to update transform from base_link to target
def updateTransformToTarget():
	global trans, tfBuffer

	#Geting the transform to the object
	try:
		#target_object should be the frame name defined from your broadcast
		trans = tfBuffer.lookup_transform("base_link", "target_object", rospy.Time())
		return True
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		rospy.logerr(e)
		return False





#Function to move the gripper to the target
def moveToTarget(pose):
	global move_group, gripper_pose_stamped, gripper_frame, droppingState

	arrivedAtPosition = False
	
	# Move gripper frame to the pose specified
	if droppingState != 3:
		gripper_pose_stamped.header.stamp = rospy.Time.now()
		gripper_pose_stamped.pose = pose

		# Move gripper frame to the pose specified
		move_group.moveToPose(gripper_pose_stamped, gripper_frame)
	else:
		#rospy.loginfo("Current Joint Positions: %s", currentJointPos)
		currentJointPos[6] = currentJointPos[6] + 1.5
		move_group.moveToJointPosition(joint_names, currentJointPos, wait=True)
	
	result = move_group.get_move_action().get_result()

	if result:
        # Checking the MoveItErrorCode
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			arrivedAtPosition = True
			#rospy.loginfo("Hello there!")
		else:
			# If you get to this point please search for:
			# moveit_msgs/MoveItErrorCodes.msg
			rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
			arrivedAtPosition = False
	else:
		rospy.logerr("MoveIt! failure no result returned.")
	
	return arrivedAtPosition





#Function to close grippers
def openGrippers():

	gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
	gripper_client.wait_for_server()

	gripper_goal = GripperCommandGoal()
	gripper_goal.command.max_effort = 10.0
	gripper_goal.command.position = gripper["open"]

	gripper_client.send_goal(gripper_goal)
	gripper_client.wait_for_result(rospy.Duration(5.0))





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

	#head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
	#rospy.loginfo("...done")





#Function fo Camera Alignment
def cameraAlignment():
	global target, previousStates, imageDimensions

	doneCameraAlignment = False

	boundry = (target["length"]/4)+40

	if boundry >= 220:
		boundry = 220

	#If we are below the line then tilt up
	if target["distanceToBottomY"] <= boundry:
		if previousStates["tiltLvl"] <= 1.43:
			previousStates["tiltLvl"] += 0.02

		tiltHead(previousStates["tiltLvl"])

	# Else if we are above the center - boundry then tilt down
	elif target["distanceToBottomY"] >= (imageDimensions["yMax"] -boundry):
		if previousStates["tiltLvl"] >= -0.74:
			previousStates["tiltLvl"] -= 0.02

		tiltHead(previousStates["tiltLvl"])


	#Otherwise set CameraAlignment to done = True
	else:
		doneCameraAlignment = True

	return doneCameraAlignment





#Align Angular
def angularAlignment(sendingMsg):
	global target

	doneAngularAlignment = False

	if abs(target["distanceToCenterX"]) >= 60:

		angular = (target["distanceToCenterX"] / 200) * 0.6

		if abs(abs(angular) - previousStates["angular"]) >= 0.05:
			sendingMsg.angular.z = angular
			previousStates["angular"] = angular
		else:
			sendingMsg.angular.z = previousStates["angular"]
	else:
		angular = 0.0
		previousStates["angular"] = 0.0
		sendingMsg.angular.z = angular
		doneAngularAlignment = True
	
	return [sendingMsg, doneAngularAlignment]





#Position to target
def positionToTarget(pubCmdVel):
	global target, previousStates, droppingState

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
	if target["depth"] == -1:
		velocity = 0.2
		previousStates["velocity"] = velocity
		sendingMsg.linear.x = velocity
		
	elif target["depth"] >= 0.68:

		#Calculate velocity with following formular: (distance/maxRange)*maxSpeed
		velocity = (target["depth"]/2.8)*0.5

		#Checking if difference between old and new velocity are bigger than certain threshhold
		if abs(abs(velocity) - previousStates["velocity"]) >= 0.03:
			sendingMsg.linear.x = velocity
			previousStates["velocity"] = velocity
		else:
			sendingMsg.linear.x = previousStates["velocity"]
			
	#Else within the range do not move velocity
	else:
		velocity = 0.0
		previousStates["velocity"] = 0.0
		sendingMsg.linear.x = velocity
		doneDistanceAlignment = True

	#Checking if the camera tiltLvl is above a threshold to ensure that we also go in height
	doneCameraAlignment = cameraAlignment()

	#Checking if all alignments have been done if yes switch to next stage
	if doneAngularAlignment and doneDistanceAlignment and doneCameraAlignment:
		droppingState = 2
	#Otherwise if angular or distance alignment not done send velocity

	elif not doneAngularAlignment or not doneDistanceAlignment:
		#rospy.loginfo("Sending velocity and angular: velocity=%s angular=%s", sendingMsg.linear.x, sendingMsg.angular.z)
		pubCmdVel.publish(sendingMsg)





#Go drop the cube
def moveToDropTarget():
	global trans, pose_offsets, previousStates

	
	t = TransformStamped()
	br = tf2_ros.TransformBroadcaster()

	arrivedAtPosition = False

	#Building the transform
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "base_link"
	t.child_frame_id = "updated_Target"

	
	t.transform.translation.x = trans.transform.translation.x + pose_offsets["firstOffset"][0]
	t.transform.translation.y = trans.transform.translation.y + pose_offsets["firstOffset"][1]
	t.transform.translation.z = trans.transform.translation.z + pose_offsets["firstOffset"][2]


	q = tf_conversions.transformations.quaternion_from_euler(pose_offsets["firstOffset"][3],
															 pose_offsets["firstOffset"][4],
															 pose_offsets["firstOffset"][5])

	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	#Sending transform and saving previous
	br.sendTransform(t)

	pose = Pose(Point(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z), 
		        Quaternion(t.transform.rotation.x, t.transform.rotation.y,
				t.transform.rotation.z, t.transform.rotation.w))

	arrivedAtPosition = moveToTarget(pose)

	#Check if we didnt reach 
	if not arrivedAtPosition:
		previousStates["amountOfTries"] += 1

		#Checking if we tried to get to position more than 3times then decrease distance to position
		if previousStates["amountOfTries"] > 3:
			pose_offsets["firstOffset"][0] = pose_offsets["firstOffset"][0] - 0.002
			previousStates["amountOfTries"] = 0
			arrivedAtPosition = False
	else:
		previousStates["amountOfTries"] = 0

	return arrivedAtPosition






#Function to reset all needed values
def initialize():
	global target, move_group, previousStates, pose_offsets

	#Resetting Target due to earlier calls it wont have correct values
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
		"velocity": 0.0,
		"angular": 0.0,
		"tiltLvl": 0.0,
		"turningSide": "left"
	}

	pose_offsets = {
	"firstOffset": [0.1, 0, 0.15, -math.pi/2, 0, -math.pi/2]
	}

	previousStates["amountOfTries"] = 0

	#Reset Target
	resetTarget()

	#Updating sensorThresholds
	changeSensorValues()

	move_group = MoveGroupInterface("arm", "base_link")





#Function used for moving
def droppingCube(pubCmdVel,pubDropIntoBinStatus):
	#Making global variables usable
	global droppingState, sensorData, previousStates, showDebug

	if showDebug:
		rospy.loginfo("DroppingIntoBinState: %s", droppingState)

	#done with dropping, so we reset camera
	if droppingState == 5:
		tiltHead(0)
		sendDone(pubDropIntoBinStatus, True)

	#drop the target
	elif droppingState == 4:
		openGrippers()
		droppingState += 1

	#Turn the wrist
	elif droppingState == 3:
		moveToTarget(0.0)
		droppingState += 1

	#moveToDropTarget
	elif droppingState == 2:
		if updateTransformToTarget():
			if moveToDropTarget():
				droppingState += 1

	#Approaching target
	elif droppingState == 1:
		positionToTarget(pubCmdVel)

	#Turning till target is within certain amount of pixel
	elif droppingState == 0:
		if turnTillObjectFound(pubCmdVel):
			droppingState = 1

	#Settin the direction to turn to
	elif droppingState == -1:

		initialize()
		
		#Checking if sensorData has data
		if sensorData["left"] != 0 :

			#Finding out which side to turn to
			if sensorData["left"] > sensorData["right"]:
				previousStates["turningSide"] = "left"
			else:
				previousStates["turningSide"] = "right"

			droppingState = 0





#Function used to setup all publisher subscribers and so on
def setup():
	#Accessing global variables
	global sensorDirection_service, robotState, resetTarget_service
	global tfBuffer, move_group, gripper_pose_stamped, listener, droppingState

	#Initiating node
	rospy.init_node('droppingIntoBin', anonymous=True)

	#Subscribing to /base_scan to get the Laserscanner information
	rospy.Subscriber('/sensorData', SensorDirection, callbackSensorData)
	
	#Subscribing to /stateMachine to get the current state of the Robot
	rospy.Subscriber('/stateMachine', StateMachine, callbackStateMachine)

	#Subscribing to /objectPosition to get the current position of the Object
	rospy.Subscriber('/objectPosition', ObjectPosition, callbackObjectPosition)

	#Subscribing to the joint states to safe initial joint states
	rospy.Subscriber('/joint_states', JointState, callbackJointStates)

	#Creating the publisher to publish to /cmd_vel
	pubCmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	#Creating the publisher to publish to /pickUpObjectStatus
	pubDropIntoBinStatus = rospy.Publisher("droppingIntoBinStatus", DoneWithState, queue_size=10)

	#Adding the sensorThreshold_setThreshold service, to change the state of the sensor thresholds
	rospy.wait_for_service("/sensorThreshold_setThreshold")
	sensorDirection_service = rospy.ServiceProxy('/sensorThreshold_setThreshold', SensorDirectionService)

	#Adding the targetAllocation_reset service, to reset Target
	rospy.wait_for_service("/targetAllocation_reset")
	resetTarget_service = rospy.ServiceProxy('/targetAllocation_reset', ResetTargetService)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	# Construct a "pose_stamped" message as required by moveToPose
	gripper_pose_stamped = PoseStamped()
	gripper_pose_stamped.header.frame_id = 'base_link'

	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		if robotState == 3:
			droppingCube(pubCmdVel, pubDropIntoBinStatus)
		else:
			droppingState = -1
			sendDone(pubDropIntoBinStatus, False)
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass