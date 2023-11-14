#!/usr/bin/env python

#This Python file is used to pickup and object





#_____________________IMPORTS_____________________

#Importing needed libraries and msg types
import rospy, math, tf2_ros, tf_conversions, actionlib, time
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from ibo1_ros360.msg import StateMachine, DoneWithState
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Twist
from moveit_msgs.msg import MoveItErrorCodes
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal, 
							  FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
sensorData = {
	"valuesOfLasers" : [None]*662,
	"numberOfLaserUsed": 0,
	"smallestDistanceToObject": 25,
	"sensorMiddle": 0
}
robotState = 0
pickUpState = -1
previousVelocity = 0
previousStates = {
	"velocity": 0,
	"moveToTry": 0
}
tfBuffer = None
listener = None
trans = None
move_group = None
gripper_pose_stamped = None
gripper_frame = 'gripper_link'
gripper = {
	"closed": 0,
	"open": 0.7
}
cylinderForPlanningScene = {
	"height": 0,
	"radius": 0,
	"xCenter": 0,
	"yCenter": 0,
	"zCenter": 0
}
pose_offsets = {
	"firstOffset": [[0, 0.4, 0.1, 0, 0, 0], ["base_link", "target_object"]],
	"secondOffset": [[0, 0.25, 0.08, 0, (math.pi/8), -(math.pi/2)], ["base_link", "target_object"]],
	"thirdOffset": [[0, 0.12, 0.04, 0, (math.pi/8), -(math.pi/2)], ["base_link", "target_object"]],
	"fourthOffset": [[0.02, -0.01, -0.01, 0, (math.pi/12), -(math.pi/2)], ["base_link", "target_object"]],
	"fithOffset": [[0.02, -0.035, -0.02, 0, (math.pi/16), -(math.pi/2)], ["base_link", "target_object"]],
	"endGoal": [[0.10, -0.015, -0.3, 0, 0, -(math.pi/2)], ["base_link", "head_camera_depth_frame"]],
}
joint_names = ["shoulder_pan_joint",
                "shoulder_lift_joint", "upperarm_roll_joint",
                "elbow_flex_joint", "forearm_roll_joint",
                "wrist_flex_joint", "wrist_roll_joint"]
currentJointPos = [0, 0, 0, 0, 0, 0, 0]
gripperJoints = [0,0]
head_joint_names = ["head_pan_joint", "head_tilt_joint"]
torso_joint_names = ["torso_lift_joint"]
planning_scene = None
showDebug = False





#_____________________CALLBACK FUNCTIONS_____________________

def callbackLaserScan(receivedMsg):
	#Making global variables usable
	global sensorData
	sensorData["smallestDistanceToObject"] = 25

	#Defining variables for how many lasers have been used and did not return inf
	numberOfLaserUsed = 0

	a#Sving ranges in a temporary variable to lessen rewriting of receivedMsg
	ranges = receivedMsg.ranges
	#rospy.loginfo(len(ranges))
	#Cycling through all elements of the array
	for index in range(len(ranges)):

		#Checking if current range is finite
		if math.isfinite(ranges[index]):
			#Checking if the range is within 2times the range of front forward facing sensor
			if ranges[index] <= (ranges[331]*2):

				#Saving the smallest distance of the object
				if sensorData["smallestDistanceToObject"] > ranges[index]:
					sensorData["smallestDistanceToObject"] = ranges[index]

				sensorData["valuesOfLasers"][numberOfLaserUsed] = ranges[index]
				numberOfLaserUsed += 1
	
	sensorData["numberOfLaserUsed"] = numberOfLaserUsed
	sensorData["sensorMiddle"] = ranges[331]





#callback function for StateMachine
def callbackStateMachine(receivedMsg):
	global robotState
	
	robotState = receivedMsg.currentState
	




#callback function for joint_states
def callbackJointStates(receivedMsg):
	global currentJointPos, gripperJoints
	
	currentJointPos[0] = receivedMsg.position[6]
	currentJointPos[1] = receivedMsg.position[7]
	currentJointPos[2] = receivedMsg.position[8]
	currentJointPos[3] = receivedMsg.position[9]
	currentJointPos[4] = receivedMsg.position[10]
	currentJointPos[5] = receivedMsg.position[11]
	currentJointPos[6] = receivedMsg.position[12]
	gripperJoints[0] = receivedMsg.position[13]
	gripperJoints[1] =  receivedMsg.position[14]





#_____________________METHODS_____________________

#Send done msg
def sendDone(publishDone, isDone):
	#Create ApproachTarget msg and send that we are done
	sendingMsg = DoneWithState()
	sendingMsg.doneWithState = isDone
	publishDone.publish(sendingMsg)





#function used to update transform from base_link to target
def updateTransformToTarget():
	global trans, tfBuffer, pickUpState, pose_offsets

	transformLinks = ["", ""]

	#Checking if pickupState is 11/12 or 13 then we use different links to lookuü the transform
	if pickUpState == 11 or pickUpState == 12 or pickUpState == 13:
		transformLinks[0] = pose_offsets["endGoal"][1][0]
		transformLinks[1] = pose_offsets["endGoal"][1][1]
	else:
		transformLinks[0] = pose_offsets["firstOffset"][1][0]
		transformLinks[1] = pose_offsets["firstOffset"][1][1]

	#Geting the transform to the object
	try:
		#target_object should be the frame name defined from your broadcast
		trans = tfBuffer.lookup_transform(transformLinks[0], transformLinks[1], rospy.Time())
		return True
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		rospy.logerr(e)
		return False





#Add Cylinder
def addCylinder():
	global planning_scene, cylinderForPlanningScene

	planning_scene.removeCollisionObject("cylinder_infront")

	planning_scene.addCylinder("cylinder_infront", cylinderForPlanningScene["height"], cylinderForPlanningScene["radius"],
							   cylinderForPlanningScene["xCenter"], cylinderForPlanningScene["yCenter"],
							   cylinderForPlanningScene["zCenter"])





#function to setup planner
def setupCollisionObjects():
	global pickUpState, sensorData, trans, planning_scene, cylinderForPlanningScene

	# Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
	planning_scene.removeCollisionObject("my_front_ground")
	planning_scene.removeCollisionObject("my_back_ground")
	planning_scene.removeCollisionObject("my_right_ground")
	planning_scene.removeCollisionObject("my_left_ground")
	planning_scene.removeCollisionObject("cylinder_infront")

	planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
	planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
	planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
	planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

	#Calculating the size for the cubes
	leftMostLaserDistance = sensorData["valuesOfLasers"][0]
	rightMostLaserDistance = sensorData["valuesOfLasers"][sensorData["numberOfLaserUsed"]-1]
	distanceMostLeftLaserRoot = pow(leftMostLaserDistance, 2)
	distanceMostRightLaserRoot = pow(rightMostLaserDistance,2)

	#220° are represented in 662lasers thus: radius = (usedLaser * maxRadius)/maxLasers
	radiusOfLaser = (sensorData["numberOfLaserUsed"] * 220)/662
	partOfFormula = 2*leftMostLaserDistance*rightMostLaserDistance*math.cos(math.radians(radiusOfLaser))

	#Calculating size for cylinder and add a buffer of 12cm
	cylinderRadrius = ((math.sqrt(distanceMostLeftLaserRoot+distanceMostRightLaserRoot-partOfFormula))+0.12)/2

	#Calculating distance to cylinder Center. Shift by 0.235 which is distance between laser_link and base_link plus a buffer of 1cm
	cylinderXCenter = sensorData["smallestDistanceToObject"]+(cylinderRadrius)+0.235+0.01

	#Calculating cylinder Y Center
	lasersToMiddle = sensorData["valuesOfLasers"].index(sensorData["sensorMiddle"])
	shiftedRadius = ((lasersToMiddle+1) * 220)/662
	cylinderYCenter = ((((radiusOfLaser/2)-shiftedRadius)*2*cylinderRadrius)/radiusOfLaser)*-1

	#Calculating height of cylinder Center
	#Formular for height is explained in documentation used cube size cm, as I got more consisten values
	cylinderHeight = trans.transform.translation.z
	cylinderZCenter = cylinderHeight/2

	cylinderForPlanningScene["height"]= cylinderHeight
	cylinderForPlanningScene["radius"] = cylinderRadrius
	cylinderForPlanningScene["xCenter"] = cylinderXCenter
	cylinderForPlanningScene["yCenter"] = cylinderYCenter
	cylinderForPlanningScene["zCenter"] = cylinderZCenter

	addCylinder()

	pickUpState = 1





#Function to update moveGroup
def updateMoveGroup(type, link):
	global move_group

	move_group = MoveGroupInterface(type, link)





#Function to move the gripper to the target
def moveToTarget(pose):
	global move_group, gripper_pose_stamped, gripper_frame, joint_names,currentJointPos, pickUpState, showDebug

	arrivedAtPosition = False

	# Move gripper frame to the pose specified
	if pickUpState != 13:
		gripper_pose_stamped.header.stamp = rospy.Time.now()
		gripper_pose_stamped.pose = pose

		# Move gripper frame to the pose specified
		move_group.moveToPose(gripper_pose_stamped, gripper_frame)
	else:
		#rospy.loginfo("Current Joint Positions: %s", currentJointPos)
		currentJointPos[6] = currentJointPos[6] - 1.5
		move_group.moveToJointPosition(joint_names, currentJointPos, wait=True)

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





#Function to control the gripper
def gripperControl(toClose):
	global gripper, gripperJoints, showDebug

	gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
	gripper_client.wait_for_server()

	gripper_goal = GripperCommandGoal()
	gripper_goal.command.max_effort = 15.0
	
	
	#Checking if we should open or close
	if toClose:
		goalPosition = gripper["closed"]
	else:
		goalPosition = gripper["open"]

	gripper_goal.command.position = goalPosition
	gripper_client.send_goal(gripper_goal)
	gripper_client.wait_for_result(rospy.Duration(6.0))

	if showDebug:
		rospy.loginfo("Grippers: %s", gripperJoints)

	#Calculating avarage gripper distance
	avarageGripperDistance = (gripperJoints[0] + gripperJoints[1])/2


	#If gripper isnt closed more than gripper closed then open again and regrip
	if avarageGripperDistance >= 0.0255:
		return False
	else:
		return True





#Tilt the camera to 0 position
def resetCamera():
	global head_joint_names

	head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	head_client.wait_for_server()

	trajectory = JointTrajectory()
	trajectory.joint_names = head_joint_names
	trajectory.points.append(JointTrajectoryPoint())

	pose = [0, 0]
	trajectory.points[0].positions = pose
	trajectory.points[0].velocities = [0.0] * len(pose)
	trajectory.points[0].accelerations = [0.0] * len(pose)
	trajectory.points[0].time_from_start = rospy.Duration(5.0)

	head_goal = FollowJointTrajectoryGoal()
	head_goal.trajectory = trajectory
	head_goal.goal_time_tolerance = rospy.Duration(0.0)

	head_client.send_goal(head_goal)





#function to move only the torso
def resetTorso(goalPose):
	global torso_joint_names

	torso_client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	torso_client.wait_for_server()

	#Setting up the initialization for moving
	pose = goalPose
	trajectory = JointTrajectory()
	trajectory.joint_names = torso_joint_names
	trajectory.points.append(JointTrajectoryPoint())
	trajectory.points[0].positions = pose
	trajectory.points[0].velocities = [0.06]
	trajectory.points[0].accelerations = [0.01]
	trajectory.points[0].time_from_start = rospy.Duration(5.0)

	#Creating the goal and its initialization
	torso_goal = FollowJointTrajectoryGoal()
	torso_goal.trajectory = trajectory
	torso_goal.goal_time_tolerance = rospy.Duration(0.0)

	#Moving to the end_point
	torso_client.send_goal(torso_goal) #optioanly add callbacks, note fixed order: goalDone_cb, active_cb, feedback_cb)





#Drive backwards
def driveBackwards(pubCmdVel):
	global sensorData, previousStates, pickUpState

	sendingMsg = Twist()

	if sensorData["smallestDistanceToObject"] <= 0.6:
		#Calculate velocity with following formular: (distance/maxRange)*maxSpeed
		velocity = -(sensorData["smallestDistanceToObject"]/1)*0.8

		#Checking if difference between old and new velocity are bigger than certain threshhold
		if abs(abs(velocity) - previousStates["velocity"]) >= 0.03:
			sendingMsg.linear.x = velocity
			previousStates["velocity"] = velocity
		else:
			sendingMsg.linear.x = previousStates["velocity"]
			
	#If within the range do not move velocity
	else:
		pickUpState += 1
		sendingMsg.linear.x = 0.0
		previousStates["velocity"] = 0.0

	pubCmdVel.publish(sendingMsg)
	




#Move the gripper to the cube
def moveToCube(offset):
	global trans, pickUpState, previousStates, cylinderForPlanningScene

	
	t = TransformStamped()
	br = tf2_ros.TransformBroadcaster()

	arrivedAtPosition = False

	#Building the transform
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = offset[1][0]
	t.child_frame_id = "updated_Target"

	
	t.transform.translation.x = trans.transform.translation.x + offset[0][0]
	t.transform.translation.y = trans.transform.translation.y + offset[0][1]
	t.transform.translation.z = trans.transform.translation.z + offset[0][2]


	q = tf_conversions.transformations.quaternion_from_euler(offset[0][3], offset[0][4], offset[0][5])

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

	#Checking if we failed to get to position, if yes increase moveToTry
	if not arrivedAtPosition:
		previousStates["moveToTry"] += 1

		#Checking if we tried to get to position more than 5times then skip position
		if previousStates["moveToTry"] > 3:
			arrivedAtPosition = True
	
	#For final pose(end goal)
	if arrivedAtPosition and pickUpState == 11:
		previousStates["moveToTry"] = 0
		pickUpState +=1
	
	#For third poses
	if arrivedAtPosition and pickUpState == 9:
		previousStates["moveToTry"] = 0
		pickUpState +=1
	
	#For fourth poses
	if arrivedAtPosition and pickUpState == 8:
		previousStates["moveToTry"] = 0
		pickUpState +=1

	#For fith poses
	if arrivedAtPosition and pickUpState == 7:
		previousStates["moveToTry"] = 0
		pickUpState +=1

	#For fith poses
	if arrivedAtPosition and pickUpState == 5:
		previousStates["moveToTry"] = 0
		pickUpState +=1

	#For fourth poses
	if arrivedAtPosition and pickUpState == 4:
		previousStates["moveToTry"] = 0
		pickUpState +=1

	#For third poses
	if arrivedAtPosition and pickUpState == 3:
		#Remove Cube Height from the cylinder
		cylinderForPlanningScene["height"] = cylinderForPlanningScene["height"] - 0.05
		cylinderForPlanningScene["zCenter"] = cylinderForPlanningScene["height"]/2
		addCylinder()
		previousStates["moveToTry"] = 0
		pickUpState +=1

	#For second poses
	if arrivedAtPosition and pickUpState == 2:
		previousStates["moveToTry"] = 0
		pickUpState +=1

	#For first poses
	if arrivedAtPosition and pickUpState == 1:
		previousStates["moveToTry"] = 0
		pickUpState +=1





#Initialize all data
def initialize():
	global trans, currentJointPos

	trans = None
	currentJointPos[0] = 0.0
	currentJointPos[1] = 0.0
	currentJointPos[2] = 0.0
	currentJointPos[3] = 0.0
	currentJointPos[4] = 0.0
	currentJointPos[5] = 0.0
	currentJointPos[6] = 0.0





#Function used for moving
def pickUp(pubCmdVel, pubObjectStatus):
	#Making global variables usable
	global pickUpState, trans, pose_offsets, planning_scene, showDebug

	if showDebug:
		rospy.loginfo("PickUpState: %s", pickUpState)


	#Send out the done with pickingUpObject
	if pickUpState == 14:
		sendDone(pubObjectStatus, True)

	#Turn the wrist
	elif pickUpState == 13:
		moveToTarget([0.0])
		pickUpState += 1
	
	#Lower the torso
	elif pickUpState == 12:
		resetTorso([0.0])
		pickUpState += 1

	#Reset camera, change updateMoveGroup and update the transform tree
	elif pickUpState == 11:
		successfulUpdate = updateTransformToTarget()
		if successfulUpdate:
			resetCamera()
			updateMoveGroup("arm", "base_link")
			moveToCube(pose_offsets["endGoal"])

	#Move away from the object
	elif pickUpState == 10:
		driveBackwards(pubCmdVel)

	elif pickUpState == 9:
		moveToCube(pose_offsets["thirdOffset"])
		planning_scene.removeCollisionObject("cylinder_infront")

	#Now move backwards through the poses
	elif pickUpState == 8:
		moveToCube(pose_offsets["fourthOffset"])

	#Now move backwards through the poses
	elif pickUpState == 7:
		moveToCube(pose_offsets["fithOffset"])

	#close grippers
	elif pickUpState == 6:
		#Checking if we closed grippers then move to next stage
		if gripperControl(True):
			pickUpState += 1

		#Otherwise open grippers again and retry
		else:
			gripperControl(False)

	#move the to pose 5
	elif pickUpState == 5:
		if updateTransformToTarget():
			moveToCube(pose_offsets["fithOffset"])

	#move the to pose 4
	elif pickUpState == 4:
		if updateTransformToTarget():
			moveToCube(pose_offsets["fourthOffset"])

	#If pickupState == 3 move the to pose 3
	elif pickUpState == 3:
		if updateTransformToTarget():
			moveToCube(pose_offsets["thirdOffset"])

	#If pickupState == 2 move the to pose 2
	elif pickUpState == 2:
		if updateTransformToTarget():
			moveToCube(pose_offsets["secondOffset"])

	#If pickupState == 1 move the to pose 1
	elif pickUpState == 1:
		if updateTransformToTarget():
			moveToCube(pose_offsets["firstOffset"])

	#If pickupState == 0 create the collision box
	elif pickUpState == 0:
		successfulUpdate = updateTransformToTarget()
		if successfulUpdate:
			setupCollisionObjects()
			# Create move group interface for a fetch robot
			updateMoveGroup("arm", "base_link")
	
	elif pickUpState == -1:
		initialize()
		pickUpState += 1
	




#Function used to setup all publisher subscribers and so on
def setup():
	#Accessing global variables
	global robotState, move_group, tfBuffer, listener, gripper_pose_stamped, planning_scene, pickUpState

	#Initiating node
	rospy.init_node('pickUpObject', anonymous=True)

	#Subscribing to /stateMachine to get the current state of the Robot
	rospy.Subscriber('/stateMachine', StateMachine, callbackStateMachine)

	#Subscribing to /base_scan to get the Laserscanner information
	rospy.Subscriber('/base_scan', LaserScan, callbackLaserScan)

	#Subscribing to the joint states to safe initial joint states
	rospy.Subscriber('/joint_states', JointState, callbackJointStates)

	#Creating the publisher to publish to /cmd_vel
	pubCmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	#Creating the publisher to publish to /pickUpObjectStatus
	pubObjectStatus = rospy.Publisher("pickUpObjectStatus", DoneWithState, queue_size=10)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	planning_scene = PlanningSceneInterface("base_link")


	# Construct a "pose_stamped" message as required by moveToPose
	gripper_pose_stamped = PoseStamped()
	gripper_pose_stamped.header.frame_id = 'base_link'


	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		if robotState == 2:
			pickUp(pubCmdVel, pubObjectStatus)
		else:
			pickUpState = -1
			sendDone(pubObjectStatus, False)
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass