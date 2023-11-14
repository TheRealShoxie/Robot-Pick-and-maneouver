#!/usr/bin/env python

#This Python file is used to go through the maze and and drive around





#_____________________IMPORTS_____________________
#Importing needed libraries, msg and srv types
import rospy
from ibo1_ros360.msg import SensorDirection, StateMachine
from geometry_msgs.msg import Twist
from ibo1_ros360.srv import SensorDirectionService





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
sensorData = {
	"left": 0.0,
	"forward": 0.0,
	"right": 0.0
}
sensorThreshold = {
	"left": 310,
	"forwardMin": 310,
	"forwardMax": 350,
	"right": 350,
}
previousStates = {
	"velocity": 0.0,
	"angular": 0.0,
	"inMaze": True,
	"inOpen": False
}
maxDistanceInMaze = 3.5
distanceToSwitchToOpen = 6
robotState = -1
sensorDirection_service = None





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





#_____________________METHODS_____________________

#send request to SensorDirectionService to change sensorThresholds
def changeSensorValues():
	global sensorDirection_service, sensorThreshold

	try: 
		response = sensorDirection_service(int(sensorThreshold["left"]), int(sensorThreshold["forwardMin"]), 
										   int(sensorThreshold["forwardMax"]), int(sensorThreshold["right"]))
		#rospy.loginfo(response)
	except rospy.ServiceException as e:
		print("Service call failed: %s", e)





#Function used for moving
def moving(pub):
	#Making global variables usable
	global sensorData, previousStates, maxDistanceInMaze, distanceToSwitchToOpen

	#Creating the sendingMsg with its corresponding type
	sendingMsg = Twist()

	#Calculate velocity and angular with following formular: (distance/maxRange)*maxSpeed
	if sensorData["forward"] > 4:
		distanceForward = 4
	else:
		distanceForward = sensorData["forward"]

	velocity = (distanceForward/4)*0.8

	#If we are in Maze
	if previousStates["inMaze"]:

		#Checking if left and right sensor are above certain threshold, if yes set that as max threshold
		if sensorData["left"] > maxDistanceInMaze:
			distanceLeft = maxDistanceInMaze
		else:
			distanceLeft = sensorData["left"]

		if sensorData["right"] > maxDistanceInMaze:
			distanceRight = maxDistanceInMaze
		else:
			distanceRight = sensorData["right"]

		#Calculate angular
		distanceLeftToRight = distanceLeft - distanceRight

		angular = ((distanceLeftToRight)/maxDistanceInMaze)*3

	#Otherwise if left or right sensor are above distanceToSwitch and we are still in Maze,
	#we set open true and inMaze false
	elif (sensorData["left"] > distanceToSwitchToOpen or sensorData["right"] > distanceToSwitchToOpen) and previousStates["inMaze"]:
		
		previousStates["inOpen"] = True
		previousStates["inMaze"] = False

	#Otherwise if we are already in open then calculate angular
	elif previousStates["inOpen"]:
		distanceLeftToRight = sensorData["left"] - sensorData["right"]

		angular = ((distanceLeftToRight)/5)*3

	#Checking if forward sensor is below certain Threshold
	if sensorData["forward"] <= 0.1:
		#Deciding to turn to the side which has more space
		if sensorData["left"] >= sensorData["right"]:
			angular = 0.3

		else:
			angular = -0.3


	#Checking if difference in velocity or angular to previous is greater a certain threshold
	if abs(velocity - previousStates["velocity"]) >= 0.05 or abs(angular - previousStates["angular"]) >= 0.2:

		#Assigning the to be send velocity and angular
		sendingMsg.linear.x = velocity
		sendingMsg.angular.z = angular
		previousStates["velocity"] = velocity
		previousStates["angular"] = angular

	#If difference within threshold then send previous values for velocity and angular
	else:
		sendingMsg.linear.x = previousStates["velocity"]
		sendingMsg.angular.z = previousStates["angular"]

	#rospy.loginfo("Sending velocity and angular: velocity=%s angular=%s", sendingMsg.linear.x, sendingMsg.angular.z)
	#Publishing msg to defined Publisher in setup
	pub.publish(sendingMsg)





#Function used to setup all publisher subscribers and so on
def setup():
	#Accessing global variables
	global sensorDirection_service, robotState

	#Initiating node
	rospy.init_node('drivingIntoOpen', anonymous=True)

	#Subscribing to /base_scan to get the Laserscanner information
	rospy.Subscriber('/sensorData', SensorDirection, callbackSensorData)
	
	#Subscribing to /stateMachine to get the current state of the Robot
	rospy.Subscriber('/stateMachine', StateMachine, callbackStateMachine)

	#Creating the publisher to publish to /cmd_vel
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	#Adding the stateMachine_setState service, to change the state the robot is in
	rospy.wait_for_service("/sensorThreshold_setThreshold")
	sensorDirection_service = rospy.ServiceProxy('/sensorThreshold_setThreshold', SensorDirectionService)

	#Updating sensorThresholds
	changeSensorValues()


	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		if robotState == 0:
			moving(pub)
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass
