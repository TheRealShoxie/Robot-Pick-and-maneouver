#!/usr/bin/env python

#This Python file is used to to print the sensor values for forward,
#left and right. Also able to define which sensor is used for what





#_____________________IMPORTS_____________________
#Importing needed libraries, msg and srv types
import rospy, math
from sensor_msgs.msg import LaserScan
from ibo1_ros360.msg import SensorDirection
from ibo1_ros360.srv import SensorDirectionService, SensorDirectionServiceResponse





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
sensorThreshold = {
	"left": 250,
	"forwardMin": 250,
	"forwardMax": 410,
	"right": 410,
}
sensorData = {
	"left": 0.0,
	"forward": 0.0,
	"right": 0.0
}





#_____________________CALLBACK FUNCTIONS_____________________

#Callback used to deal with Requests
def callback(request):
	global sensorThreshold

	responseStr = ""
	responseBool = True

	#_______________________Sensor Thresholds_______________________
	#Checking if thresholdLeft greater 0 and smaller or same thresholdforwardMin
	if request.thresholdLeft > 0 and request.thresholdLeft <= request.thresholdForwardMin:
		sensorThreshold["left"] = request.thresholdLeft
		responseStr += " - Updated thresholdLeft"
	else:
		responseStr += " - Couldnt update thresholdLeft"
		responseBool = False
	
	#Checking if thresholdForwardMin greater 0, greater or same left and smaller thresholdforwardMax
	if (request.thresholdForwardMin > 0 and request.thresholdForwardMin >= request.thresholdLeft
	    and request.thresholdForwardMin < request.thresholdForwardMax):

		sensorThreshold["forwardMin"] = request.thresholdForwardMin
		responseStr += " - Updated thresholdForwardMin"
	else:
		responseStr += " - Couldnt update thresholdForwardMin"
		responseBool = False

	#Checking if thresholdForwardMax greater 0, smaller or same right and greater thresholdforwardMin
	if (request.thresholdForwardMax > 0 and request.thresholdForwardMax <= request.thresholdRight
	    and request.thresholdForwardMax > request.thresholdForwardMin):
		sensorThreshold["forwardMax"] = request.thresholdForwardMin
		responseStr += " - Updated thresholdForwardMax"
	else:
		responseStr += " - Couldnt update thresholdForwardMax"
		responseBool = False

	#Checking if thresholdLeft greater 0 and greater or same thresholdforwardMax
	if request.thresholdRight > 0 and request.thresholdRight >= request.thresholdForwardMax:
		sensorThreshold["right"] = request.thresholdRight
		responseStr += " - Updated thresholdRight"
	else:
		responseStr += " - Couldnt update thresholdRight"
		responseBool = False
	response = [responseStr, str(responseBool)]
	
	return SensorDirectionServiceResponse(response)





#callback function for LaserScan
def callbackLaserScan(receivedMsg):
	#Making global variables usable
	global sensorData, previousValues

	#Defining variables for how many lasers have been used and did not return inf
	amountOfLaserLeft = 0
	amountOfLaserForward = 0
	amountOfLaserRight = 0

	#Defining variables to save the accumulated distances for left, forward and right
	totalDistanceLeft = 0
	totalDistanceForward = 0
	totalDistanceRight = 0

	#Saving ranges in a temporary variable to lessen rewriting of receivedMsg
	ranges = receivedMsg.ranges

	#Cycling through all elements of the array
	for index in range(len(ranges)):

		#Checking if current range is finite
		if math.isfinite(ranges[index]):

			#Assigning the range to its corresponding direction and checking if within distanceThreshold
			if index < sensorThreshold["left"]:
				totalDistanceLeft += ranges[index]
				amountOfLaserLeft += 1

			elif index > sensorThreshold["right"]:
				totalDistanceRight += ranges[index]
				amountOfLaserRight += 1
			
			elif index >= sensorThreshold["forwardMin"] and index <= sensorThreshold["forwardMax"]:
				totalDistanceForward += ranges[index]
				amountOfLaserForward += 1

	#Calculating avarage distances for corresponding direction
	sensorData["left"] = totalDistanceLeft/amountOfLaserLeft
	sensorData["forward"] = totalDistanceForward/amountOfLaserForward
	sensorData["right"] = totalDistanceRight/amountOfLaserRight





#_____________________METHODS_____________________

#Function stateMachine which publishes the stateMachine
def sendingLaserScanData(pub):
	#Making global variables usable
	global sensorData

	#Creating the sendingMsg with its corresponding type
	sendingMsg = SensorDirection()

	#Assigning the robotState to the msg
	sendingMsg.thresholdLeft = sensorData["left"]
	sendingMsg.thresholdForward = sensorData["forward"]
	sendingMsg.thresholdRight = sensorData["right"]

	#publishing the state of the robot
	pub.publish(sendingMsg)





#Function used to setup all publisher subscribers and so on
def setup():
	rospy.init_node('sensorThreshold_setThreshold', anonymous=True)

	#Creating a new publisher named stateMachine
	pub = rospy.Publisher('sensorData', SensorDirection, queue_size=10)

	#Subscribing to /base_scan to get the Laserscanner information
	rospy.Subscriber('/base_scan', LaserScan, callbackLaserScan)

	#Subscribing to /stateMachine to get the stateMachine msgs
	service = rospy.Service("sensorThreshold_setThreshold", SensorDirectionService, callback)

	
	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		sendingLaserScanData(pub)
		rate.sleep()
	rospy.spin()
   
if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass