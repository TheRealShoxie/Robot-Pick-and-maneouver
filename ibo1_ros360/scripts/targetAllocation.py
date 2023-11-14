#!/usr/bin/env python

#This Python file is used to look for a target send it to transform tree and
#and publish certain values





#_____________________IMPORTS_____________________

#Importing needed libraries and msg types
import rospy, cv2, math, tf2_ros, tf_conversions
from opencv_apps.msg import MomentArrayStamped
from ibo1_ros360.msg import ObjectPosition
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from ibo1_ros360.srv import ResetTargetService, ResetTargetServiceResponse





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
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
br = tf2_ros.TransformBroadcaster()
t = TransformStamped()
previousStates = {
	"transform": None,
	"xPos": -1,
	"yPos": -1,
	"area": -1,
	"depth": -1
}
bridge = CvBridge()
showDebug = False





#_____________________CALLBACK FUNCTIONS_____________________

#Callback used to deal with Requests
def callbackRequest(request):
	global target

	#Checking what value we receiver
	resetTargetRequest = request.resetTarget
	returnValue = False
	
	#If request is to reset target reset all values
	if resetTargetRequest:
		target["xPos"] = -1
		target["yPos"] = -1
		target["area"] = -1
		target["length"] = -1
		target["distanceToCenterX"] = -1
		target["distanceToCenterY"] = -1
		target["distanceToBottomY"] = -1
		target["depth"] = -1
		previousStates["transform"] = None
		previousStates["xPos"] = -1
		previousStates["yPos"] = -1
		previousStates["area"] = -1
		previousStates["depth"] = -1
		returnValue = True
	else:
		returnValue = False


	return ResetTargetServiceResponse(returnValue)





# callback function for contour_moments
def callbackContouMoments(receivedMsg):
	global target

	#Setting initial values for min x and y of a object
	currentMinX = imageDimensions["xMax"]
	currentMinY = imageDimensions["yMax"]

	#Checking if we see anything with the color filter
	if len(receivedMsg.moments) > 0:

		#Going through all moments to check which one is the most left one
		for moment in receivedMsg.moments:
			if  moment.center.y < currentMinY:
				currentMinX = moment.center.x
				currentMinY = moment.center.y
				target["area"] = moment.area
				previousStates["area"] = moment.area
				target["length"] = moment.length
		
		#Assigning the minimum x and y pos as the positions
		target["xPos"] = currentMinX
		target["yPos"] = currentMinY
		previousStates["xPos"] = currentMinX
		previousStates["yPos"] = currentMinY

		#Calculating the pixel distance to the X center and Y bottom
		target["distanceToCenterX"] = ((imageDimensions["xMax"]/2) - target["xPos"])
		target["distanceToCenterY"] = ((imageDimensions["yMax"]/2) - target["yPos"])
		target["distanceToBottomY"] = imageDimensions["yMax"] - target["yPos"]

	#Otherwise set the values to -1 to tell that nothing has been found
	else:
		target["xPos"] = -1
		target["yPos"] = -1
		target["area"] = -1
		target["length"] = -1
		target["distanceToCenterX"] = -1
		target["distanceToCenterY"] = -1
		target["distanceToBottomY"] = -1





# callback function for depth_camera
def callbackDepthCamera(receiverMsg):
	global target, bridge, t, br, previousStates, showDebug

	cv_image = bridge.imgmsg_to_cv2(receiverMsg, "32FC1")

	#Check if we see the target otherwise do not calculate depth
	if target["xPos"] != -1 and target["yPos"] != -1:
		try:
			depth = cv_image[int(target["yPos"]), int(target["xPos"])]

			if math.isfinite(depth):
				target["depth"] = depth
				previousStates["depth"] = depth

				#Checking if the area of the target is bigger than 1500 then create transform
				if target["area"] > 1000:
					#Building the transform
					t.header.stamp = rospy.Time.now()
					t.header.frame_id = "head_camera_depth_frame"
					t.child_frame_id = "target_object"
					t.transform.translation.x = depth
					t.transform.translation.y = (((target["xPos"]-(imageDimensions["xMax"]/2))/554.254691191187)*depth)*-1
					t.transform.translation.z = (((target["yPos"]-(imageDimensions["yMax"]/2))/554.254691191187)*depth)*-1

					#quaternion calculation
					q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
					t.transform.rotation.x = q[0]
					t.transform.rotation.y = q[1]
					t.transform.rotation.z = q[2]
					t.transform.rotation.w = q[3]

					#Sending transform and saving previous
					br.sendTransform(t)
					previousStates["transform"] = t
				else:
					if previousStates["transform"] != None:
						#Updating sent time
						previousStates["transform"].header.stamp = rospy.Time.now()
						br.sendTransform(previousStates["transform"])

			else:
				target["depth"] = -1

			if showDebug:
				cv2.circle(cv_image, (int(target["xPos"]), int(target["yPos"])), 5, 255)
				cv2.imshow("Image window", cv_image)
				cv2.waitKey(3)

		except CvBridgeError as e:
			print(e)

	else:
		if previousStates["xPos"] != -1 and previousStates["yPos"] != -1 and previousStates["area"] != -1:

			if previousStates["transform"] != None:
				#Updating sent time
				previousStates["transform"].header.stamp = rospy.Time.now()
				br.sendTransform(previousStates["transform"])
			if showDebug:
				cv2.circle(cv_image, (int(previousStates["xPos"]), int(previousStates["xPos"])), 5, 255)
				cv2.imshow("Image window", cv_image)
				cv2.waitKey(3)





#_____________________METHODS_____________________

#function to send the Object position
def sendingObjectPosition(pub):
	global target

	#Checking if xPos, yPos and area have values otherwise don't publish
	if target["xPos"] != -1 and target["yPos"] != -1 and target["area"] != -1:

		#Send message
		sendingMsg = ObjectPosition()
		sendingMsg.xPos = target["xPos"]
		sendingMsg.yPos = target["yPos"]
		sendingMsg.area = target["area"]
		sendingMsg.length = target["length"]
		sendingMsg.distanceToCenterX = target["distanceToCenterX"]
		sendingMsg.distanceToCenterY = target["distanceToCenterY"]
		sendingMsg.distanceToBottomY = target["distanceToBottomY"]
		sendingMsg.depth = target["depth"]

		pub.publish(sendingMsg)





#Function used to setup all publisher subscribers and so on
def setup():

	#Initiating node
	rospy.init_node('targetAllocation', anonymous=True)

	#Subscribing to /contour_moments/moments to get the Laserscanner information
	rospy.Subscriber('/contour_moments/moments', MomentArrayStamped, callbackContouMoments)

	#Subscribing to /head_camera/depth_registered/image_raw to acces depth camera data
	rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, callbackDepthCamera)

	#Creating the publisher to publish to /cmd_vel
	pub = rospy.Publisher("/objectPosition", ObjectPosition, queue_size=10)

	#Starting to /stateMachine_setState service to receive requests
	service = rospy.Service("targetAllocation_reset", ResetTargetService, callbackRequest)

	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		sendingObjectPosition(pub)
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass