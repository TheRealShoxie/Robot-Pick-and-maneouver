#!/usr/bin/env python

#This Python file is used to publish the state machine and hold its value





#_____________________IMPORTS_____________________

#Importing needed libraries, msg and srv types
import rospy
from ibo1_ros360.msg import StateMachine
from ibo1_ros360.srv import StateMachineService, StateMachineServiceResponse





#_____________________GLOBAL VARIABLES_____________________

#Defining global variables
robotState = -1





#_____________________CALLBACK FUNCTIONS_____________________

#Callback used to deal with Requests
def callbackRequest(request):
	global robotState

	robotState = request.changeState
	return StateMachineServiceResponse(True)





#_____________________METHODS_____________________

#Function stateMachine which publishes the stateMachine
def stateMachine(pub):
	#Making global variables usable
	global robotState

	#Creating the sendingMsg with its corresponding type
	sendingMsg = StateMachine()

	#Assigning the robotState to the msg
	sendingMsg.currentState = robotState

	#publishing the state of the robot
	pub.publish(sendingMsg)





#Function used to setup all publisher subscribers and so on
def setup():
	rospy.init_node('stateMachine_setState', anonymous=True)

	#Creating a new publisher named stateMachine
	pub = rospy.Publisher('stateMachine', StateMachine, queue_size=10)

	#Starting to /stateMachine_setState service to receive requests
	service = rospy.Service("stateMachine_setState", StateMachineService, callbackRequest)

	
	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		stateMachine(pub)
		rate.sleep()
	rospy.spin()
   
if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass