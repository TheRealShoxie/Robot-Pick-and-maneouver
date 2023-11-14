This ReadMe explains on how to run my program.

IMPORTANT INFORMATION: 
		This folder contains _old_practical_folder_. This folder is not used for any
		logic of the program and only has the practical code in it. Fruther another world
		named world23 has been added which switches the positions of the pillars to test
		that the higher cube always gets picked up first.

1.) Place this folder within catkin/src 
2.) Ensure all needed files in folder scripts and launch are executable
3.) catkin_make the whole project
4.) Start roscore
5.) In a seperate terminal do following: roslaunch ibo1_ros360 assignment_rviz_gazebo.launch 
	This will start the gazebo environment and rviz. If rviz is not needed pls comment it out 
	inside the above mentioned launch file which can be found in the launch folder
6.) In a seperate terminal do following: rosrun ibo1_ros360 main.py
	This will start all the logic for running the program.
7.) Finished everything that needed to be done has been done