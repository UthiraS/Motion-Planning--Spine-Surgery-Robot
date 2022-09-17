#!/usr/bin/env python
# coding: utf-8

import sys
import time
import csv
import numpy as np
import math


import rtde_control
# import rtde_io
# import rtde_receive
# import rtde.rtde as rtde
# import rtde.rtde_config as rtde_config
sys.path.append('..')

# rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")#Simulation
# rtde_io = rtde_io.RTDEIOInterface("127.0.0.1")
# rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")













    


def TrajectoryProfiling(set_joint_values):

	rtde_c = rtde_control.RTDEControlInterface("172.16.101.224")#UR5
	# rtde_io = rtde_io.RTDEIOInterface("172.16.101.224")
	# rtde_r = rtde_receive.RTDEReceiveInterface("172.16.101.224")		
	

	# print(set_joint_values)


    # move to home position
	# joint_q = [0,-1.57,1.57,0,1.57,3.14]
	# rtde_c.moveJ(joint_q,0.4,0.08)			

    #move to 1st waypoint
	print(set_joint_values[0][0],set_joint_values[0][1],set_joint_values[0][2],set_joint_values[0][3],set_joint_values[0][4],set_joint_values[0][5])
	rtde_c.moveJ([set_joint_values[0][0],set_joint_values[0][1],set_joint_values[0][2],set_joint_values[0][3],set_joint_values[0][4],set_joint_values[0][5]],0.4,0.5)	
	print(" ... ")

	velocity = 0.001 
	acceleration = 0.008
	dt = 0.8 # time taken to move from one joint waypoints to another (1-15 waypoints) (0.8 seconds)
	lookahead_time = 0.2 #varies ()
	gain = 300

    #move the robot through 17 waypoints using servoj
	i =0	
	while(i<20):
		# print(i)

		start = time.time()
		rtde_c.servoJ([set_joint_values[i][0],set_joint_values[i][1],set_joint_values[i][2],set_joint_values[i][3],set_joint_values[i][4],set_joint_values[i][5]],velocity, acceleration, dt, lookahead_time, gain)    
		end = time.time()
		duration = end - start

		if duration < dt:
		    time.sleep(dt - duration)
		i =i +1
		# #increasing the time between 15th and 16th waypoints and 16th and 17th waypoints (2 seconds)
		# if(i==15):
		# 	dt =2	


	#sleep for sometime before ending the script		
	time.sleep(1)
	rtde_c.servoStop()
	rtde_c.stopScript()
	
	# if(rtde_c.isConnected()):
	# 	rtde_c.disconnect()
	# print("Done!") 

	
def main():
	jv = np.array([[ 0.4292424 , -1.80642722,  0.99225691,  5.00878161, -1.6227311 ,
        0.01861445], [ 0.43844998, -1.84418927,  1.16341455,  4.87378818, -1.62149843,
        0.02531907], [ 0.44498659, -1.86563255,  1.30950678,  4.74736907, -1.61896575,
        0.02968598], [ 0.44802645, -1.87403996,  1.43759523,  4.62573212, -1.61470893,
        0.03100672], [ 0.44701072, -1.8711312 ,  1.55156366,  4.50673429, -1.6084303 ,
        0.02880928], [ 0.44160606, -1.85782448,  1.65373167,  4.38903284, -1.59993987,
        0.02282076], [ 0.43162525, -1.83456463,  1.74551584,  4.27175072, -1.58911812,
        0.01289636], [ 0.4170801 , -1.80143638,  1.82770319,  4.15431525, -1.57594607,
       -0.00093812], [ 0.39812083, -1.75828914,  1.90060197,  4.03642325, -1.56047871,
       -0.01854259], [ 0.37869293, -1.70331626,  1.96173038,  3.92057722, -1.550598  ,
       -0.03527227], [ 0.35541097, -1.63825553,  2.01324162,  3.80452901, -1.53876486,
       -0.05532983], [ 0.3288935 , -1.5630946 ,  2.05445535,  3.68904045, -1.52530323,
       -0.07819364], [ 0.29987558, -1.47819504,  2.08450305,  3.57541655, -1.51059975,
       -0.10324578], [ 0.2691785 , -1.38445659,  2.10246083,  3.46552328, -1.49508671,
       -0.12979613], [ 0.23772396, -1.28335087,  2.10748723,  3.36166933, -1.47924627,
       -0.15706651], [ 0.20642694, -1.17684028,  2.09899917,  3.26634743, -1.46355251,
       -0.18427896], [ 0.17618   , -1.06709403,  2.07676154,  3.18186133, -1.44846032,
       -0.21066553], [ 0.17618   , -1.06709403,  2.07676154,  3.18186133, -1.44846032,
       -0.21066553], [ 0.17491713, -0.94816322,  2.08030403,  3.05952306, -1.44783195,
       -0.21176927], [ 0.17365765, -0.82300984,  2.06672941,  2.94807968, -1.44720543,
       -0.21287021]])

	TrajectoryProfiling(jv)
	
	
	

main()
