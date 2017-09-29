#!/usr/bin/env python
from openravepy import *
import fetchpy
import rospy
import numpy as np

from fetchwbp import util, patterns
from fetchwbp.planner import MJWBPlanner
from fetchwbp.plotting import plottingPoints

#from MJwbp.planner import MJWBPlanner



if __name__ == '__main__':
	rospy.init_node('fetchpy')
	fetch_args = {'sim':True, 'viewer':'qtcoin'}
	env, robot = fetchpy.initialize(**fetch_args)
	viewer = env.GetViewer()
	originaxes = misc.DrawAxes(env, [1,0,0,0,0,0,0], dist = 1, linewidth= 2)
	keep_going = True


	#Moving the arm to a proper pose to start trajectory
	to_Table = ([0., 0.70503065, -0.81321057,  0.44084394,  1.52903305, -0.37976212,0.92392059,  0.8291418])
	robot.arm_torso.PlanToConfiguration(to_Table, execute = True) 
	raw_input("Press enter to Create Pattern")

	#Creating pattern and plot the points we need our robot to go through, starting from the current 
	#pose of the gripper
	poses = patterns.createZigZagPattern(util.transformToPose(util.getTransform(robot,'gripper_link')),200,1)
	handles = []
	pl = plottingPoints(env,handles)
	pl.plotPoints(poses, 0.005, color = 'pink')
	raw_input("Press enter to Start Executing Pattern")

	#Obtaining ee pose for each point in pattern
	stat_trns = util.getTransformBetweenLinks(robot,'wrist_roll_link','gripper_link')
	new_poses = []
	for i in poses:
		trns = util.poseToTransform(i)
		new_trns = np.dot(trns, stat_trns)
		new_poses.append(util.transformToPose(new_trns))

	#send EE pattern point to the planner
	planner = MJWBPlanner(robot, handles)
	all_poses = planner.executePath(new_poses, 500, handles, traj_name = 'my_traj.xml')


	print 'I have planned the trajectory and it is saved'
	raw_input("Press enter to Exit")



	#Take robot to start position of the trajectory
		


	#while keep_going:
