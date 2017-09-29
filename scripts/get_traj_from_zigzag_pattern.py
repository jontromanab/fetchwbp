#!/usr/bin/env python
from openravepy import *
import fetchpy
import rospy

from fetchwbp import util, patterns, planner
from fetchwbp.plotting import plottingPoints

#from MJwbp.planner import MJWBPlanner



if __name__ == '__main__':
	rospy.init_node('fetchpy')
	fetch_args = {'sim':True, 'viewer':'qtcoin'}
	env, robot = fetchpy.initialize(**fetch_args)
	viewer = env.GetViewer()
	originaxes = misc.DrawAxes(env, [1,0,0,0,0,0,0], dist = 1, linewidth= 2)
	keep_going = True



	to_Table = ([0., 0.70503065, -0.81321057,  0.44084394,  1.52903305, -0.37976212,0.92392059,  0.8291418])
	robot.arm_torso.PlanToConfiguration(to_Table, execute = True) 
	raw_input("Press enter to continue...")


	poses = patterns.createZigZagPattern(util.transformToPose(util.getTransform(robot,'gripper_link')),200,3)
	handles = []
	pl = plottingPoints(env,handles)
	pl.plotPoints(poses, 0.005, color = 'pink')
	raw_input("Press enter to continue...")

	#Take robot to start position of the trajectory
		


	#while keep_going:
