#!/usr/bin/env python
from openravepy import *
import fetchpy
import rospy
import numpy as np
import time

from fetchwbp import util, patterns
from fetchwbp.planner import MJWBPlanner
from fetchwbp.plotting import plottingPoints

#from MJwbp.planner import MJWBPlanner
def waitrobot(robot):
		while not robot.GetController().IsDone():
			time.sleep(0.1)


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
	raw_input("Press enter to continue")

	manip = robot.SetActiveManipulator('arm_torso')
	basemanip = interfaces.BaseManipulation(robot)
	print 'Transform now:'
	Tee = manip.GetEndEffectorTransform()

	jointnames=['torso_lift_joint','shoulder_pan_joint','shoulder_lift_joint','upperarm_roll_joint','elbow_flex_joint','forearm_roll_joint','wrist_flex_joint','wrist_roll_joint']
	#robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
	robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis)
	arm_curr = robot.GetDOFValues(manip.GetArmIndices())
	#robot.base.Forward(0.1, execute = True)
	#sol = manip.FindIKSolution(Tee, IkFilterOptions.CheckEnvCollisions)
	while(keep_going):
		print 'starting loop'
		print 'going forward'
		#raw_input("Press enter to continue...")
		first_sol = [0.29377777, 0.83095013, -0.16160664, 0.44084394, 1.52903305, -1.16335257, 0.4771901, 1.58043013]
		first_sol_base = [0.1, 0, 0]
		first_sol.extend(first_sol_base)
		basemanip.MoveActiveJoints(goal=first_sol,maxiter=5000,steplength=10,maxtries=2)
		#print robot.GetDOFVelocities(manip.GetArmIndices())
		#robot.base.Forward(0.1, execute = True)
		waitrobot(robot)
		#print sol
		#robot.arm.PlanToEndEffectorPose(Tee,execute = True)

		print 'going back center'
		#raw_input("Press enter to continue...")
		#second_sol = [ 2.16293471e-04, 7.04974705e-01,-8.12747977e-01, 4.40843940e-01,1.52903305e+00, -3.80069727e-01, 9.23528643e-01, 8.29270090e-01]
		center_sol = []
		center_sol.extend(arm_curr)
		center_sol_base = [0, 0, 0]
		center_sol.extend(center_sol_base)
		basemanip.MoveActiveJoints(goal=center_sol,maxiter=5000,steplength=10,maxtries=2)
		#print robot.GetDOFVelocities(manip.GetArmIndices())
		#robot.base.Forward(-0.1, execute = True)
		waitrobot(robot)
		

	
	
		#robot.base.Forward(-0.2, execute = True)
		#sol = manip.FindIKSolution(Tee, IkFilterOptions.CheckEnvCollisions)
		print 'going back 2nd'
		#raw_input("Press enter to continue...")
		third_sol = [ 0.14338345, 0.61748536, -0.39198779, 0.44084394, 1.22903305, -0.55902202, 0.83521167, 0.74906054]
		third_sol_base = [-0.1, 0, 0]
		third_sol.extend(third_sol_base)
		
		basemanip.MoveActiveJoints(goal=third_sol,maxiter=5000,steplength=10,maxtries=2)
		#print robot.GetDOFVelocities(manip.GetArmIndices())
		#robot.base.Forward(-0.1, execute = True)
		waitrobot(robot)
		#print sol
		#robot.arm.PlanToEndEffectorPose(Tee,execute = True)
	
	
		#robot.base.Forward(0.1, execute = True)
		#sol = manip.FindIKSolution(Tee, IkFilterOptions.CheckEnvCollisions)
		print 'going back center'
		#raw_input("Press enter to continue...")
		#forth_sol = [ 2.16293471e-04, 7.04974705e-01,-8.12747977e-01, 4.40843940e-01,1.52903305e+00, -3.80069727e-01, 9.23528643e-01, 8.29270090e-01]
		basemanip.MoveActiveJoints(goal=center_sol,maxiter=5000,steplength=10,maxtries=2)
		waitrobot(robot)


		print 'Rotating now +0.5'
		fifth_sol = [0.1833299, 0.17957563, -0.41729582, 0.44084394,1.52903305, -0.74254459,0.61490822,  1.06765718]
		fifth_sol_base = [0, 0, 0.5]
		fifth_sol.extend(fifth_sol_base)
		basemanip.MoveActiveJoints(goal=fifth_sol,maxiter=5000,steplength=10,maxtries=2)
		#print robot.GetDOFVelocities(manip.GetArmIndices())
		waitrobot(robot)
		#robot.base.Rotate(0.5, execute = True)
		#sol = manip.FindIKSolution(Tee, IkFilterOptions.CheckEnvCollisions)
		#print sol

		print 'going back center'
		#raw_input("Press enter to continue...")
		#sixth_sol = [ 2.16293471e-04, 7.04974705e-01,-8.12747977e-01, 4.40843940e-01,1.52903305e+00, -3.80069727e-01, 9.23528643e-01, 8.29270090e-01]
		basemanip.MoveActiveJoints(goal=center_sol,maxiter=5000,steplength=10,maxtries=2)
		waitrobot(robot)


		print 'Rotating now -0.5'
		#robot.base.Rotate(-0.5, execute = True)
		#sol = manip.FindIKSolution(Tee, IkFilterOptions.CheckEnvCollisions)
		seventh_sol = [0.17738291,1.22278736, -0.39754901, 0.44084394,1.42903305,-0.67994824,0.67709178,  1.02158291]
		seventh_sol_base = [0, 0, -0.5]
		seventh_sol.extend(seventh_sol_base)
		basemanip.MoveActiveJoints(goal=seventh_sol,maxiter=5000,steplength=10,maxtries=2)
		#print robot.GetDOFVelocities(manip.GetArmIndices())
		waitrobot(robot)
		#print sol

		print 'going back center'
		#raw_input("Press enter to continue...")
		#sixth_sol = [ 2.16293471e-04, 7.04974705e-01,-8.12747977e-01, 4.40843940e-01,1.52903305e+00, -3.80069727e-01, 9.23528643e-01, 8.29270090e-01]
		basemanip.MoveActiveJoints(goal=center_sol,maxiter=5000,steplength=10,maxtries=2)
		waitrobot(robot)
		
		