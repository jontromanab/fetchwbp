#!/usr/bin/env python
from openravepy import *
import fetchpy
import rospy
import numpy as np
import time

from prpy.rave import save_trajectory 

from fetchwbp import util, patterns
from fetchwbp.planner import MJWBPlanner
from fetchwbp.plotting import plottingPoints

#from MJwbp.planner import MJWBPlanner
def waitrobot(robot):
		while not robot.GetController().IsDone():
			time.sleep(0.5)


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
	Tee = manip.GetEndEffectorTransform()

	jointnames=['torso_lift_joint','shoulder_pan_joint','shoulder_lift_joint','upperarm_roll_joint','elbow_flex_joint','forearm_roll_joint','wrist_flex_joint','wrist_roll_joint']
	#robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
	robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis)

	cspec = robot.GetActiveConfigurationSpecification('linear')
	vel = np.zeros(11)
	curr_time = round(time.time() * 1000)
	traj = RaveCreateTrajectory(env, '')
	cspec.AddGroup('joint_velocities', dof= 8, interpolation='quadratic')
	cspec.AddGroup('affine_velocities', dof= 4, interpolation='next')
	cspec.AddDeltaTimeGroup()
	traj.Init(cspec)
	#robot.base.Forward(0.1, execute = True)
	#sol = manip.FindIKSolution(Tee, IkFilterOptions.CheckEnvCollisions)
	#while(keep_going):
	print 'starting loop'
	print 'going forward'
	curr_time = round(time.time() * 1000)
	value = []
	arm_curr = robot.GetDOFValues(manip.GetArmIndices())
	base_curr = [0,0,0]
	curr = []
	curr.extend(arm_curr)
	curr.extend(base_curr)
	value.extend(curr)
	value.extend(vel)
	value.extend([0.0])
	value.extend([0.0])
	traj.Insert(0, value)

	#going Forward
	forward = [0.29377777, 0.83095013, -0.16160664, 0.44084394, 1.52903305, -1.16335257, 0.4771901, 1.58043013]
	forward_base = [0.1, 0, 0]
	forward.extend(forward_base)
	basemanip.MoveActiveJoints(goal=forward,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(forward)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(1, value)


	# Coming back to center
	basemanip.MoveActiveJoints(goal=curr,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(curr)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(2, value)


	#Going back
	backward = [0.14338345, 0.61748536, -0.39198779, 0.44084394, 1.22903305, -0.55902202, 0.83521167, 0.74906054]
	backward_base = [-0.1, 0, 0]
	backward.extend(backward_base)
	basemanip.MoveActiveJoints(goal=backward,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(backward)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(3, value)


	# Coming back to center
	basemanip.MoveActiveJoints(goal=curr,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(curr)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(4, value)


	#Rotating +0.5
	rotate_pos = [0.1833299, 0.17957563, -0.41729582, 0.44084394,1.52903305, -0.74254459,0.61490822,  1.06765718]
	rotate_pos_base = [0,0,0.5]
	rotate_pos.extend(rotate_pos_base)
	basemanip.MoveActiveJoints(goal=rotate_pos,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(rotate_pos)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(5, value)

	# Coming back to center
	basemanip.MoveActiveJoints(goal=curr,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(curr)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(6, value)

	#Rotating -0.5
	rotate_neg = [0.17738291,1.22278736, -0.39754901, 0.44084394,1.42903305,-0.67994824,0.67709178,  1.02158291]
	rotate_neg_base = [0,0,-0.5]
	rotate_neg.extend(rotate_neg_base)
	basemanip.MoveActiveJoints(goal=rotate_neg,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(rotate_neg)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(7, value)

	# Coming back to center
	basemanip.MoveActiveJoints(goal=curr,maxiter=5000,steplength=10,maxtries=2)
	waitrobot(robot)
	value = []
	value.extend(curr)
	value.extend(vel)
	time_now = round(time.time() * 1000)
	dt = time_now - curr_time
	value.extend([dt/1000.])
	value.extend([dt/1000.])
	traj.Insert(8, value)

	save_trajectory(traj,'/home/abhi/Desktop/traj2/whole_body_ee_fixed_traj.xml')
		
		