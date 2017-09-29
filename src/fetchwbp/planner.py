import numpy as np
import time
from openravepy import *


class MJWBPlanner:
	"""Whole body planner by merged jacobian. Returns a whole body trajectory 
	that can be direcly executed in Fetchpy"""
	def __init__(self, robot):
		self.robot = robot
		self.manip = self.robot.SetActiveManipulator('arm_torso')
		self.basemanip = interfaces.BaseManipulation(self.robot)

	"""Returns velocity given start,end pose and time"""
	def getVel(start_pose, end_pose, unitTime = 1.0):
		return (end_pose-start_pose)/unitTime
	

		
	"""Bounds joint velocitu limits"""
	def boundVelocity(robot, velocity, joint_velocity_limits=None):
		self.robot.SetActiveDOFs(self.manip.GetArmIndices())
		if joint_velocity_limits is None:
			joint_velocity_limits = self.robot.GetActiveDOFMaxVel()
		elif joint_velocity_limits:
			joint_velocity_limits = numpy.array([numpy.PINF] * self.robot.GetActiveDOF())
		new_vel = []
		for i in range(len(joint_velocity_limits)):
			if velocity[i] > joint_velocity_limits[i]:
				new_vel.append(joint_velocity_limits[i])
			else:
				new_vel.append(velocity[i])
		return numpy.array(new_vel)


	"""Calculates Jacobian of the arm"""
	def calculateJacobianArm(robot):
		#manip = robot.GetActiveManipulator()
		jacob_spatial = self.manip.CalculateJacobian()
		jacob_angular = self.manip.CalculateAngularVelocityJacobian()
		jacob = numpy.vstack((jacob_spatial, jacob_angular))
		return jacob 

	"""Calculates Jacobian of the base"""
	def calculateJacobianBase(robot):
		l = 0.37476
		r = 0.0225
		jacob_base = np.array(([r/2, r/2],[0,0],[0,0],[0,0],[0,0],[-r/l,r/l]))
		return jacob_base

	""" Returns full jacobian of arm and base combined"""
	def calculateFullJacobian(robot):
		jacob_arm = self.calculateJacobianArm(robot)
		jacob_base = self.calculateJacobianBase(robot)
		full_jacob = numpy.hstack((jacob_arm, jacob_base))
		return full_jacob

	"""Checks if velocity is within limit"""
	def isInLimit(vel, bounds):
		for i in range(len(vel)):
			if(vel[i]>=bounds[i][1] or vel[i]<=bounds[i][0]):
				return False

	"""Returns cartesian position given current affine displacement"""
	def calculateBaseGoalCart(q_dot_base):
		jacob_base = self.calculateJacobianBase(robot)
		cart_vel = numpy.dot(jacob_base, q_dot_base)
		TeeBase = self.robot.GetTransform()
		trns = matrixFromAxisAngle([0,0,cart_vel[-1]])
		trns[0:3, 3] = [cart_vel[0],0, 0]
		goal = np.dot(TeeBase,trns)
		transl = goal[0:3, 3]
		angles = axisAngleFromRotationMatrix(goal)
		cart_vel2 = [transl[0],0.0, angles[-1]]
		return cart_vel2

	
	"""Returns filted joint velocity, for every step it checks for all the values of velocity
	and if gets more than the limit, it sets value to that row in jacobian and recalculates again"""
	def getQDot(robot, pose, unitTime, joint_velocity_limits=None):
		joint_limit_tolerance=3e-2
		manip = robot.GetActiveManipulator()
		vel = getVel(transformToPose(manip.GetEndEffectorTransform()), pose, unitTime)
		full_jacob = calculateFullJacobian(robot)
		jacob_inv = np.linalg.pinv(full_jacob)
		q_dot = numpy.dot(jacob_inv, vel)
		if joint_velocity_limits is None:
			joint_velocity_limits = robot.GetActiveDOFMaxVel()
		elif joint_velocity_limits:
			joint_velocity_limits = numpy.array([numpy.PINF] * robot.GetActiveDOF())
		bounds = numpy.column_stack((-joint_velocity_limits, joint_velocity_limits))
		q_curr = robot.GetActiveDOFValues()
		q_min, q_max = robot.GetActiveDOFLimits()
		dq_bounds = [(0., max) if q_curr[i] <= q_min[i] + joint_limit_tolerance else
		(min, 0.) if q_curr[i] >= q_max[i] - joint_limit_tolerance else
		(min, max) for i, (min, max) in enumerate(bounds)]
		q_dot_arm = q_dot[:-2]
		curr_values = manip.GetArmDOFValues()
		changed_values = curr_values + q_dot_arm * 1.0
		value = 0
		while(isInLimit(changed_values,dq_bounds) == False):
			value = value+1
			for i in range(len(q_dot_arm)):
				if(changed_values[i]>=dq_bounds[i][1]-joint_limit_tolerance or changed_values[i]<=dq_bounds[i][0]+joint_limit_tolerance):
					full_jacob[:,i] = 0.0
					jacob_inv = np.linalg.pinv(full_jacob)
					q_dot = numpy.dot(jacob_inv, vel)
					q_dot_arm = q_dot[:-2]
					curr_values = manip.GetArmDOFValues()
					changed_values = curr_values + q_dot_arm * 1.0
		return q_dot


	""" Obtain the next joint vales(arm) and affine values(base) to execute"""
	def getGoalToExecute(robot, pose, unitTime):
		#manip = robot.GetActiveManipulator()
		q_dot = getQDot(robot, pose, unitTime,joint_velocity_limits=numpy.PINF)
		q_dot_arm = q_dot[:-2]*0.25
		q_dot_base = q_dot[-2:]
		curr_values = self.manip.GetArmDOFValues()
		changed_values = curr_values + q_dot_arm * unitTime
		cart_vel = calculateBaseGoalCart(q_dot_base)
		finalgoal = numpy.hstack((changed_values, cart_vel))
		return q_dot_arm, finalgoal

	"""Execution. This function is called on every iteration
	On each step, it calculates the joint, affine values from the jacobian and moves the robot
	to the next point"""
	def executeVelPath(robot, pose, handles, unitTime = 1.0,joint_velocity_limits=None):
		manip = robot.SetActiveManipulator('arm_torso')
		basemanip = interfaces.BaseManipulation(robot)
		with robot:
			Tee = manip.GetEndEffectorTransform()
			TeeBase = robot.GetTransform()
			Tee_gripper = getTransform('gripper_link')
			jointnames=['torso_lift_joint','shoulder_pan_joint','shoulder_lift_joint','upperarm_roll_joint','elbow_flex_joint','forearm_roll_joint','wrist_flex_joint','wrist_roll_joint']
			robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis)
			robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
			robot.SetAffineRotationAxisMaxVels(np.ones(4))

			arm_vel, finalgoal = getGoalToExecute(robot, pose, unitTime)
			basemanip.MoveActiveJoints(goal=finalgoal,maxiter=5000,steplength=1,maxtries=2)

			pl.plotPoint(transformToPose(TeeBase), 0.01, yellow)
			pl.plotPoint(transformToPose(Tee_gripper), 0.01, blue)
		waitrobot(robot)
		return transformToPose(Tee), transformToPose(TeeBase), arm_vel, finalgoal













