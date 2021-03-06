import numpy as np
import time
from openravepy import *
from prpy.rave import save_trajectory 
import util
from plotting import plottingPoints
import rospkg



class MJWBPlanner:
	"""Whole body planner by merged jacobian. Returns a whole body trajectory 
	that can be direcly executed in Fetchpy"""
	def __init__(self, robot, handles):
		self.robot = robot
		self.manip = self.robot.SetActiveManipulator('arm_torso')
		self.basemanip = interfaces.BaseManipulation(self.robot)
		self.pl_ = plottingPoints(robot.GetEnv(),handles)


	def waitrobot(self):
		while not self.robot.GetController().IsDone():
			time.sleep(0.05)

	"""Returns velocity given start,end pose and time"""
	def getVel(self,start_pose, end_pose, unitTime = 1.0):
		return (end_pose-start_pose)/unitTime

	def discretizePath(self, poses, resolution):
		waypoints = []
		for i in range(len(poses)-1):
			mid_waypoints = self.createWayPoints(poses[i], poses[i+1], int(resolution/(len(poses)-1)))
			for j in mid_waypoints:
				waypoints.append(j)
		return waypoints

	def createWayPoints(self,start_pose, end_pose, resolution):
		poses=[]
		int_pose = (start_pose - end_pose)/resolution
		first_pose = start_pose
		poses.append(first_pose)
		for i in range(resolution-1):
			first_pose = first_pose - int_pose
			poses.append(first_pose)
		return poses
	

		
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
	def calculateJacobianArm(self):
		#manip = robot.GetActiveManipulator()
		jacob_spatial = self.manip.CalculateJacobian()
		jacob_angular = self.manip.CalculateAngularVelocityJacobian()
		jacob = numpy.vstack((jacob_spatial, jacob_angular))
		return jacob 

	"""Calculates Jacobian of the base"""
	def calculateJacobianBase(self):
		l = 0.37476
		r = 0.0225
		jacob_base = np.array(([r/2, r/2],[0,0],[0,0],[0,0],[0,0],[-r/l,r/l]))
		return jacob_base

	""" Returns full jacobian of arm and base combined"""
	def calculateFullJacobian(self):
		jacob_arm = self.calculateJacobianArm()
		jacob_base = self.calculateJacobianBase()
		full_jacob = numpy.hstack((jacob_arm, jacob_base))
		return full_jacob

	"""Checks if velocity is within limit"""
	def isInLimit(self, vel, bounds):
		for i in range(len(vel)):
			if(vel[i]>=bounds[i][1] or vel[i]<=bounds[i][0]):
				return False

	"""Returns cartesian position given current affine displacement"""
	def calculateBaseGoalCart(self, q_dot_base):
		jacob_base = self.calculateJacobianBase()
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
	def getQDot(self, pose, unitTime, joint_velocity_limits=None):
		joint_limit_tolerance=3e-2
		manip = self.robot.GetActiveManipulator()
		vel = self.getVel(util.transformToPose(self.manip.GetEndEffectorTransform()), pose, unitTime)
		full_jacob = self.calculateFullJacobian()
		jacob_inv = np.linalg.pinv(full_jacob)
		q_dot = numpy.dot(jacob_inv, vel)
		if joint_velocity_limits is None:
			joint_velocity_limits = self.robot.GetActiveDOFMaxVel()
		elif joint_velocity_limits:
			joint_velocity_limits = numpy.array([numpy.PINF] * self.robot.GetActiveDOF())
		bounds = numpy.column_stack((-joint_velocity_limits, joint_velocity_limits))
		q_curr = self.robot.GetActiveDOFValues()
		q_min, q_max = self.robot.GetActiveDOFLimits()
		dq_bounds = [(0., max) if q_curr[i] <= q_min[i] + joint_limit_tolerance else
		(min, 0.) if q_curr[i] >= q_max[i] - joint_limit_tolerance else
		(min, max) for i, (min, max) in enumerate(bounds)]
		q_dot_arm = q_dot[:-2]
		curr_values = manip.GetArmDOFValues()
		changed_values = curr_values + q_dot_arm * 1.0
		value = 0
		while(self.isInLimit(changed_values,dq_bounds) == False):
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
	def getGoalToExecute(self, pose, unitTime):
		#manip = robot.GetActiveManipulator()
		q_dot = self.getQDot(pose, unitTime,joint_velocity_limits=numpy.PINF)
		q_dot_arm = q_dot[:-2]*0.25
		q_dot_base = q_dot[-2:]
		curr_values = self.manip.GetArmDOFValues()
		changed_values = curr_values + q_dot_arm * unitTime
		cart_vel = self.calculateBaseGoalCart(q_dot_base)
		finalgoal = numpy.hstack((changed_values, cart_vel))
		return q_dot_arm, finalgoal

	"""Execution. This function is called on every iteration
	On each step, it calculates the joint, affine values from the jacobian and moves the robot
	to the next point"""
	def executeVelPath(self, pose, handles, unitTime = 1.0,joint_velocity_limits=None):
		with self.robot:
			Tee = self.manip.GetEndEffectorTransform()
			TeeBase = self.robot.GetTransform()
			Tee_gripper = util.getTransform(self.robot, 'gripper_link')
			jointnames=['torso_lift_joint','shoulder_pan_joint','shoulder_lift_joint','upperarm_roll_joint','elbow_flex_joint','forearm_roll_joint','wrist_flex_joint','wrist_roll_joint']
			self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in jointnames],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis)
			self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
			self.robot.SetAffineRotationAxisMaxVels(np.ones(4))
			arm_vel, finalgoal = self.getGoalToExecute(pose, unitTime)
			self.basemanip.MoveActiveJoints(goal=finalgoal,maxiter=5000,steplength=1,maxtries=2)
			self.pl_.plotPoint(util.transformToPose(TeeBase), 0.01, color = 'yellow')
			self.pl_.plotPoint(util.transformToPose(Tee_gripper), 0.01, color = 'blue')
		self.waitrobot()
		return util.transformToPose(Tee), util.transformToPose(TeeBase), arm_vel, finalgoal

	def executePath(self, path, resolution, handles, traj_name='whole_body_traj.xml'):
		dis_poses = self.discretizePath(path, resolution)
		poses = []
		base_poses = []
		all_poses = []
		all_poses.append(dis_poses)
		jointnames=['torso_lift_joint','shoulder_pan_joint','shoulder_lift_joint','upperarm_roll_joint','elbow_flex_joint','forearm_roll_joint','wrist_flex_joint','wrist_roll_joint']
		self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in jointnames],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis)
		cspec = self.robot.GetActiveConfigurationSpecification('linear')
		traj = RaveCreateTrajectory(self.robot.GetEnv(), '')
		cspec.AddGroup('joint_velocities', dof= 8, interpolation='quadratic')
		cspec.AddGroup('affine_velocities', dof= 4, interpolation='next')
		cspec.AddDeltaTimeGroup()
		traj.Init(cspec)
		#Creating the first point of the trajectory (the current joint values of the robot)
		arm_curr = self.robot.GetDOFValues(self.manip.GetArmIndices())
		base_curr = np.zeros(3)
		base_vel = np.zeros(4)
		arm_vel_cuur = np.zeros(8)
		dt = 0.
		value = []
		value.extend(arm_curr)
		value.extend(base_curr)
		value.extend(arm_vel_cuur)
		value.extend(base_vel)
		value.extend([dt])
		traj.Insert(0, value)
		curr_time = round(time.time() * 1000)

		for i in range(len(dis_poses)-1):
			pose, base_pose, arm_vel, finalgoal = self.executeVelPath(dis_poses[i+1], handles, unitTime = 1.0)
			# now creating other waypoints of the trajectory
			value = []
			value.extend(finalgoal[ :8])
			value.extend(finalgoal[-3: ])
			value.extend(arm_vel)
			value.extend(np.zeros(3))
			time_now = round(time.time() * 1000)
			dt = time_now - curr_time
			value.extend([dt/5000.])
			value.extend([dt/5000.])
			traj.Insert(i+1, value)
			poses.append(pose)
			base_poses.append(base_pose)
			rospack = rospkg.RosPack()
			path = rospack.get_path('fetchwbp')
			filename = path+'/trajectories/'+traj_name
			save_trajectory(traj,filename)
			all_poses.append(poses) 
			all_poses.append(base_poses) 
		return all_poses













