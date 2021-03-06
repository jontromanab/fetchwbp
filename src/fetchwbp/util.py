from openravepy import *
import numpy as np



def getTransform(robot, link_name):
	"""Returns transform of the given link"""
	return robot.GetLinkTransformations()[robot.GetLink(link_name).GetIndex()]

def transformToPose(transform):
	"""Returns a pose[displacements, angles] given homogeneous transformation"""
	transl = transform[0:3, 3]
	angles = axisAngleFromRotationMatrix(transform)
	pose = np.append(transl, angles)
	return pose

def poseToTransform(pose):
	"""Returns a homogeneous transformation given pose[displacements, angles]"""
	angles = pose[3:6]
	trns = matrixFromAxisAngle(angles)
	trns[0:3, 3] = pose[0:3]
	return trns

def getTransformBetweenLinks(robot,link1, link2):
    trns1 = robot.GetLinkTransformations()[robot.GetLink(link1).GetIndex()]
    trns2 = robot.GetLinkTransformations()[robot.GetLink(link2).GetIndex()]
    return np.dot(np.linalg.inv(trns2),trns1) 