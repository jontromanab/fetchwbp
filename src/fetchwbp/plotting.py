from openravepy import *
import numpy as np

dict = {'red': [1,0,0,1],'blue': [0,0,1,1],
'pink': [1,0,0.5,1],'yellow': [1,1,0,1]}



class plottingPoints:
	def __init__(self, env, handles):
		self.env = env
		self.handles = handles

	def plotPoint(self, pose, size, color):
		point = np.array((pose[0:3]))
		self.handles.append(self.env.plot3(points = point, pointsize = size,colors = np.asarray(dict[color]), drawstyle = 1))

	def plotPoints(self, poses, size, color = 'red'):
		for i in poses:
			point = np.array((i[0:3]))
			self.handles.append(self.env.plot3(points = point, pointsize = size,colors = np.asarray(dict[color]), drawstyle = 1))