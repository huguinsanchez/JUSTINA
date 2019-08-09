#! /usr/bin/env python
import sys
from localization.srv import *
from navig_msgs.srv import prob_localization
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped as pwcs
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from oct2py import octave
import numpy as np
import time
import math

from scipy.stats import multivariate_normal

octave.addpath('/usr/share/octave/packages/statistics-1.2.4/')
global cov
global mean
global limits_dict_min
global limits_dict_max
global cov_pc
global client_mvncdf

def truncate(number, digits):
	stepper = 10.0 ** digits
	return math.trunc(stepper * number) / stepper
#def mvn_cdf(xl,xu,mean,cov,step):
#	aux_x=0.0
#	sum=0.0
#	step_sq=step*step
#	print len(np.arange(xl[0],xu[0],step))
#	for i in np.arange(xl[0],xu[0],step):
#		#aux_x=multivariate_normal.pdf(,mean=mean, cov=cov)
#		for j in np.arange(xl[1],xu[1],step):
#			#print "entre" + str(i)
#			sum+=step_sq*multivariate_normal.pdf([i,j],mean=mean, cov=cov)
#			#print sum
#	return sum

def  handle_prob_loc(req):
	global cov
	global mean
	global limits_dict_min
	global limits_dict_max
	global cov_pc
	global client_mvncdf
	result=Float64()
	cov[0][0]=truncate(cov[0][0],6)
	cov[0][1]=truncate(cov[0][1],6)
	cov[1][0]=truncate(cov[1][0],6)
	cov[1][1]=truncate(cov[1][1],6)
	#for i in cov:
	#	if i<0:
	#		i=0
	print "limit inf"+ str(limits_dict_min[req.room])
	print "limit max"+ str(limits_dict_max[req.room])
	print"mean(pose)"
	print mean
	#print "cov"
	#print cov
	#eigen_vals=np.linalg.eig(cov)
	#print"eigen values"
	#print eigen_vals
	#result.data=octave.mvncdf(limits_dict_min[req.room],limits_dict_max[req.room],mean,cov)
	print"cov pc"
	print cov_pc
	#print octave.mvncdf(limits_dict_min[req.room],limits_dict_max[req.room],mean,cov)
	aux=client_mvncdf(limits_dict_min[req.room][0],limits_dict_min[req.room][1],limits_dict_max[req.room][0],limits_dict_max[req.room][1],mean[0],mean[1],0.001,cov_pc[0][0],cov_pc[0][1],cov_pc[1][0],cov_pc[1][1])
	result.data=aux.prob
	print result
	return result

def limits(file):
	global limits_dict_min
	global limits_dict_max
	f=open(file,"r")
	line=f.readline()
	while line!="":
		aux=line.split()
		if len(aux)>3 and aux[1]=="room":
			x_min=aux[2]
			y_min=aux[3]
			x_max=x_min
			y_max=y_min
			for i in range (2, len(aux),2):
				if x_min>aux[i]:
					x_min=aux[i]
				elif x_max<aux[i]:
					x_max=aux[i]
				if y_min>aux[i+1]:
					y_min=aux[i+1]
				elif y_max<aux[i+1]:
					y_max=aux[i+1]
			array_min=np.array([float(x_min),float(y_min)])
			array_max=np.array([float(x_max),float(y_max)])
			limits_dict_min.update({aux[0]:array_min})
			limits_dict_max.update({aux[0]:array_max})
		line=f.readline()
	print"limits locations"
	print limits_dict_min
	print limits_dict_max

def callback_pose(msg):	
	global cov
	global mean
	cov=np.array(msg.pose.covariance, dtype=float).reshape((6,6))
	cov=cov[:2,:2]
	mean=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])
	#print mean

def callback_particlecloud(msg):
	global cov_pc
	#print"LOCALIZATION NODE PARTICLE CLOUD"
	#print msg.poses[0].position
	particlecloud=list()
	for i in range(len(msg.poses)):
		particlecloud.append([msg.poses[i].position.x,msg.poses[i].position.y])
	particlecloud=np.array(particlecloud)
	cov_pc=np.cov(particlecloud,rowvar=False)
	#print"particlecloud"
	#print particlecloud
	#print "cov_pc"
	#print cov_pc

def main(file):
	global limits_dict_min
	global limits_dict_max
	global client_mvncdf

	limits_dict_min=dict()
	limits_dict_max=dict()
	print"Initialiing Probability of position"
	print"File of locations: "+str(file)
	rospy.init_node('prob_loc_amcl', anonymous=True)
	rospy.Subscriber("/navigation/localization/amcl_pose", pwcs, callback_pose)
	rospy.Subscriber("/navigation/localization/particlecloud", PoseArray, callback_particlecloud)
	limits(file)
	rate=rospy.Rate(10)
	s=rospy.Service("/navigation/localization/prob_location", prob_localization, handle_prob_loc)
	client_mvncdf=rospy.ServiceProxy("/navigation/localization/mvncdf",MVNCDF)
	while not rospy.is_shutdown():

		rate.sleep()

if __name__ == '__main__':
	try:
		file=""
		if "-f" in sys.argv:
			file=sys.argv[sys.argv.index("-f") + 1]
		print (file)
		main(file)
	except rospy.ROSInterruptException:
		pass