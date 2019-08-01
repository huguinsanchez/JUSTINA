#! /usr/bin/env python
import sys
#from localization.srv import *
from navig_msgs.srv import prob_localization
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped as pwcs
from std_msgs.msg import Float32
from oct2py import octave
import numpy as np
import time
import math

octave.addpath('/usr/share/octave/packages/statistics-1.2.4/')
global cov
global mean
global limits_dict_min
global limits_dict_max

def truncate(number, digits):
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper

def  handle_prob_loc(req):
	global cov
	global mean
	global limits_dict_min
	global limits_dict_max
	result=Float32()
	cov[0][0]=truncate(cov[0][0],6)
	cov[0][1]=truncate(cov[0][1],6)
	cov[1][0]=truncate(cov[1][0],6)
	cov[1][1]=truncate(cov[1][1],6)
	#for i in cov:
	#	if i<0:
	#		i=0
	print"mean(pose)"
	print mean
	print "cov"
	print cov
	eigen_vals=np.linalg.eig(cov)
	print"eigen values"
	print eigen_vals
	result.data=octave.mvncdf(limits_dict_min[req.room],limits_dict_max[req.room],mean,cov)
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

def main(file):
	global limits_dict_min
	global limits_dict_max
	limits_dict_min=dict()
	limits_dict_max=dict()
	print"Initialiing Probability of position"
	print"File of locations: "+str(file)
	rospy.init_node('prob_loc_amcl', anonymous=True)
	rospy.Subscriber("/navigation/localization/amcl_pose", pwcs, callback_pose)
	limits(file)
	rate=rospy.Rate(10)
	s=rospy.Service("/navigation/localization/prob_location", prob_localization, handle_prob_loc)

	while not rospy.is_shutdown():

		rate.sleep()

if __name__ == '__main__':
	file="/home/hugo/tesis/JUSTINA/catkin_ws/src/planning/knowledge/navigation/known_delimitation_hugo.txt"
	try:
	    if "-f" in sys.argv:
        	file=sys.argv[sys.argv.index("-f") + 1]
	    main(file)
	except rospy.ROSInterruptException:
	    pass