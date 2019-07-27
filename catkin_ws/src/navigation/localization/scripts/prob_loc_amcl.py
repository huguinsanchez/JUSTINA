#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped as pwcs
from scipy.stats import multivariate_normal
import numpy as np


def callback_pose(msg):
	cov=np.array(msg.pose.covariance).reshape((6,6))
	cov=cov[:2,:2]
	mean=np.append(msg.pose.pose.position.x,msg.pose.pose.position.y)
	print mean

def main():
	print"Initialiing Probability of position"
	rospy.init_node('prob_loc_amcl', anonymous=True)
	rospy.Subscriber("/navigation/localization/amcl_pose", pwcs, callback_pose)
	rate=rospy.Rate(10)

	while not rospy.is_shutdown():

		rate.sleep()

if __name__ == '__main__':
  try:
      main()
  except rospy.ROSInterruptException:
      pass