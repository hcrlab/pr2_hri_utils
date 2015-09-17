#!/usr/bin/env python

import rospy
from pr2_point.point_service import PointService
from pr2_point.srv import PR2Point

if __name__ == '__main__':
    rospy.init_node('pr2_point_service', anonymous=True)
    
    arm_index = 0
    is_calibration = True
    
    ps = PointService(arm_index=arm_index)

    if is_calibration:
    	ps.start_calibration()
	    while(not rospy.is_shutdown() and ps.is_calibrating):
	    	ps.update()

	ps.load_pointing_poses()
    rospy.Service(
              'pr2_point_service',
              PR2Point,
              ps.point_cb)

    rospy.spin()
