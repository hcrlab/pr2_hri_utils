#!/usr/bin/env python

import rospy
import pr2_point
from pr2_point.point_service import PointService
from pr2_point.srv import PR2Point

if __name__ == '__main__':
    rospy.init_node('pr2_point_service', anonymous=True)
    
    arm_index = rospy.get_param('/point_service_node/arm_index')
    is_calibration = rospy.get_param('/point_service_node/is_calibration')
    
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
