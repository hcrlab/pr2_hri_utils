#!/usr/bin/env python

import rospy
from pr2_point.point_service import PointService
from pr2_point.srv import PR2Point

if __name__ == '__main__':
    rospy.init_node('pr2_point_service', anonymous=True)
    
    ps = PointService()

    rospy.Service(
              'pr2_point_service',
              PR2Point,
              ps.point_cb)

    rospy.spin()