#!/usr/bin/env python

import rospy
from pr2_point.srv import PR2Point

if __name__ == '__main__':
    rospy.init_node('pr2_point_service_test', anonymous=True)

    rospy.wait_for_service('pr2_point_service')
    point_srv = rospy.ServiceProxy('pr2_point_service',
                                       PR2Point)
    rospy.loginfo('Got response from the PR2 pointing service...')
    
    try:
        point_srv(0.5, 0.5, 1.0)
        point_srv(0, 0, 1.0)
        point_srv(0, 1, 1.0)
        point_srv(1, 1, 1.0)
        point_srv(0, 1, 1.0)
        point_srv(-1, -1, 1.0)
        
    except rospy.ServiceException:
        rospy.logerr('PR2 point service call failed.')