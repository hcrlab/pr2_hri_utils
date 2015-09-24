#!/usr/bin/env python

import rospy
import pr2_point
from pr2_point.point_service import PointService
from pr2_point.srv import PR2Point

if __name__ == '__main__':
    rospy.init_node('pr2_point_service', anonymous=True)

    arm_index = rospy.get_param('/point_service_node/arm_index')
    is_calibration = rospy.get_param('/point_service_node/is_calibration')
    pose_file_dir = '{}/data'.format(str(rospack.get_path('pr2_point')))
    pose_file_dir = rospy.get_param('pose_file_dir', pose_file_dir)


    if arm_index == 2:
        ps0 = PointService(arm_index=0, pose_file_dir)
        ps1 = PointService(arm_index=1, pose_file_dir)

        if is_calibration:
            rospy.loginfo('Calibrating right arm.')
            ps0.start_calibration()
            while(not rospy.is_shutdown() and ps0.is_calibrating):
                ps0.update()

            rospy.loginfo('Calibrating left arm.')
            ps1.start_calibration()
            while(not rospy.is_shutdown() and ps1.is_calibrating):
                ps1.update()


        ps0.load_pointing_poses()
        ps1.load_pointing_poses()
        ps0.move_to_neutral()
        ps1.move_to_neutral()

        rospy.Service(
                  'pr2_point_service/arm0',
                  PR2Point,
                  ps0.point_cb)

        rospy.Service(
                  'pr2_point_service/arm1',
                  PR2Point,
                  ps1.point_cb)

    else:
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
