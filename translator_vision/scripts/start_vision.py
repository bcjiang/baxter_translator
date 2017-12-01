#!/usr/bin/env python
import freenect
import cv2
import frame_convert2
import numpy as np
from std_msgs.msg import String
import rospy
#from matplotlib import pypltsplt
from geometry_msgs.msg import (PoseStamped,Pose,Point,Quaternion)

# some change
#kmnfwkonf

cv2.namedWindow('Depth')


def main():
    keep_running = True
    pub = rospy.Publisher('detected_hand', Pose, queue_size=5)
    rospy.init_node('translator_vision', anonymous=True)
    rate = rospy.Rate(60) # Hz

    while keep_running:
        depth_frame = np.array(get_depth())
        color_frame = np.array(get_video())
        #only works to about 90
        closest = find_min_idx(depth_frame)
        radius = 50
        cv2.circle(depth_frame, closest, radius, (0,255,0),4)
        cv2.imshow('Depth', depth_frame)

        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
        cur_pose = Pose(
        position=Point(x=closest[0], y=closest[1], z=0.05),
        orientation=overhead_orientation)
        pub.publish(cur_pose)

        if cv2.waitKey(10) == 27:
            keep_running = False

        print closest

    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)

    #     depth_frame = np.array(get_depth())
    #     color_frame = np.array(get_video())
    #     #only works to about 90
    #     closest = find_min_idx(depth_frame)
    #     radius = 50
    #     cv2.circle(depth_frame, closest, radius, (0,255,0),4)
    #     cv2.imshow('Depth', depth_frame)
    #     # if cv2.waitKey(10) == 27:
    #     #     keep_running = False

    #     print closest

    #     rate.sleep()


def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])


def get_video():
    return frame_convert2.video_cv(freenect.sync_get_video()[0])

def find_min_idx(x):
    k = x.argmin()
    ncol = x.shape[1]
    return k%ncol, k/ncol


if __name__ == "__main__":
    main()