import roslib
import rospy

from std_msgs.msg import String
import cv2
import sys, time
## Message
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import numpy as np
import open3d as od


# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber("chatter", String, callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

def image_handler(image_msg, depth_msg):
  # Solve all of perception here...
    print("image handler called")
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
    cv_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    o3d_color = o3d.geometry.Image(color)
    o3d_depth = o3d.geometry.Image(depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth)
    pointcloud = o3d.geometry.pointcloud.create_from_rgbd_image(self.rgbd_image, intrinsic)

    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    depth_message = bridge.cv2_to_imgmsg(cv_depth, encoding="passthrough")
    ## publish

def main(args):
    rospy.init_node('vision_project')
    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(image_handler)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
        

if __name__ == '__main__':
    main(sys.argv)
