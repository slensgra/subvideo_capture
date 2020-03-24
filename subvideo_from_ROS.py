import rospy
import sensor_msgs.msg
import sys
import cv_bridge
from __future__ import print_function

import utils

class SubvideoROS:
    def __init__(self):
        rospy.init_node('subvideo_watcher')
        self.parser = utils.initialize_argument_parser()

        if not parser.topic:
            print("subvideo_from_ROS.py requires that the 'topic' argument is set.")
            self.parser.print_help()
            sys.exit(1)

        self.subscriber = rospy.Subscriber(
            parser.topic,
            sensor_msgs.msg.Image,
            self.image_callback,
            queue_size=1
        )

        self.bridge = cv_bridge.CvBridge()

    def spin(self):
        rospy.spin()

    def image_callback(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message)

        scaled_image_color = cut_and_scale_image(
            raw_image=raw_image,
            min_corner_x=parser.min_corner_x,
            min_corner_y=parser.min_corner_y,
            width=parser.width,
            height=parser.height,
            scale_factor=parser.scale_factor,
        )

        cv2.imshow("file", image)
        cv2.waitKey(0)
        

if __name__ == "__main__":
    subvideo_instance = SubvideoROS()
    subvideo_instance.spin()
