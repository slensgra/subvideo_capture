from __future__ import print_function

import rospy
import sensor_msgs.msg
import sys
import cv_bridge
import cv2

import utils

class SubvideoROS:
    def __init__(self):
        rospy.init_node('subvideo_watcher')
        self.parser = utils.initialize_argument_parser()
        self.args = self.parser.parse_args()
        self.video_writer = None

        if not self.args.topic:
            print("subvideo_from_ROS.py requires that the 'topic' argument is set.")
            self.args.print_help()
            sys.exit(1)

        if self.args.outfile:
            cv2.VideoWriter(
                self.args.outfile,
                cv2.VideoWriter_fourcc('M','J','P','G'),
                20,
                (parser.width * parser.scale_factor, parser.height * parser.scale_factor)
            )


        self.subscriber = rospy.Subscriber(
            self.args.topic,
            sensor_msgs.msg.Image,
            self.image_callback,
            queue_size=1,
            buff_size=1024*10000
        )

        self.bridge = cv_bridge.CvBridge()

    def spin(self):
        rospy.spin()

    def image_callback(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message)

        scaled_image_color = utils.cut_and_scale_image(
            raw_image=image,
            min_corner_x=self.args.min_corner_x,
            min_corner_y=self.args.min_corner_y,
            width=self.args.width,
            height=self.args.height,
            scale_factor=self.args.scale_factor,
        )

        cv2.imshow("file", scaled_image_color)
        cv2.waitKey(10)
        

if __name__ == "__main__":
    subvideo_instance = SubvideoROS()
    subvideo_instance.spin()
