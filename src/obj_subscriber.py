#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Subscriber:
    def __init__(self):
        self.rgb_img = []
        self.rgb_switch = 0
        self.rgb_flag = 0

        self.depth_img = []
        self.depth_switch = 0
        self.depth_flag = 0

        self.sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_rgb, queue_size=5)
        self.sub_depth = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, self.callback_depth, queue_size=5)
        rospy.sleep(1)

    def callback_rgb(self,data):    # more intensive than using wait_for_message, but necessary to keep camera running
        
        if self.rgb_switch == 0:
            pass
        else:
            # Use cv_bridge() to convert the ROS image to OpenCV format
            # frame = self.cvbridge.imgmsg_to_cv2(data, "bgr8")
            # frame = np.array(frame, dtype=np.uint8)
            self.rgb_img=data
            self.rgb_flag = 0

    def callback_depth(self,data):    
        if self.depth_switch == 0:
            pass
        else:
            # frame = self.cvbridge.imgmsg_to_cv2(data) #, "32FC1"
            # frame = np.array(frame, dtype=np.float32)
            self.depth_img = data
            self.depth_flag = 0

    def update_feed(self):
        #update stored rgb_img and depth_img
        self.depth_flag = self.depth_switch = 1
        self.rgb_flag = self.rgb_switch = 1

        #turn off both callbacks once they have each been called at least once
        #ensures the rgb and depth images match up
        while ((self.depth_flag == 1) | (self.rgb_flag == 1)):
            pass
        self.depth_switch = self.rgb_switch = 0

        
def main():
    rospy.init_node("obj_camera_subscriber")
    subscriber = Subscriber()
    rate = rospy.Rate(2) #rgb/image_raw publishes ~10hz, /camera/depth_registered/hw_registered/image_rect publishes ~30hz
    while not rospy.is_shutdown():
        subscriber.update_feed()
        print "== Subscriber: Stored feed updated"
        rate.sleep()
    



if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        cleanup()