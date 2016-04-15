#!/usr/bin/env python

#This node subscribes to the pocketsphinx_recognizer/output topic and opens/closes nodes accordingly
#Currently does not support remapping

import rospy
import os
import subprocess
from std_msgs.msg import String

import rospkg

class PocketSphinxController:
    def __init__(self):
        self.sub = rospy.Subscriber('/pocketsphinx_recognizer/output', String, self.callback, queue_size=1)
        print "==pocketsphinx_controller: Subscriber Initialized"
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('liam_neeson')

    def callback(self,output):
        message = output.data
        print "\n==pocketsphinx_controller: Message: %s" %message
        if ('baxter' in message):
            if ('find' in message) & (('noodles' in message) | ('lunch' in message)):
                print "\n==Let's have lunch."
                os.system('rosnode kill obj_dregistered')
                rospy.sleep(2)
                
                # cmd = 'rosrun liam_neeson obj_dregistered.py '+path+'/src/classifier_data/maruchan_sample.png '+path+'/src/classifier_data/maruchan_asus15.xml'
                subprocess.Popen(['rosrun', 'liam_neeson','obj_dregistered.py',self.package_path+'/src/classifier_data/maruchan_sample.png',self.package_path+'/src/classifier_data/maruchan_asus15.xml'])
                # os.system('gnome-terminal -x '+cmd)
                # os.system('gnome-terminal -x sh -c "sleep 1; bash"')
                # os.system('rosrun liam_neeson obj_dregistered.py '+path+'/src/classifier_data/maruchan_sample.png '+path+'/src/classifier_data/maruchan_asus15.xml')
                return
            elif ('find' in message) & ('coffee' in message):
                print "\n==Time for a coffee break."
                os.system('rosnode kill obj_dregistered')
                rospy.sleep(2)
                subprocess.Popen(['rosrun', 'liam_neeson','obj_dregistered.py',self.package_path+'/src/classifier_data/coffee_sample.png',self.package_path+'/src/classifier_data/coffee_asus15.xml'])
                return
            elif ('find' in message) & ('fruit' in message):
                print "\n==Fiber is good for you."
                os.system('rosnode kill obj_dregistered')
                rospy.sleep(2)
                subprocess.Popen(['rosrun', 'liam_neeson','obj_dregistered.py',self.package_path+'/src/classifier_data/mapear_sample.png',self.package_path+'/src/classifier_data/mapear_asus15.xml'])
                return
            elif ('hello' in message):
                print "\n==Hi there!"
                os.system('rosnode kill baxter_display_left')
                return
            elif ('good' in message) & ('job' in message):
                os.system('rosnode kill obj_dregistered')
                rospy.sleep(2)
                os.system('rosnode kill baxter_cam_disp_left')
                print "\n==Thank you."
                return
            elif ('open' in message) & ('display' in message):
                print "\n==Opening left-hand camera."
                os.system('rosnode kill baxter_display_left')
                rospy.sleep(2)
                subprocess.Popen(['rosrun', 'liam_neeson','baxter_display_left.py'])
                # os.system('rosrun liam_neeson baxter_display_left.py')
                return
            else:
                print "\n==pocketsphinx_controller: Incomplete message: '%s'" %message

        else:
            print "\n==pocketsphinx_controller: Message not recognized: '%s' \n(Be sure to say 'Baxter' in command.)" %message


if __name__ == '__main__':
    # try:
        rospy.init_node('pocketsphinx_controller')
        
        print "==Initialized pocketsphinx_controller"

        controller = PocketSphinxController()
        print "==Ready for commands!"
        rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass