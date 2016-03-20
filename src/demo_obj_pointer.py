#!/usr/bin/env python

import rospy
import baxter_interface
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Point
import moveit_commander as mc
from scale_trajectory import scale_trajectory_speed as scale_traj
import math
import tf
from tf2_msgs.msg import TFMessage
import numpy as np
from sensor_msgs.msg import Range

class Controller:
    def __init__(self,name):
        #by default, assuming camera faces straight forward from baxter
        self.tt = tf.TransformListener()
        rospy.sleep(1)
        self.robot = mc.RobotCommander()
        self.scene = mc.PlanningSceneInterface()
        self.arm = mc.MoveGroupCommander(name)
        self.camera_trans = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]) #default value
        self.get_camera_trans()
        self.sub = []
        self.s0_offset = +0.75
        self.s0_ref = -self.s0_offset
        self.s1_ref = 0.0

    def get_camera_trans(self):
        transformer = tf.Transformer()
        for i in range(100):
            try:
                print "===== Searching for camera transform"
                t=self.tt.getLatestCommonTime("/base","/camera_link1")
                (trans,orien) = self.tt.lookupTransform("/base","/camera_link1",t)  #orn is presumably quaternion
                euler = tf.transformations.euler_from_quaternion(orien)
                self.camera_trans=tf.transformations.compose_matrix(angles=euler, translate=trans)
                print "===== Successfully obtained camera transform."
                print "===== The camera is located at: ",trans, euler, " ...in the robot frame.\n"
                break
            except:
                rospy.sleep(0.05)

    def init_subscriber(self):
        self.sub = rospy.Subscriber('/obj_position', Point, self.callback)

    def callback(self,data):
        #data is 3-element list of xyz coord in camera frame
        print "Here have a callback!"
        coord = [data.x,data.y,data.z]
        self.point(coord)
        pass

    def get_obj_coord(self,xyz):
        pt=tf.transformations.compose_matrix(translate=(xyz[0],xyz[1],xyz[2]))
        obj_transform=np.dot(self.camera_trans,pt)
        pt_base=tf.transformations.decompose_matrix(obj_transform)
        translation = pt_base[3]
        # quaternion=tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])

        return (translation)

    def update_ref_ang(self,coord):
        #returns latitude and longitude (centered at base) given xyz coord of object (ref coord is sphere origin).
        s0_ref = math.atan2(coord[1],coord[0]) - self.s0_offset
        s1_ref = -math.atan2(coord[2],coord[0])
        self.s0_ref = np.sign(s0_ref) * min(1.6,abs(s0_ref))
        self.s1_ref = np.sign(s1_ref) * min(1.0,abs(s1_ref))


    def move(self,tar_pose,speed=0):
        # left_arm.set_planning_time(0.25)
        print "Proposed Target Joints: ", tar_pose
        print "Current Joints: ", self.arm.get_current_joint_values()
        self.arm.set_joint_value_target(tar_pose)
        plan = self.arm.plan(tar_pose)
        if speed>0:
            plan=scale_traj(plan,speed)
        return self.arm.execute(plan,wait=False)

    def point(self,xyz,speed=0):
        base_xyz=self.get_obj_coord(xyz)
        print "Object in Robot Frame: ", base_xyz
        self.update_ref_ang(base_xyz)
        if self.move([self.s0_ref,self.s1_ref,0,0,0,0,0],speed=speed):
            print '===== Moved'

    def initialize(self):
        # neutral_pos = dict(zip(jts_left,[0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        # jts_left = self.arm.get_active_joints()
        # initial_pos=[-0.08, -0.89, -0.31, 1.34, -1.98, 0.60, -0.79]
        # initial_pos=[-0.54, -0.67, 0.18, 1.18, -2.5, 0.65, 2.5]
        initial_pos=[-self.s0_offset,0,0,0,0,0,0]
        self.arm.set_joint_value_target(initial_pos)
        plan = self.arm.plan(initial_pos)
        if (self.arm.execute(plan,wait=True)):    #Is blocking in order to ensure it reaches that position
            print '===== Arm Initialized'


    def get_curr_pose(self):
        #puts current pose in 6-element list format
        pose = self.arm.get_current_pose()
        xyz = [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]
        (r,p,y) = tf.transformations.euler_from_quaternion([pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w])
        pose = xyz+[r,p,y]
        return pose

    def shift_curr_pose(self,dpose=[0,0,0,0,0,0],x=0,y=0,z=0,ox=0,oy=0,oz=0):
        #takes 6-element list of xyzrpy difference amounts or individual values
        shift = [x,y,z,ox,oy,oz]
        pose = self.get_curr_pose()
        pose = [sum(x) for x in zip(dpose, shift, pose)]
        return pose

    def damp(self,tar_pose):
        pos_thresh=0.1      #max safe increment values
        ang_thresh=3.14/6
        curr_pose = np.asarray(self.get_curr_pose())
        curr_pose = curr_pose.astype(float)
        new_pose = np.asarray(tar_pose)
        new_pose = new_pose.astype(float)
        max_pos_diff= max(abs(new_pose[:3]-curr_pose[:3]))
        max_ang_diff= max(abs(new_pose[3:]-curr_pose[3:]))
        if (max_pos_diff>pos_thresh):
            new_pose[:3]=((new_pose[:3]-curr_pose[:3])*(pos_thresh/max_pos_diff))+curr_pose[:3]
        if (max_ang_diff>ang_thresh):
            new_pose[3:]=((new_pose[3:]-curr_pose[3:])*(ang_thresh/max_ang_diff))+curr_pose[3:]
        return new_pose.tolist()

    def test(self):
        self.initialize()
        pose=self.shift_curr_pose(x=-0.1)
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        self.arm.execute(plan,wait=False)
        pose=self.shift_curr_pose(x=0.1)
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        self.arm.execute(plan,wait=False)
        pose=self.shift_curr_pose(oy=-0.3)
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        self.arm.execute(plan,wait=False)
        pose=self.shift_curr_pose(oy=0.3)
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        self.arm.execute(plan,wait=False)

def main():
    rospy.init_node('point_and_point_some_more')
    # IR = IR_sensor("right")
    # camera_coord = IR.get_coord()
    # camera=RGBD_camera([0.9,0,0,0,0,0])
    # camera.pub_static_trans()
    left_arm=Controller("left_arm")
    # print left_arm.calc_sphere(1,[1,0,0])

    print '\n===== Initializing Arm Position'
    left_arm.initialize()
    print '===== Initialization Complete\n'

    print '\n=====OBJ_POINTER: Initializing Subscriber\n'
    # left_arm.init_subscriber()
    
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        coord=rospy.wait_for_message("/obj_position", Point, timeout=None)
        coord=[coord.x,coord.y,coord.z]
        print "Object in Camera Frame: ",coord
        left_arm.point(coord)
        rate.sleep()
    # left_arm.point([1,0,1])


if __name__=='__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass