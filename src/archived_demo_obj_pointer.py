#!/usr/bin/env python

import rospy
import baxter_interface
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
import moveit_commander as mc
from scale_trajectory import scale_trajectory_speed as scale_traj
import math
import tf
from tf2_msgs.msg import TFMessage
import numpy as np
from sensor_msgs.msg import Range

# class RGBD_camera:
#     def __init__(self,trans):
#         #takes 6-element transformation list xyz and rpy and publishes latched static transform
#         self.pub_tf = rospy.Publisher("/tf_static", TFMessage , queue_size=1, latch=True)
#         self.trans=trans
#         rospy.sleep(1)
#     def set_trans(self,trans):
#         self.trans=trans
#     def pub_static_trans(self):
#         print "===== RGBDCamera: Publishing Static Transform"
#         quat = tf.transformations.quaternion_from_euler(self.trans[0],self.trans[1],self.trans[2])
#         msg = TransformStamped()
#         msg.header.stamp = rospy.Time.now()
#         msg.transform.rotation.x = quat[0]
#         msg.transform.rotation.y = quat[1]
#         msg.transform.rotation.z = quat[2]
#         msg.transform.rotation.w = quat[3] 
#         msg.header.frame_id = "base"   
#         msg.transform.translation.x = self.trans[0]
#         msg.transform.translation.y = self.trans[1]
#         msg.transform.translation.z = self.trans[2]
#         msg.child_frame_id = "camera_link"
#         self.pub_tf.publish([msg])
#         print "===== RGBDCamera: Publish Successful"

class IR_sensor:
    def __init__(self,side):
        self.side = side #string that is either "right" or "left", depending on which sensor you're using
        self.tt = tf.TransformListener()
        rospy.sleep(1)

    def get_coord(self):
        print "IR: Searching for Object"
        IRrange=self.get_IR()
        print IRrange
        return self.transIRtoBase(IRrange)
    def get_IR(self):
        IR=rospy.wait_for_message("/robot/range/"+self.side+"_hand_range/state", Range, timeout=None)
        while (IR.range>65 or IR.range<0.15):
            print "IR: Object out of range"
            IR=rospy.wait_for_message("/robot/range/"+self.side+"_hand_range/state", Range, timeout=None)
        return IR.range

    def transIRtoBase(self,IRrange):
        flag = 1
        
        for i in range(100):
            try:
                t=self.tt.getLatestCommonTime("/base","/"+self.side+"_hand_range")
                (trans,orien) = self.tt.lookupTransform("/base","/"+self.side+"_hand_range",t)  #orn is presumably quaternion
            except:
                rospy.sleep(0.1)
        matrix=tf.transformations.compose_matrix(angles=tf.transformations.euler_from_quaternion(orien), translate=trans)
        pt=tf.transformations.compose_matrix(translate=(IRrange,0,0))
        # numpy.array(((IRrange),(0),(0),(1)))
        ptbase=tf.transformations.decompose_matrix(np.dot(matrix,pt))
        euler= ptbase[2]
        translation = ptbase[3]
        # quaternion=tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
        return (euler,translation)

class Controller:
    def __init__(self,name):
        #by default, assuming camera faces straight forward from baxter
        self.tt = tf.TransformListener()
        rospy.sleep(1)
        self.robot = mc.RobotCommander()
        self.scene = mc.PlanningSceneInterface()
        self.arm = mc.MoveGroupCommander(name)
        self.camera_trans=[]
        self.get_camera_trans()
        self.sub=[]
    #     self.camera_coord=camera_coord
    #     self.transform=self.get_transform(z_angle)

    def get_camera_trans(self):
        transformer = tf.Transformer()
        # while not transformer.frameExists("camera_link1"):
        #     print "Waiting for camera_link frame"
        #     rospy.sleep(0.25)
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
        coord = [data.x,data.y,data.z]
        self.point(coord)
        pass

    # def get_transform(self,z_angle):
    #     matrix_trans=tf.transformations.compose_matrix(translate=self.camera_coord)
    #     matrix_z_angle=tf.transformations.compose_matrix(angles=[0,0,z_angle])
    #     matrix_z=tf.transformations.compose_matrix(angles=[-math.pi/2,0,0])
    #     matrix_xy=tf.transformations.compose_matrix(angles=[0,0,-math.pi/2])
    #     matrix_rot=numpy.dot(numpy.dot(matrix_z,matrix_xy),matrix_z_angle)
    #     cam_base=numpy.dot(matrix_rot,matrix_trans)
    #     return cam_base

    def get_obj_coord(self,xyz):
        pt=tf.transformations.compose_matrix(translate=(xyz[0],xyz[1],xyz[2]))
        obj_transform=np.dot(pt,self.camera_trans)
        pt_base=tf.transformations.decompose_matrix(obj_transform)
        translation = pt_base[3]
        # quaternion=tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
        # print translation
        return (translation)
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

    def calc_sphere(self,r,coord):
        #returns 6-element list of xyz position and rpy angles on spherical surface (of radius r) given xyz coord of object (ref coord is sphere origin).
        scale = r / math.sqrt((coord[0]**2) + (coord[1]**2) + (coord[2]**2))

        #this is essentially a problem of similar triangles. Just normalize the coord to r.
        s_coord = [x*scale for x in coord]
        #calculating rpy angles using pitch and yaw as latitude and longitude
        s_angle = [0]*3
        s_angle[1] = math.atan2(coord[0],coord[2])
        s_angle[2] = math.atan2(coord[1],coord[0])
        return (s_coord+s_angle)

    def point(self,xyz,damp=False,speed=1.5):
        for r in (0.85,1,0.7):
            base_xyz=self.get_obj_coord(xyz)
            print "Object Coord: ", base_xyz
            pose=self.calc_sphere(r,base_xyz)
            if self.move(pose,damp=damp,speed=speed):
                print '===== Moved'
                return

    def initialize(self):
        # neutral_pos = dict(zip(jts_left,[0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        # jts_left = self.arm.get_active_joints()
        # initial_pos=[-0.08, -0.89, -0.31, 1.34, -1.98, 0.60, -0.79]
        initial_pos=[-0.54, -0.67, 0.18, 1.18, -2.5, 0.65, 2.5]
        self.arm.set_joint_value_target(initial_pos)
        plan = self.arm.plan(initial_pos)
        if (self.arm.execute(plan,wait=True)):    #Is blocking in order to ensure it reaches that position
            print '===== Arm Initialized'

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

    def move(self,tar_pose,damp=False,speed=0):
        # left_arm.set_planning_time(0.25)
        print "Proposed Target Pose", tar_pose
        if damp:
            tar_pose=self.damp(tar_pose)
            # (tar_pose,fraction)=compute_cartesian_path(tar_pose, eef_step, jump_threshold, avoid_collisions = False)
        print "Current Pose", self.get_curr_pose()
        print "Damped Target Pose: ", tar_pose
        self.arm.set_pose_target(tar_pose)
        plan = self.arm.plan()
        if speed>0:
            plan=scale_traj(plan,speed)
        return self.arm.execute(plan,wait=False)

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
    left_arm.init_subscriber()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass