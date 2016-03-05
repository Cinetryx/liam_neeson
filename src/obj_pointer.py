#!/usr/bin/env python

import rospy
import baxter_interface
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped, PoseArray
import moveit_commander as mc
from scale_trajectory import scale_trajectory_speed as scale_traj
import math
import tf

# from math import pi, sqrt

class object:
    def __init__(self, pose):
        self.pose=pose
        self.x=self.pose.position.x
        self.y=self.pose.position.y
        self.z=self.pose.position.z

    def update(dx=0,dy=0,dz=0):
        self.x=self.x+dx
        self.y=self.y+dy
        self.z=self.z+dz
        self.pose=initPose(self.x,self.y,self.z)
        return self.pose

    def xyz():
        return (self.x,self.y,self.z)


# def initPose(px,py,pz,ox=[],oy=[],oz=[]):
#     Pose=PoseStamped()
#     Pose.header.stamp = rospy.Time.now()
#     Pose.pose.position.x = px
#     Pose.pose.position.y = py
#     Pose.pose.position.z = pz
#     if (ox!=[]):
#         quaternion = tf.transformations.quaternion_from_euler(ox, oy, oz)
#         Pose.pose.orientation.x = quaternion[0]
#         Pose.pose.orientation.y = quaternion[1]
#         Pose.pose.orientation.z = quaternion[2]
#         Pose.pose.orientation.w = quaternion[3]
#     return Pose

def initJointDict(jnames,jarray):
    # dict={}
    # for i in range(0,len(jarray)):
    #     dict[jnames[i]]=jarray[i]

    #more efficiently...
    return dict(zip(jnames,jarray))


# def get_curr_jts(jnames,bi_group):
#     #just reorders the baxter_interface dictionary values
#     bi_jts = bi_group.joint_angles() #dictionary of jt angles
#     curr_jts=[0]*len(bi_jts)
#     for i in range(0,len(bi_jts)):
#         curr_jts[i]=bi_jts[jnames[i]]
#     return curr_jts

def get_curr_pose(left_arm):
    #puts current pose in 6-element list format
    pose = left_arm.get_current_pose()
    (x,y,z) = pose.pose.position
    (r,p,y) = tf.transformations.euler_from_quaternion(pose.pose.orientation)
    pose = [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]+[r,p,y]
    return pose

def shift_curr_pose(left_arm,dpose=[0,0,0,0,0,0],x=0,y=0,z=0,ox=0,oy=0,oz=0):
    #takes 6-element list of xyzrpy difference amounts or individual values
    shift=[x,y,z,ox,oy,oz]+dpose
    pose=get_curr_pose(left_arm)+shift
    return pose

def calc_sphere(r,coord):
    #returns 6-element list of xyz position and rpy angles on spherical surface (of radius r) given xyz coord of object (ref coord is sphere origin).
    scale = r / math.sqrt(coord[0]**2 + coord[1]**2 + coord[2]**2)
    #this is essentially a problem of similar triangles. Just normalize the coord to r.
    s_coord = [x*scale for x in coord]
    #calculating roll pitch yaw angles
    s_angle = [0]*3
    s_angle[0] = math.atan2(coord[2],coord[1])
    s_angle[1] = math.atan2(coord[2],coord[0])
    s_angle[2] = math.atan2(coord[0],coord[1])
    #     s_angle = tf.transformations.quaternion_from_euler(s_angle[0], s_angle[1], s_angle[2])
    return (s_coord+s_angle)

# def calc_pose(xyz_rpy):
#     #takes in tuple of two lists: xyz coordinates and rpy angles
#     #returns pose described
#     xyz=xyz_rpy[0]
#     rpy=xyz_rpy[1]
#     return initPose(xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2])
#     pass
    
def initialize(left_arm):
    # neutral_pos = dict(zip(jts_left,[0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
    jts_left = left_arm.get_active_joints()
    initial_pos=[-0.08, -0.89, -0.31, 1.34, -1.98, 0.60, -0.79]
    # initial_pos = dict(zip(jts_left,[0,0,0,0,0,0,0]))
    left_arm.set_joint_value_target(initial_pos)
    print "Searching for plan..."
    plan = left_arm.plan(initial_pos)
    rospy.sleep(2)
    print "Executing plan!"
    # plan = scale_traj(plan, 0.5)
    left_arm.execute(plan,wait=True)    #Is blocking in order to ensure it reaches that position
    left_arm.clear_pose_targets()
    
def bi_initialize(left_arm):
    left_arm.move_to_neutral()
    jts_left = left_arm.joint_names()
    print jts_left
    initial_pos = dict(zip(jts_left,[-0.08, -0.89, -0.31, 1.34, -1.98, 0.60, -0.79]))
    # initial_pos = dict(zip(jts_left,[0,0,0,0,0,0,0]))
    left_arm.set_joint_positions(initial_pos)


def move (left_arm):
    print "============ Pos1"
    # left_arm.set_goal_position_tolerance(0.05)
    # left_arm.shift_pose_target(axis,value) #axis=0:5 for x,y,z,r,p,y
    # plan = scale_traj(plan, 0.5)
    pose=shift_curr_pose(left_arm,x=0.2)
    left_arm.set_pose_targets(pose)
    plan = left_arm.plan()
    left_arm.execute(plan,wait=False)
    left_arm.clear_pose_targets()

    pose=shift_curr_pose(left_arm,x=-0.2)
    left_arm.set_pose_targets(pose)
    plan = left_arm.plan()
    left_arm.execute(plan,wait=False)
    left_arm.clear_pose_targets()

    pose=shift_curr_pose(left_arm,y=-0.2)
    left_arm.set_pose_targets(pose)
    plan = left_arm.plan()
    left_arm.execute(plan,wait=False)
    left_arm.clear_pose_targets()

    pose=shift_curr_pose(left_arm,oz=-3.14/4)
    left_arm.set_pose_targets(pose)
    plan = left_arm.plan()
    left_arm.execute(plan,wait=False)
    left_arm.clear_pose_targets()

    pose=shift_curr_pose(left_arm,oz=3.14/2)
    left_arm.set_pose_targets(pose)
    plan = left_arm.plan()
    left_arm.execute(plan,wait=False)
    left_arm.clear_pose_targets()

    # rospy.sleep(5)
    # print left_arm.get_current_pose()
    
    # print "============ Pos2"
    # pose=initPose(0.9,0.0,0.0,3.14/2,3.14/2,0)
    # left_arm.set_pose_target(pose)
    # plan = left_arm.plan()
    # plan = scale_traj(plan, 0.5)
    # left_arm.execute(plan,wait=True)
    # # print left_arm.get_current_pose()
    # left_arm.clear_pose_targets()
    # rospy.sleep(1)
    # print "============ Pos3"
    # pose=initPose(0.7,0.0,0.0,3.14/2,3.14/2,0)
    # left_arm.set_pose_target(pose)
    # plan = left_arm.plan()
    # plan = scale_traj(plan, 0.5)
    # left_arm.execute(plan,wait=True)
    # # print left_arm.get_current_pose()
    # left_arm.clear_pose_targets()
    # print "============ Pos4"
    # pose=initPose(0.5,0.0,0.0,3.14/2,3.14/2,0)
    # left_arm.set_pose_target(pose)
    # plan = left_arm.plan()
    # plan = scale_traj(plan, 0.5)
    # left_arm.execute(plan,wait=True)
    # # print left_arm.get_current_pose()
    # left_arm.clear_pose_targets()

    # temp = rospy.wait_for_message("Dpos", PoseArray)



def callback(self,data):
    #update obj position
    pass


def main():
    rospy.init_node('PointAndPointSomeMore')
    print "============ Waiting for RVIZ..."
    # rospy.sleep(20)
    robot = mc.RobotCommander()
    scene = mc.PlanningSceneInterface()
    left_arm = mc.MoveGroupCommander("left_arm")
    bi_left_arm = baxter_interface.limb.Limb("left")
    # jts_left = left_arm.get_active_joints()
    bi_jts_left = bi_left_arm.joint_names()
    # print jts_left
    # print left_arm.get_current_joint_values()
    # print get_curr_jts(jts_left,bi_left_arm)
    # print left_arm.get_current_pose()
    # left_arm.set_goal_joint_tolerance(1)
    # left_arm.allow_replanning(True)
    # print left_arm.get_goal_tolerance()
    # print bi_left_arm.endpoint_pose()
    # jts_left = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    # bi_jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    
    print '\n=====OBJ_POINTER: Initializing Arm Position\n'
    initialize(left_arm)
    print '\n=====OBJ_POINTER: Arm Successfully Initialized\n'
    
    print left_arm.get_current_pose()
    print left_arm.get_current_rpy()
    print left_arm.get_current_joint_values()

    left_arm.set_pose_target([0.75,0.6,0.6,0,3.14/2,0]) #latitude and longitude approach using oy and oz solely
    plan = left_arm.plan()
    left_arm.execute(plan,wait=True)

    print left_arm.get_current_pose()
    print left_arm.get_current_rpy()
    print left_arm.get_current_joint_values()
    rospy.sleep(1)
    # left_arm.set_planning_time(0.1)
    print '\n=====Now Testing\n'
    move(left_arm)
    # print '\n=====OBJ_POINTER: Initializing Subscriber\n'
    # sub = rospy.Subscriber('/obj_position', Point, self.callback)
    # bi_initialize(bi_left_arm)

if __name__=='__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass