# -*- coding: utf-8 -*-
import cv2
import sys
import time
import rospy
import numpy as np
import threading
import math
import tf
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
import actionlib
from geometry_msgs.msg import Twist,Quaternion,PoseWithCovarianceStamped,PoseArray,Pose
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import types
import pprint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # for 3D graph
#import pylab import *

class OpencvClass():
    #MAP_PATH = "~/maps/mymap.pgm" # Reject absolute path.
    MAP_PATH = "/home/okano/maps/mymap.pgm"
    def __init__(self):
        # Grid create --->
        self.X_GRID = 384
        self.Y_GRID = 832

        self.x_grid = np.linspace(0, self.X_GRID, self.X_GRID+1) # 385
        self.y_grid = np.linspace(0, self.Y_GRID, self.Y_GRID+1) # 833

        self.xx, self.yy = np.meshgrid(self.x_grid, self.y_grid) # 格子点となるx,y座標を作成。  
   
        # Image --->
        self.img = cv2.imread(self.MAP_PATH, cv2.IMREAD_GRAYSCALE) # cv2.IMREAD_COLOR
        self.img = np.insert(self.img, 384, 1, axis=1)
        self.img = np.insert(self.img, 832, 1, axis=0)

        # Display --->
        #self.plot1(self.xx, self.yy, self.img)
        #self.displayImage()

    def plot1(self, x, y, ret):
        plt.gca().set_aspect('equal', adjustable='box') # グラフの縦横の比をそろえるコマンド   
        plt.contourf(x, y, ret>0, cmap=plt.cm.bone) # 条件(ret>0)を満たす部分とそうでない部分で色を変える
        plt.show()


    def getVirtualLidarValue(self, _x, _y, _r):
        ''' 
            Return lidar value.
            obj : (1, 360)
        ''' 
        # TODO fix abs
        x = _x * math.cos(_r)
        y = _y * math.sin(_r)
        lidar = np.zeros(360 - 1)
        for deg in range(0,360 - 1):
            lidar[deg] = 0
        return lidar 


    def displayImage(self):
        print "displayImage"
        cv2.imshow("opencv_image",self.img)
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        # TODO be careful
        sys.exit()

        #self.thread.start()


class ParticleFilter():
    RECORD_PATH = ""
    PARTICLE_NUM = 1000
    def __init__(self):     
        '''
            This function do that.
            - particle initialization
            - opencv activate
            - robot init position setting
        '''
        # param init
        self.time = time.time()
        self.v_x = 0
        self.v_y = 0
        self.move_x = 0
        self.move_y = 0
        self.rotate_z = 0
        self.theta = 0
        self.mymap = []

        # ros init
        self.particle_pub = rospy.Publisher('/particle_info',PoseArray,queue_size=1)
        self.predict_marker_pub = rospy.Publisher('/predict_pose',Marker,queue_size=1)
        self.gazebo_marker_pub = rospy.Publisher('/gazebo_pose',Marker,queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu,self.imuCB)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laserCB)
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid,self.mapCB)
        self.robot_pose_sub = rospy.Subscriber('/gazebo/model_states',ModelStates, self.visualizeSimulatedRobotPoseCB)

        # other init
        self.initParticle()
        self.opencv = OpencvClass()


    # ----------------------------------------------------
    def imuCB(self, _receive):
        self.imu_data = _receive

        # data setting
        time_diff = time.time() - self.time
        self.time = time.time()

        t =  time.time() - self.time
        self.time = time.time()
        #ori = self.imu_data.orientation
        acc = self.imu_data.linear_acceleration

        # calc
        self.v_x += acc.x * t
        self.v_y += acc.y * t

        self.move_x += self.v_x * t
        self.move_y += self.v_y * t
        self.rotate_z += self.imu_data.angular_velocity.z * t
        #print self.move_x
        #print self.move_y
        self.angular_velocity = self.imu_data.angular_velocity


    def laserCB(self, _receive):
        self.lidar_dist = np.array(_receive.ranges)
        #print self.lidar_dist.shape
        #print self.lidar_dist


    def mapCB(self,msg):
        PIXEL_UNIT = 20 # 1m = 20pix
        width = int(msg.info.width)
        height = int(msg.info.height)
        mymap = np.array(msg.data)
        mymap = np.reshape(mymap,(width,height))
        #print msg.info.origin


    # ----------------------------------------------------
    def initParticle(self):
        self.particles = np.random.randn(self.PARTICLE_NUM,2)
        alpha = math.pi/2
        self.particles = np.insert(self.particles, 2, alpha, axis=1) # Add rotate value.
        #print self.particles


    def sampling(self):
        # particle updata
        self.theta = self.rotate_z
        print self.theta
        for i in range(0,len(self.particles)):
            self.particles[i,0] += \
                    self.move_x * math.sin(90 - self.theta)+ \
                    self.move_y * math.sin(self.theta)     + \
                    np.random.randn()
            self.particles[i,1] += \
                    self.move_x * math.cos(90 - self.theta) + \
                    self.move_y * math.cos(self.theta)      + \
                    np.random.randn()
            self.particles[i,2] += self.rotate_z


    def weightCalc(self):
        '''
            compare lidar_dist to particles (0~359 deg)
        '''
        particle_lidar_value = []

        # get particles weight 
        for i in range(0,len(self.particles)):
            x = self.particles[i,0]
            y = self.particles[i,1]
            r = self.particles[i,2]
            np_list = self.opencv.getVirtualLidarValue(x, y, r)
            particle_lidar_value.append(np_list)

        #print self.lidar_dist

    def predict(self): 
        '''
            This func do,
            - sampling
            - weight calc
            - re-sampling
        '''
        print "predict now"
        self.sampling()
        self.weightCalc()

        self.visualizeParticle(self.particles)
        self.visualizeLocalizedRobotPose(self.particles)

        self.recordPredictDifference()


    def recordPredictDifference(self):
        '''
            It record difference between predict to real.
        '''
        pass


    # ----------------------------------------------------
    def visualizeParticle(self ,particles):
        particle_msg = PoseArray()
        particle_msg.header.frame_id = "map"
        particle_msg.header.stamp = rospy.Time.now()
        for i in range(0,len(particles)):
            pose = Pose()
            pose.position.x = particles[i,0]
            pose.position.y = particles[i,1]
            pose.position.z = 0
            alpha = particles[i,2]
            q = tf.transformations.quaternion_from_euler(0,0,alpha)

            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            particle_msg.poses.append(pose)
        self.particle_pub.publish(particle_msg)


    def visualizeLocalizedRobotPose(self, particles):
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "localized_robot_pose"
        marker_data.id = 1
        marker_data.action = Marker.ADD

        # x,y sum
        x,y,pitch = particles.mean(axis=0)

        # setting
        q = tf.transformations.quaternion_from_euler(0,0,pitch)
        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0

        marker_data.pose.orientation.x = q[0]
        marker_data.pose.orientation.y = q[1]
        marker_data.pose.orientation.z = q[2]
        marker_data.pose.orientation.w = q[3]

        marker_data.color.r = 0.0
        marker_data.color.g = 1.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.3
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0
        self.predict_marker_pub.publish(marker_data)


    def visualizeSimulatedRobotPoseCB(self, model_state):
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "simulated_robot_pose"
        marker_data.id = 1
        marker_data.action = Marker.ADD
        marker_data.pose.position.x = model_state.pose[2].position.x
        marker_data.pose.position.y = model_state.pose[2].position.y
        marker_data.pose.position.z = 0
        marker_data.pose.orientation.x = model_state.pose[2].orientation.x
        marker_data.pose.orientation.y = model_state.pose[2].orientation.y
        marker_data.pose.orientation.z = model_state.pose[2].orientation.z
        marker_data.pose.orientation.w = model_state.pose[2].orientation.w
        marker_data.color.r = 0.0
        marker_data.color.g = 0.0
        marker_data.color.b = 1.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.3
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1
        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0
        self.gazebo_marker_pub.publish(marker_data)


if __name__ == '__main__':
    rospy.init_node('particle')

    p = ParticleFilter()
    time.sleep(1)

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        p.predict()
        rate.sleep()
