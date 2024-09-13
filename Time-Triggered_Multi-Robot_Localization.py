#!/usr/bin/env	python
from __future__ import division
import	rospy
from sensor_msgs.msg import Range
import message_filters
from math import *
from nav_msgs.msg import Odometry
from numpy import *
from random import *
from robot import *
from ekf_functions import *
from matplotlib.pyplot import *
from geometry_msgs.msg import Point, Quaternion
import tf
import time
import geometry_msgs.msg
import ar_track_alvar_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers 
import random


global dist_sens_min_range, dist_sens_max_range, proximity_min_range, proximity_max_range,MOT_STEP_DIST,WHEEL_CIRCUMFERENCE,WHEEL_DISTANCE,WHEEL_SEPARATION,WHEEL_DIAMETER,  ROBOT_RADIUS


## e-puck2 dimensions
# Wheel Radio (cm)
WHEEL_DIAMETER = 4
# Separation between wheels (cm)
WHEEL_SEPARATION = 5.3

# Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
WHEEL_DISTANCE = 0.053
# Wheel circumference (meters).
WHEEL_CIRCUMFERENCE = ((WHEEL_DIAMETER*math.pi)/100.0)
# Distance for each motor step (meters); a complete turn is 1000 steps.
MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/1000.0)    # 0.000125 meters per step (m/steps)


class Range_Bearing():
    def __init__(self,deltaTheta_epuck0=0,deltaTheta_epuck1=0,deltaSteps_epuck0=0,deltaSteps_epuck1=0,deltaTheta_epuck2=0,deltaTheta_epuck3=0,deltaSteps_epuck2=0,deltaSteps_epuck3=0, filtered_epuck0=matrix([0.372, -0.147 ,0]),filtered_epuck1=matrix([0.305, -0.1624 ,0]), filtered_epuck2=matrix([0.2682, -0.025,0]),filtered_epuck3=matrix([0.340, -0.0235 ,0]), covariance_epuck0=np.array([[0.0000000001, 0, 0],[0, 0.0000000001, 0],[0, 0, 0.0000000001]]), covariance_epuck1=np.array([[0.0000000001, 0, 0],[0, 0.0000000001, 0],[0, 0, 0.0000000001]]),covariance_epuck2=np.array([[0.0000000001, 0, 0],[0, 0.0000000001, 0],[0, 0, 0.0000000001]]), covariance_epuck3=np.array([[0.0000000001, 0, 0],[0, 0.0000000001, 0],[0, 0, 0.0000000001]]), init_xpos_epuck0=0.372, init_ypos_epuck0=-0.147, init_theta_epuck0=0,init_xpos_epuck1=0.305, init_ypos_epuck1=-0.1624, init_theta_epuck1=0, init_xpos_epuck2=0.2682, init_ypos_epuck2=-0.025, init_theta_epuck2=0,init_xpos_epuck3=0.340, init_ypos_epuck3=-0.0235, init_theta_epuck3=0, yaw_epuck0=0, yaw_epuck1=0, yaw_epuck2=0, yaw_epuck3=0, camera_epuck0=matrix([0.378, -0.1293 ,0]), camera_epuck1=matrix([0.311, -0.145 ,0]), camera_epuck2=matrix([0.268, -0.010,0]), camera_epuck3=matrix([0.3400, -0.006 ,0]), flag_epuck0=0, flag_epuck1=0, flag_epuck2=0, flag_epuck3=0, Z_epuck0=0,Z_epuck1=0, Z_epuck2=0,Z_epuck3=0):
          rospy.init_node('range_bearing', anonymous=True)
          self.theta_epuck0 = init_theta_epuck0
          self.x_pos_epuck0 = init_xpos_epuck0
          self.y_pos_epuck0 = init_ypos_epuck0
          self.theta_epuck1 = init_theta_epuck1
          self.x_pos_epuck1 = init_xpos_epuck1
          self.y_pos_epuck1 = init_ypos_epuck1
          self.theta_epuck2 = init_theta_epuck2
          self.x_pos_epuck2 = init_xpos_epuck2
          self.y_pos_epuck2 = init_ypos_epuck2
          self.theta_epuck3 = init_theta_epuck3
          self.x_pos_epuck3 = init_xpos_epuck3
          self.y_pos_epuck3 = init_ypos_epuck3

          self.Z_epuck0=Z_epuck0
          self.Z_epuck1=Z_epuck1
          self.Z_epuck2=Z_epuck2
          self.Z_epuck3=Z_epuck3

          self.leftStepsPrev_epuck0 = 0
          self.rightStepsPrev_epuck0 = 0
          self.leftStepsPrev_epuck1 = 0
          self.rightStepsPrev_epuck1 = 0
          self.leftStepsPrev_epuck2 = 0
          self.rightStepsPrev_epuck2 = 0
          self.leftStepsPrev_epuck3 = 0
          self.rightStepsPrev_epuck3 = 0

          self.yaw_epuck0=yaw_epuck0
          self.yaw_epuck1=yaw_epuck1
          self.yaw_epuck2=yaw_epuck2
          self.yaw_epuck3=yaw_epuck3

          self.deltaTheta_epuck1=deltaTheta_epuck1
          self.deltaTheta_epuck0=deltaTheta_epuck0
          self.deltaSteps_epuck0=deltaSteps_epuck0
          self.deltaSteps_epuck1=deltaSteps_epuck1

          self.deltaTheta_epuck2=deltaTheta_epuck2
          self.deltaTheta_epuck3=deltaTheta_epuck3
          self.deltaSteps_epuck2=deltaSteps_epuck2
          self.deltaSteps_epuck3=deltaSteps_epuck3

          self.covariance_epuck0=covariance_epuck0
          self.covariance_epuck1=covariance_epuck1
          self.covariance_epuck2=covariance_epuck2
          self.covariance_epuck3=covariance_epuck3

          self.filtered_epuck0=filtered_epuck0
          self.filtered_epuck1=filtered_epuck1
          self.filtered_epuck2=filtered_epuck2
          self.filtered_epuck3=filtered_epuck3

          self.camera_epuck0=camera_epuck0
          self.camera_epuck1=camera_epuck1
          self.camera_epuck2=camera_epuck2
          self.camera_epuck3=camera_epuck3

          self.total_distance_epuck0=0
          self.total_distance_epuck1=0
          self.total_distance_epuck2=0
          self.total_distance_epuck3=0

          self.previous_x_epuck0 = 0
          self.previous_y_epuck0 = 0
          self.previous_x_epuck1 = 0
          self.previous_y_epuck1 = 0
          self.previous_x_epuck2 = 0
          self.previous_y_epuck2 = 0
          self.previous_x_epuck3 = 0
          self.previous_y_epuck3 = 0

          self.flag_epuck0=flag_epuck0
          self.flag_epuck1=flag_epuck1
          self.flag_epuck2=flag_epuck2
          self.flag_epuck3=flag_epuck3

          self.first_run_epuck0 = True
          self.second_run_epuck0 = True
          self.second_run_epuck1 = True
          self.first_run_epuck1 = True
          self.first_run_epuck2 = True
          self.second_run_epuck2 = True
          self.second_run_epuck3 = True
          self.first_run_epuck3 = True

          self.Gamma_range = 0.05
          self.Delta_range = 0.05
          self.Gamma_bearing = 0.01
          self.Delta_bearing = 0.01
          # ROBOT AND LOCALIZATION 
          self.ekf2w=None
	  self.movingRobot=None
	  self.stationaryRobot=None
	  self.display_toggle=1		# Displays/closes graphs

	  self.ut_epuck0=[0,0]
          self.ut_epuck1=[0,0]
	  self.ut_epuck2=[0,0]
          self.ut_epuck3=[0,0]

          self.Odometry_epuck0 = rospy.Subscriber('/epuck2_robot_0/odom',Odometry,self.callback_epuck0)
          self.Odometry_epuck1 = rospy.Subscriber('/epuck2_robot_1/odom',Odometry,self.callback_epuck1)
          self.Odometry_epuck2 = rospy.Subscriber('/epuck2_robot_2/odom',Odometry,self.callback_epuck2)
          self.Odometry_epuck3 = rospy.Subscriber('/epuck2_robot_3/odom',Odometry,self.callback_epuck3)
          self.EKF_epuck0= rospy.Publisher('/epuck2_robot_0/robot_pose_ekf/odom_combined', Odometry, queue_size=10)
          self.EKF_epuck1= rospy.Publisher('/epuck2_robot_1/robot_pose_ekf/odom_combined', Odometry, queue_size=10)
          self.EKF_epuck2= rospy.Publisher('/epuck2_robot_2/robot_pose_ekf/odom_combined', Odometry, queue_size=10)
          self.EKF_epuck3= rospy.Publisher('/epuck2_robot_3/robot_pose_ekf/odom_combined', Odometry, queue_size=10)

          self.EKF0= rospy.Publisher('/epuck2_robot_0/robot_pose_ekf', Odometry, queue_size=10)
          self.EKF1= rospy.Publisher('/epuck2_robot_1/robot_pose_ekf', Odometry, queue_size=10)
          self.EKF2= rospy.Publisher('/epuck2_robot_2/robot_pose_ekf', Odometry, queue_size=10)
          self.EKF3= rospy.Publisher('/epuck2_robot_3/robot_pose_ekf', Odometry, queue_size=10)

          self.EKF_epuck0= rospy.Publisher('/epuck2_robot_0/robot_pose_ekf/odom_combined', Odometry, queue_size=10)
          self.EKF_epuck1= rospy.Publisher('/epuck2_robot_1/robot_pose_ekf/odom_combined', Odometry, queue_size=10)
          self.EKF_epuck2= rospy.Publisher('/epuck2_robot_2/robot_pose_ekf/odom_combined', Odometry, queue_size=10)
          self.EKF_epuck3= rospy.Publisher('/epuck2_robot_3/robot_pose_ekf/odom_combined', Odometry, queue_size=10)

          self.camera_aurco_epuck0= rospy.Publisher('/epuck2_robot_0/ar_track_alvar', Odometry, queue_size=10)
          self.camera_aurco_epuck1= rospy.Publisher('/epuck2_robot_1/ar_track_alvar', Odometry, queue_size=10)
          self.camera_aurco_epuck2= rospy.Publisher('/epuck2_robot_2/ar_track_alvar', Odometry, queue_size=10)
          self.camera_aurco_epuck3= rospy.Publisher('/epuck2_robot_3/ar_track_alvar', Odometry, queue_size=10)
          self.camera_epuck0_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck0,queue_size=100)
          self.camera_epuck1_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck1,queue_size=100)
          self.camera_epuck2_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck2,queue_size=100)
          self.camera_epuck3_sub=rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers,self.Camera_epuck3,queue_size=100)
          self.Filtered_epuck0_sub=rospy.Subscriber('/epuck2_robot_0/robot_pose_ekf/odom_combined',Odometry,self.Filtered_epuck0)
          self.Filtered_epuck1_sub=rospy.Subscriber('/epuck2_robot_1/robot_pose_ekf/odom_combined',Odometry,self.Filtered_epuck1)
          self.Filtered_epuck2_sub=rospy.Subscriber('/epuck2_robot_2/robot_pose_ekf/odom_combined',Odometry,self.Filtered_epuck2)
          self.Filtered_epuck3_sub=rospy.Subscriber('/epuck2_robot_3/robot_pose_ekf/odom_combined',Odometry,self.Filtered_epuck3)



#########################################################################################
         
	  #	Initialize robots if Localization command is received

	  #	Initialize positions of the robots
			#	Robot0 is always starts at  (x,y,z)=(0,0,0)
			#	Robot1 and the rest start at (x,y,z)=(Distance_from_kinect,0,0)
          self.ekf2w=None
	  initialPoses=[[0.372, -0.147 ,0],[0.305, -0.1624 ,0],[0.2682, -0.025,0],[0.340, -0.0235 ,0]]
          numRobots=4
          #List of Robots
          self.Robots=[]
	  #Number of Steps
	  self.stepNumber=0

	  for robotID in range(numRobots):
			self.Robots.append(Robot(robotID,initialPoses[robotID]))
          


	  print "*************************** LOCALIZATION STARTED *****************************"

    def   Localization_epuck0(self, ut_epuck0, x_pos_epuck0, y_pos_epuck0, theta_epuck0,flag_epuck0):
                                self.moving_robot=0
                                mylist = [1,2,3]
                                self.stationary_robot=random.choice(mylist)
                                camera_epuck=[self.camera_epuck0, self.camera_epuck1,self.camera_epuck2,self.camera_epuck3]                                
                                range_cam=self.camera_epuck0-camera_epuck[self.stationary_robot]
                                range_pow=pow(abs(range_cam[0,0]),2) + pow(abs(range_cam[0,1]),2)
                                Z_range_epuck0=sqrt(range_pow) 
                                yaw_epuck=[self.yaw_epuck0, self.yaw_epuck1,self.yaw_epuck2,self.yaw_epuck3]                                                            
                                print self.yaw_epuck0*(180/pi)
                                print yaw_epuck[self.stationary_robot]*(180/pi)                               
                                Z_bearing_epuck0=-self.yaw_epuck0-self.yaw_epuck0+yaw_epuck[self.stationary_robot]
                                Z_bearing_epuck0_norm=normalizeAngle(Z_bearing_epuck0)
                                
                                self.Z_epuck0=[Z_range_epuck0,Z_bearing_epuck0_norm]
                              
                                if (self.flag_epuck0!= 0):
                                      self.ekf2w_localization_epuck0(self.moving_robot,self.stationary_robot,self.ut_epuck0,self.Z_epuck0, 
                                      self.deltaTheta_epuck0,self.deltaSteps_epuck0,self.x_pos_epuck0,self.y_pos_epuck0,
                                      self.theta_epuck0,self.camera_epuck0,self.flag_epuck0)
                                self.movingRobot=None
                                self.stationaryRobot=None
	                        self.ut=[0,0]



    def   Localization_epuck1(self, ut_epuck1, x_pos_epuck1, y_pos_epuck1, theta_epuck1,flag_epuck1):
                                self.moving_robot=1
                                mylist = [0,2,3]
                                self.stationary_robot=random.choice(mylist)
                                camera_epuck=[self.camera_epuck0,self.camera_epuck1,self.camera_epuck2,self.camera_epuck3]                                
                                range_cam=self.camera_epuck1-camera_epuck[self.stationary_robot]
                                range_pow=pow(abs(range_cam[0,0]),2) + pow(abs(range_cam[0,1]),2)
                                Z_range_epuck1=sqrt(range_pow) 
                                yaw_epuck=[self.yaw_epuck0,self.yaw_epuck1,self.yaw_epuck2,self.yaw_epuck3]                                                            
                                print self.yaw_epuck1*(180/pi)
                                print yaw_epuck[self.stationary_robot]*(180/pi)                               
                                Z_bearing_epuck1=-self.yaw_epuck1-self.yaw_epuck1+yaw_epuck[self.stationary_robot]
                                Z_bearing_epuck1_norm=normalizeAngle(Z_bearing_epuck1)
                                
                                self.Z_epuck1=[Z_range_epuck1,Z_bearing_epuck1_norm]
                               
                                if (self.flag_epuck1!= 0):
	                                self.ekf2w_localization_epuck1(self.moving_robot,self.stationary_robot,self.ut_epuck1,self.Z_epuck1, 
                                        self.deltaTheta_epuck1,self.deltaSteps_epuck1,self.x_pos_epuck1,self.y_pos_epuck1,self.theta_epuck1,self.camera_epuck1,self.flag_epuck1)
                                self.movingRobot=None
	                        self.stationaryRobot=None
	                        self.ut=[0,0]


    def   Localization_epuck2(self, ut_epuck2, x_pos_epuck2, y_pos_epuck2, theta_epuck2,flag_epuck2):
          
                                self.moving_robot=2
                                mylist = [0,1,3]
                                self.stationary_robot=random.choice(mylist)
                                camera_epuck=[self.camera_epuck0,self.camera_epuck1,self.camera_epuck2, self.camera_epuck3]                                
                                range_cam=self.camera_epuck2-camera_epuck[self.stationary_robot]
                                range_pow=pow(abs(range_cam[0,0]),2) + pow(abs(range_cam[0,1]),2)
                                Z_range_epuck2=sqrt(range_pow) 
                                yaw_epuck=[self.yaw_epuck0,self.yaw_epuck1,self.yaw_epuck2,self.yaw_epuck3]                                                            
                                print self.yaw_epuck2*(180/pi)
                                print yaw_epuck[self.stationary_robot]*(180/pi)                               
                                Z_bearing_epuck2=-self.yaw_epuck2-self.yaw_epuck2+yaw_epuck[self.stationary_robot]
                                Z_bearing_epuck2_norm=normalizeAngle(Z_bearing_epuck2)
                                
                                self.Z_epuck2=[Z_range_epuck2,Z_bearing_epuck2_norm]
                               
                                if (self.flag_epuck2!= 0):
                                      self.ekf2w_localization_epuck2(self.moving_robot,self.stationary_robot,self.ut_epuck2,self.Z_epuck2,
                                      self.deltaTheta_epuck2,self.deltaSteps_epuck2,self.x_pos_epuck2,self.y_pos_epuck2,
                                      self.theta_epuck2,self.camera_epuck2,self.flag_epuck2)
                                self.movingRobot=None
                                self.stationaryRobot=None
	                        self.ut=[0,0]

    def   Localization_epuck3(self, ut_epuck3, x_pos_epuck3, y_pos_epuck3, theta_epuck3,flag_epuck3):
                                                             
                                self.moving_robot=3
                                mylist = [0,1,2]
                                self.stationary_robot=random.choice(mylist)
                                camera_epuck=[self.camera_epuck0,self.camera_epuck1,self.camera_epuck2,self.camera_epuck3]                                
                                range_cam=self.camera_epuck3-camera_epuck[self.stationary_robot]
                                range_pow=pow(abs(range_cam[0,0]),2) + pow(abs(range_cam[0,1]),2)
                                Z_range_epuck3=sqrt(range_pow) 
                                yaw_epuck=[self.yaw_epuck0,self.yaw_epuck1,self.yaw_epuck2,self.yaw_epuck3]                                                            
                                print self.yaw_epuck3*(180/pi)
                                print yaw_epuck[self.stationary_robot]*(180/pi)                               
                                Z_bearing_epuck3=-self.yaw_epuck3-self.yaw_epuck3+yaw_epuck[self.stationary_robot]
                                Z_bearing_epuck3_norm=normalizeAngle(Z_bearing_epuck3)
                               
                                self.Z_epuck3=[Z_range_epuck3,Z_bearing_epuck3_norm]
                               
                                if (self.flag_epuck3!= 0):
                                      self.ekf2w_localization_epuck3(self.moving_robot,self.stationary_robot,self.ut_epuck3,self.Z_epuck3, 
                                      self.deltaTheta_epuck3,self.deltaSteps_epuck3,self.x_pos_epuck3,self.y_pos_epuck3,self.theta_epuck3,
                                      self.camera_epuck3,self.flag_epuck3)
                                self.movingRobot=None
                                self.stationaryRobot=None
	                        self.ut=[0,0]

         

    def  Filtered_epuck0(self,msg):
               self.filtered_x_epuck0 = msg.pose.pose.position.x
               self.filtered_y_epuck0 = msg.pose.pose.position.y
               self.filtered_theta_epuck0 = msg.pose.pose.orientation.z
               self.filtered_epuck0=matrix([self.filtered_x_epuck0,self.filtered_y_epuck0,self.filtered_theta_epuck0])
               self.covariance_epuck0_0=msg.pose.covariance[0]
               self.covariance_epuck0_1=msg.pose.covariance[1]
               self.covariance_epuck0_2=msg.pose.covariance[2]
               self.covariance_epuck0_3=msg.pose.covariance[3]
               self.covariance_epuck0_4=msg.pose.covariance[4]
               self.covariance_epuck0_5=msg.pose.covariance[5]
               self.covariance_epuck0_6=msg.pose.covariance[6]
               self.covariance_epuck0_7=msg.pose.covariance[7]
               self.covariance_epuck0_8=msg.pose.covariance[8]
               self.covariance_epuck0=np.array([[self.covariance_epuck0_0, self.covariance_epuck0_1, self.covariance_epuck0_2],[self.covariance_epuck0_3, self.covariance_epuck0_4, self.covariance_epuck0_5],[self.covariance_epuck0_6, self.covariance_epuck0_7, self.covariance_epuck0_8]])


                      


    def  Filtered_epuck1(self,msg):
           self.filtered_x_epuck1 = msg.pose.pose.position.x
           self.filtered_y_epuck1 = msg.pose.pose.position.y
           self.filtered_theta_epuck1 = msg.pose.pose.orientation.z
           self.filtered_epuck1=matrix([self.filtered_x_epuck1,self.filtered_y_epuck1,self.filtered_theta_epuck1])
           self.covariance_epuck1_0=msg.pose.covariance[0]
           self.covariance_epuck1_1=msg.pose.covariance[1]
           self.covariance_epuck1_2=msg.pose.covariance[2]
           self.covariance_epuck1_3=msg.pose.covariance[3]
           self.covariance_epuck1_4=msg.pose.covariance[4]
           self.covariance_epuck1_5=msg.pose.covariance[5]
           self.covariance_epuck1_6=msg.pose.covariance[6]
           self.covariance_epuck1_7=msg.pose.covariance[7]
           self.covariance_epuck1_8=msg.pose.covariance[8]
           self.covariance_epuck1=np.array([[self.covariance_epuck1_0, self.covariance_epuck1_1, self.covariance_epuck1_2],[self.covariance_epuck1_3, self.covariance_epuck1_4, self.covariance_epuck1_5],[self.covariance_epuck1_6, self.covariance_epuck1_7, self.covariance_epuck1_8]])


    def  Filtered_epuck2(self,msg):
               self.filtered_x_epuck2 = msg.pose.pose.position.x
               self.filtered_y_epuck2 = msg.pose.pose.position.y
               self.filtered_theta_epuck2 = msg.pose.pose.orientation.z
               self.filtered_epuck2=matrix([self.filtered_x_epuck2,self.filtered_y_epuck2,self.filtered_theta_epuck2])
               self.covariance_epuck2_0=msg.pose.covariance[0]
               self.covariance_epuck2_1=msg.pose.covariance[1]
               self.covariance_epuck2_2=msg.pose.covariance[2]
               self.covariance_epuck2_3=msg.pose.covariance[3]
               self.covariance_epuck2_4=msg.pose.covariance[4]
               self.covariance_epuck2_5=msg.pose.covariance[5]
               self.covariance_epuck2_6=msg.pose.covariance[6]
               self.covariance_epuck2_7=msg.pose.covariance[7]
               self.covariance_epuck2_8=msg.pose.covariance[8]
               self.covariance_epuck2=np.array([[self.covariance_epuck2_0, self.covariance_epuck2_1, self.covariance_epuck2_2],[self.covariance_epuck2_3, self.covariance_epuck2_4, self.covariance_epuck2_5],[self.covariance_epuck2_6, self.covariance_epuck2_7, self.covariance_epuck2_8]])


    def  Filtered_epuck3(self,msg):
               self.filtered_x_epuck3 = msg.pose.pose.position.x
               self.filtered_y_epuck3 = msg.pose.pose.position.y
               self.filtered_theta_epuck3 = msg.pose.pose.orientation.z
               self.filtered_epuck3=matrix([self.filtered_x_epuck3,self.filtered_y_epuck3,self.filtered_theta_epuck3])
               self.covariance_epuck3_0=msg.pose.covariance[0]
               self.covariance_epuck3_1=msg.pose.covariance[1]
               self.covariance_epuck3_2=msg.pose.covariance[2]
               self.covariance_epuck3_3=msg.pose.covariance[3]
               self.covariance_epuck3_4=msg.pose.covariance[4]
               self.covariance_epuck3_5=msg.pose.covariance[5]
               self.covariance_epuck3_6=msg.pose.covariance[6]
               self.covariance_epuck3_7=msg.pose.covariance[7]
               self.covariance_epuck3_8=msg.pose.covariance[8]
               self.covariance_epuck3=np.array([[self.covariance_epuck3_0, self.covariance_epuck3_1, self.covariance_epuck3_2],[self.covariance_epuck3_3, self.covariance_epuck3_4, self.covariance_epuck3_5],[self.covariance_epuck3_6, self.covariance_epuck3_7, self.covariance_epuck3_8]])





    def  Camera_epuck0(self,AlvarMarkers_epuck0):
           self.camera_x_epuck0=1.7*AlvarMarkers_epuck0.markers[0].pose.pose.position.x - 0.3 +0.127 + 0.063 + 0.02 +0.01
           self.camera_y_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.position.y -0.109 -0.01
           self.camera_z_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.position.z
           x_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.x
           y_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.y
           z_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.z
           w_epuck0=AlvarMarkers_epuck0.markers[0].pose.pose.orientation.w
           self.camera_epuck0=matrix([self.camera_x_epuck0,self.camera_y_epuck0,self.camera_z_epuck0])          
           siny_cosp_epuck0 = 2 * (w_epuck0 * z_epuck0 + x_epuck0 * y_epuck0)
           cosy_cosp_epuck0 = w_epuck0**2 + x_epuck0**2 - y_epuck0**2 -z_epuck0**2
           self.yaw_epuck1 = atan2(siny_cosp_epuck0, cosy_cosp_epuck0)
           #print self.yaw_epuck1*(180/pi)



    def  Camera_epuck1(self,AlvarMarkers_epuck1):
           self.camera_x_epuck1=1.775*AlvarMarkers_epuck1.markers[1].pose.pose.position.x -0.36 + 0.15 +0.045 -0.09 + 0.02
           self.camera_y_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.position.y + 0.023 -0.017
           self.camera_z_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.position.z
           x_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.x
           y_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.y
           z_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.z
           w_epuck1=AlvarMarkers_epuck1.markers[1].pose.pose.orientation.w
           self.camera_epuck1=matrix([self.camera_x_epuck1,self.camera_y_epuck1,self.camera_z_epuck1])          
           siny_cosp_epuck1 = 2 * (w_epuck1 * z_epuck1 + x_epuck1 * y_epuck1)
           cosy_cosp_epuck1 = w_epuck1**2 + x_epuck1**2 - y_epuck1**2 -z_epuck1**2
           self.yaw_epuck0 = atan2(siny_cosp_epuck1, cosy_cosp_epuck1)


    def  Camera_epuck2(self,AlvarMarkers_epuck2):
           self.camera_x_epuck2=1.85*AlvarMarkers_epuck2.markers[2].pose.pose.position.x -0.36 + 0.15 -0.0542 -0.08 -0.01
           self.camera_y_epuck2=1.05*AlvarMarkers_epuck2.markers[2].pose.pose.position.y + 0.0115 -0.04 +0.03
           self.camera_z_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.position.z
           x_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.x
           y_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.y
           z_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.z
           w_epuck2=AlvarMarkers_epuck2.markers[2].pose.pose.orientation.w
           self.camera_epuck2=matrix([self.camera_x_epuck2,self.camera_y_epuck2,self.camera_z_epuck2])          
           siny_cosp_epuck2 = 2 * (w_epuck2 * z_epuck2 + x_epuck2 * y_epuck2)
           cosy_cosp_epuck2 = w_epuck2**2 + x_epuck2**2 - y_epuck2**2 -z_epuck2**2
           self.yaw_epuck2 = atan2(siny_cosp_epuck2, cosy_cosp_epuck2)


    def  Camera_epuck3(self,AlvarMarkers_epuck3):
           self.camera_x_epuck3=1.6*AlvarMarkers_epuck3.markers[-1].pose.pose.position.x -0.36 + 0.15 -0.0377
           self.camera_y_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.position.y + 0.13
           self.camera_z_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.position.z
           x_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.x
           y_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.y
           z_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.z
           w_epuck3=AlvarMarkers_epuck3.markers[-1].pose.pose.orientation.w
           self.camera_epuck3=matrix([self.camera_x_epuck3,self.camera_y_epuck3,self.camera_z_epuck3])          
           siny_cosp_epuck3 = 2 * (w_epuck3 * z_epuck3 + x_epuck3 * y_epuck3)
           cosy_cosp_epuck3 = w_epuck3**2 + x_epuck3**2 - y_epuck3**2 -z_epuck3**2
           self.yaw_epuck3 = atan2(siny_cosp_epuck3, cosy_cosp_epuck3)


  




    def  callback_epuck0(self,odom_sens):

         if(self.first_run_epuck0):
            self.previous_x_epuck0 = odom_sens.pose.pose.position.x
            self.previous_y_epuck0 = odom_sens.pose.pose.position.y
         x_epuck0 = odom_sens.pose.pose.position.x
         y_epuck0 = odom_sens.pose.pose.position.y
         theta_epuck0=0
         d_increment_epuck0 = sqrt((x_epuck0 - self.previous_x_epuck0) * (x_epuck0 - self.previous_x_epuck0) +
                   (y_epuck0 - self.previous_y_epuck0)*(y_epuck0 - self.previous_y_epuck0))

         self.leftStepsDiff_epuck0 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck0    # Expressed in meters.
         self.rightStepsDiff_epuck0 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck0  # Expressed in meters.

         self.deltaTheta_epuck0 = (self.rightStepsDiff_epuck0 - self.leftStepsDiff_epuck0)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck0 = (self.rightStepsDiff_epuck0 + self.leftStepsDiff_epuck0)/2  # Expressed in meters.

         self.x_pos_epuck0 += self.deltaSteps_epuck0*math.cos(self.theta_epuck0 + self.deltaTheta_epuck0)  # Expressed in meters.
         self.y_pos_epuck0 += self.deltaSteps_epuck0*math.sin(self.theta_epuck0 + self.deltaTheta_epuck0)  # Expressed in meters.
         self.theta_epuck0 += 2*self.deltaTheta_epuck0   # Expressed in radiant.

         self.leftStepsPrev_epuck0 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck0 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck0=d_increment_epuck0*1000 

         self.total_distance_epuck0 = self.total_distance_epuck0 + d_increment_epuck0
         #print "Total distance traveled e-puck0 is %0.2f"  %    self.total_distance_epuck0
         self.previous_x_epuck0=odom_sens.pose.pose.position.x
         self.previous_y_epuck0=odom_sens.pose.pose.position.y
         self.first_run_epuck0 = False
         self.ut_epuck0=[self.total_distance_epuck0,self.total_distance_epuck0]
         self.Localization_epuck0(self.ut_epuck0,self.x_pos_epuck0, self.y_pos_epuck0, self.theta_epuck0, self.flag_epuck0)
         


    def  callback_epuck1(self,odom_sens):
         
         if(self.first_run_epuck1):
            self.previous_x_epuck1 = odom_sens.pose.pose.position.x
            self.previous_y_epuck1 = odom_sens.pose.pose.position.y
         x_epuck1 = odom_sens.pose.pose.position.x
         y_epuck1 = odom_sens.pose.pose.position.y
         theta_epuck1=0
         d_increment_epuck1 = sqrt((x_epuck1 - self.previous_x_epuck1) * (x_epuck1 - self.previous_x_epuck1) +
                   (y_epuck1 - self.previous_y_epuck1)*(y_epuck1 - self.previous_y_epuck1))

         self.leftStepsDiff_epuck1 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck1    # Expressed in meters.
         self.rightStepsDiff_epuck1 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck1  # Expressed in meters.

         self.deltaTheta_epuck1 = (self.rightStepsDiff_epuck1 - self.leftStepsDiff_epuck1)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck1 = (self.rightStepsDiff_epuck1 + self.leftStepsDiff_epuck1)/2  # Expressed in meters.

         self.x_pos_epuck1 += self.deltaSteps_epuck1*math.cos(self.theta_epuck1 + self.deltaTheta_epuck1)  # Expressed in meters.
         self.y_pos_epuck1 += self.deltaSteps_epuck1*math.sin(self.theta_epuck1 + self.deltaTheta_epuck1)  # Expressed in meters.
         self.theta_epuck1 += 2*self.deltaTheta_epuck1   # Expressed in radiant.

         self.leftStepsPrev_epuck1 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck1 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck1=d_increment_epuck1*1000 
         self.total_distance_epuck1 = self.total_distance_epuck1 + d_increment_epuck1
         self.previous_x_epuck1=odom_sens.pose.pose.position.x
         self.previous_y_epuck1=odom_sens.pose.pose.position.y
         self.first_run_epuck1 = False
         self.ut_epuck1=[self.total_distance_epuck1,self.total_distance_epuck1]
         self.Localization_epuck1(self.ut_epuck1,self.x_pos_epuck1, self.y_pos_epuck1, self.theta_epuck1,self.flag_epuck1)


    def  callback_epuck2(self,odom_sens):

         if(self.first_run_epuck2):
            self.previous_x_epuck2 = odom_sens.pose.pose.position.x
            self.previous_y_epuck2 = odom_sens.pose.pose.position.y
         x_epuck2 = odom_sens.pose.pose.position.x
         y_epuck2 = odom_sens.pose.pose.position.y
         theta_epuck2=0
         d_increment_epuck2 = sqrt((x_epuck2 - self.previous_x_epuck2) * (x_epuck2 - self.previous_x_epuck2) +
                   (y_epuck2 - self.previous_y_epuck2)*(y_epuck2 - self.previous_y_epuck2))

         self.leftStepsDiff_epuck2 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck2    # Expressed in meters.
         self.rightStepsDiff_epuck2 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck2  # Expressed in meters.

         self.deltaTheta_epuck2 = (self.rightStepsDiff_epuck2 - self.leftStepsDiff_epuck2)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck2 = (self.rightStepsDiff_epuck2 + self.leftStepsDiff_epuck2)/2  # Expressed in meters.

         self.x_pos_epuck2 += self.deltaSteps_epuck2*math.cos(self.theta_epuck2 + self.deltaTheta_epuck2)  # Expressed in meters.
         self.y_pos_epuck2 += self.deltaSteps_epuck2*math.sin(self.theta_epuck2 + self.deltaTheta_epuck2)  # Expressed in meters.
         self.theta_epuck2 += 2*self.deltaTheta_epuck2   # Expressed in radiant.

         self.leftStepsPrev_epuck2 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck2 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck2=d_increment_epuck2*1000 

         self.total_distance_epuck2 = self.total_distance_epuck2 + d_increment_epuck2
         #print "Total distance traveled e-puck0 is %0.2f"  %    self.total_distance_epuck0
         self.previous_x_epuck2=odom_sens.pose.pose.position.x
         self.previous_y_epuck2=odom_sens.pose.pose.position.y
         self.first_run_epuck2 = False
         self.ut_epuck2=[self.total_distance_epuck2,self.total_distance_epuck2]
         self.Localization_epuck2(self.ut_epuck2,self.x_pos_epuck2, self.y_pos_epuck2, self.theta_epuck2, self.flag_epuck2)




    def  callback_epuck3(self,odom_sens):

         if(self.first_run_epuck3):
            self.previous_x_epuck3 = odom_sens.pose.pose.position.x
            self.previous_y_epuck3 = odom_sens.pose.pose.position.y
         x_epuck3 = odom_sens.pose.pose.position.x
         y_epuck3 = odom_sens.pose.pose.position.y
         theta_epuck3=0
         d_increment_epuck3 = sqrt((x_epuck3 - self.previous_x_epuck3) * (x_epuck3 - self.previous_x_epuck3) +
                   (y_epuck3 - self.previous_y_epuck3)*(y_epuck3 - self.previous_y_epuck3))

         self.leftStepsDiff_epuck3 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST - self.leftStepsPrev_epuck3    # Expressed in meters.
         self.rightStepsDiff_epuck3 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST - self.rightStepsPrev_epuck3  # Expressed in meters.

         self.deltaTheta_epuck3 = (self.rightStepsDiff_epuck3 - self.leftStepsDiff_epuck3)/2*WHEEL_DISTANCE # Expressed in radiant.
         self.deltaSteps_epuck3 = (self.rightStepsDiff_epuck3 + self.leftStepsDiff_epuck3)/2  # Expressed in meters.

         self.x_pos_epuck3 += self.deltaSteps_epuck3*math.cos(self.theta_epuck3 + self.deltaTheta_epuck3)  # Expressed in meters.
         self.y_pos_epuck3 += self.deltaSteps_epuck3*math.sin(self.theta_epuck3 + self.deltaTheta_epuck3)  # Expressed in meters.
         self.theta_epuck3 += 2*self.deltaTheta_epuck3   # Expressed in radiant.

         self.leftStepsPrev_epuck3 = odom_sens.twist.twist.linear.x*MOT_STEP_DIST  # Expressed in meters.
         self.rightStepsPrev_epuck3 = odom_sens.twist.twist.linear.y*MOT_STEP_DIST    # Expressed in meters.
         self.flag_epuck3=d_increment_epuck3*1000 

         self.total_distance_epuck3 = self.total_distance_epuck3 + d_increment_epuck3
         #print "Total distance traveled e-puck0 is %0.2f"  %    self.total_distance_epuck0
         self.previous_x_epuck3=odom_sens.pose.pose.position.x
         self.previous_y_epuck3=odom_sens.pose.pose.position.y
         self.first_run_epuck3 = False
         self.ut_epuck3=[self.total_distance_epuck3,self.total_distance_epuck3]
         self.Localization_epuck3(self.ut_epuck3,self.x_pos_epuck3, self.y_pos_epuck3, self.theta_epuck3, self.flag_epuck3)

    
         
         
    def ekf2w_localization_epuck0(self,moving_robot,stationary_robot,ut_epuck0,Z_epuck0,deltaTheta_epuck0,deltaSteps_epuck0,x_pos_epuck0,y_pos_epuck0,theta_epuck0,camera_epuck0,flag_epuck0):
		

                Z=transpose(matrix(Z_epuck0))
                movingRobot=moving_robot
                stationaryRobot=stationary_robot
                ut=matrix(ut_epuck0)
                
                theta_epuck0_norm=normalizeAngle(theta_epuck0)

                print Z


		# Set the current Step which is the last element in a list
		currentStep=-1



		#   Compute the current noiseless pose from previous noiseless Pose
                self.Robots[movingRobot].mu_expected=matrix([x_pos_epuck0,y_pos_epuck0,theta_epuck0_norm])
                #print self.Robots[movingRobot].mu_expected

                
                x_est_pre_epuck0=self.Robots[movingRobot].mu[-1][0,0]
                y_est_pre_epuck0=self.Robots[movingRobot].mu[-1][0,1]
                theta_est_pre_epuck0=self.Robots[movingRobot].mu[-1][0,2]

               
                
                x_est_cur_epuck0 = x_est_pre_epuck0 + deltaSteps_epuck0*math.cos(theta_est_pre_epuck0 + deltaTheta_epuck0)  # Expressed in meters.
                y_est_cur_epuck0 = y_est_pre_epuck0 + deltaSteps_epuck0*math.sin(theta_est_pre_epuck0 + deltaSteps_epuck0)  # Expressed in meters.
                theta_est_cur_epuck0 = theta_est_pre_epuck0+ 2*deltaTheta_epuck0   # Expressed in radiant.

		#   Estimate the predicted (priori) state of the robot  from previous filtered pose and control input ut
		# mu_bar=matrix([self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]])

                theta_est_cur_epuck0_norm=normalizeAngle(theta_est_cur_epuck0)

                mu_bar=matrix([x_est_cur_epuck0,y_est_cur_epuck0,theta_est_cur_epuck0])
                
                print mu_bar
                
                
                print deltaTheta_epuck0
                print deltaSteps_epuck0

		#   Compute the partial derivatives of the predicted state w.r.t control input(G_ut) and w.r.t previous
		#   corrected state (G_mut).
		[G_mut, G_ut]=evaluatePredictionJacobians(deltaSteps_epuck0,self.Robots[movingRobot].mu[currentStep], deltaTheta_epuck0)
               
                print G_mut

                filtered_epuck=[self.filtered_epuck0,self.filtered_epuck1,self.filtered_epuck2, self.filtered_epuck3]
                covariance_epuck=[self.covariance_epuck0,self.covariance_epuck1,self.covariance_epuck2, self.covariance_epuck3]
                
                self.Robots[stationaryRobot].mu[currentStep]=filtered_epuck[stationaryRobot]
                
                #print self.Robots[stationaryRobot].mu[currentStep]

                print filtered_epuck[stationaryRobot]

		#   Estimate Z_bar
		Z_bar=estimateRelativePose(mu_bar,self.Robots[stationaryRobot].mu[currentStep],self.Robots[movingRobot].S0)

                print Z_bar
		#   Estimate Z_diff
		Z_diff=evaluateRelativePoseDifference(Z,Z_bar)
		print Z_diff

                

		#############################################################
		#updatedK=abs(Z_diff[1,0]/pi)
		#self.Robots[movingRobot].K=matrix([updatedK,updatedK])
		##############################################################


		#   Determine the covariance Matrix of the wheel encoders
		Rt=getOdometryCovariance(ut,self.Robots[movingRobot].K)



                
                
		
		#   Estimate the predicted (priori) Covariance (Sigma Bar)
		sigma_bar=G_mut*self.Robots[movingRobot].sigma[currentStep]*(transpose(G_mut)) + G_ut*Rt*(transpose(G_ut))

                print sigma_bar
		



		#   Estimate the partial derivative of the observation/measurement w.r.t to
		#   predicted state (priori)
		[Hr,Hl]=evaluateMeasurementJacobians(mu_bar,self.Robots[stationaryRobot].mu[currentStep])
		#######################################
		#Hl=Hl*0
		#######################################


		
		#   Determine the covariance matrix of the sensors
		Qt=getSensorCovariance(self.Robots[movingRobot].S)

		self.Robots[stationaryRobot].sigma=[covariance_epuck[stationaryRobot]]


		#   Calculate innovation (residula) covariance
		S= (Hr)*(sigma_bar)*(transpose(Hr))+ (Hl)*(self.Robots[stationaryRobot].sigma[currentStep])*(transpose(Hl))+Qt

		
		#   Calculate Kalman Gain (Kgain)
		Kgain=  (sigma_bar)*(transpose(Hr))*(linalg.inv(S))
                
                print Kgain


		#   Update mean
		self.Robots[movingRobot].mu.append(mu_bar+ (transpose(Kgain*Z_diff)))

		#   Normalize theta to [-pi,pi]
		self.Robots[movingRobot].mu[currentStep][0,2]=normalizeAngle(self.Robots[movingRobot].mu[currentStep][0,2])
                
		#   Update Covariance
                self.Robots[movingRobot].sigma.append((identity(3)-Kgain*Hr)*sigma_bar)
                print self.Robots[movingRobot].sigma
		print "Robot Number:",movingRobot
		print "Odometers",self.Robots[movingRobot].mu_expected[-1][0,0],self.Robots[movingRobot].mu_expected[-1][0,1],self.Robots[movingRobot].mu_expected[-1][0,2]*(180/pi)
		print "Filtered:", self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]*(180/pi)

		#	Draw the new position on graph


		# Write to external File



		#  Print and Increment the number of localization steps
		#print "Step Number:",self.stepNumber
		self.stepNumber=self.stepNumber+1


                camera_msg = Odometry()
                camera_msg.header.stamp = rospy.Time.now()
                camera_msg.header.frame_id = "odom"
                camera_msg.child_frame_id = "/base_link"
                camera_msg.pose.pose.position = Point(camera_epuck0[0,0], camera_epuck0[0,1], camera_epuck0[0,2])
                q = tf.transformations.quaternion_from_euler(0, 0, camera_epuck0[0,2])
                camera_msg.pose.pose.orientation = Quaternion(*q)
                self.endTime = time.time()
                self.camera_aurco_epuck0.publish(camera_msg)
                

                ekf_odom_msg = Odometry()
                ekf_odom_msg.header.stamp = rospy.Time.now()
                ekf_odom_msg.header.frame_id = "base_link"
                ekf_odom_msg.pose.pose.position = Point(self.Robots[movingRobot].mu[-1][0,0], self.Robots[movingRobot].mu[-1][0,1], 0)
                ekf_odom_msg.pose.pose.orientation.z = self.Robots[movingRobot].mu[-1][0,2]
                ekf_odom_msg.pose.covariance[0]=self.Robots[movingRobot].sigma[-1][0,0]
                ekf_odom_msg.pose.covariance[1]=self.Robots[movingRobot].sigma[-1][0,1]
                ekf_odom_msg.pose.covariance[2]=self.Robots[movingRobot].sigma[-1][0,2]
                ekf_odom_msg.pose.covariance[3]=self.Robots[movingRobot].sigma[-1][1,0]
                ekf_odom_msg.pose.covariance[4]=self.Robots[movingRobot].sigma[-1][1,1]
                ekf_odom_msg.pose.covariance[5]=self.Robots[movingRobot].sigma[-1][1,2]
                ekf_odom_msg.pose.covariance[6]=self.Robots[movingRobot].sigma[-1][2,0]
                ekf_odom_msg.pose.covariance[7]=self.Robots[movingRobot].sigma[-1][2,1]
                ekf_odom_msg.pose.covariance[8]=self.Robots[movingRobot].sigma[-1][2,2]
                self.endTime = time.time()

                self.EKF_epuck0.publish(ekf_odom_msg)


                ekf_odom = Odometry()
                ekf_odom.header.stamp = rospy.Time.now()
                ekf_odom.header.frame_id = "base_link"
                ekf_odom.pose.pose.position = Point(1.5*self.Robots[movingRobot].mu[-1][0,0]-0.16+0.036-0.06, self.Robots[movingRobot].mu[-1][0,1], 0)
                self.endTime = time.time()
                self.EKF0.publish(ekf_odom)



                xEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,0])
                yEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,1])
                thetaEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,2])

                xEKF=str(self.Robots[movingRobot].mu[-1][0,0])
                yEKF=str(self.Robots[movingRobot].mu[-1][0,1])
                thetaEKF=str(self.Robots[movingRobot].mu[-1][0,2])
                Z_rel=str(Z)
                deltaSteps=str(self.deltaSteps_epuck0)
                deltaTheta=str(self.deltaTheta_epuck0)
                stepNumber=str(self.stepNumber)
             



                encoder_data=stepNumber+" "+str(movingRobot)+" "+xEncoder+" "+yEncoder+" "+thetaEncoder+"  "+Z_rel+" "+deltaSteps+" "+deltaTheta+'\n'
                ekf_data=stepNumber+" "+str(movingRobot)+" "+xEKF+" "+yEKF+" "+thetaEKF+'\n'

                with open("encoder_data.txt0","a") as encoderFile:
			if(stepNumber!=0):
				encoderFile.write(encoder_data)
			else:
				encoderFile.write("==============================================================================\n")    
				encoderFile.write(encoder_data)

                with open("ekf_data.txt0","a") as ekfFile:
			if(stepNumber!=0):
				ekfFile.write(ekf_data)  
			else:
				ekfFile.write("==============================================================================\n")    
				ekfFile.write(ekf_data)  




    def ekf2w_localization_epuck1(self,moving_robot,stationary_robot,ut_epuck1,Z_epuck1,deltaTheta_epuck1,deltaSteps_epuck1,x_pos_epuck1,y_pos_epuck1,theta_epuck1,camera_epuck1,flag_epuck1):

                Z=transpose(matrix(Z_epuck1))
                theta_epuck1_norm=normalizeAngle(theta_epuck1)
                movingRobot=moving_robot
                stationaryRobot=stationary_robot
                ut=matrix(ut_epuck1)
                

                
                print Z



		# Set the current Step which is the last element in a list
		currentStep=-1


                
		#   Compute the current noiseless pose from previous noiseless Pose
                self.Robots[movingRobot].mu_expected=matrix([x_pos_epuck1,y_pos_epuck1,theta_epuck1_norm])

               
                print self.Robots[movingRobot].mu_expected




                x_est_pre_epuck1=self.Robots[movingRobot].mu[-1][0,0]
                y_est_pre_epuck1=self.Robots[movingRobot].mu[-1][0,1]
                theta_est_pre_epuck1=self.Robots[movingRobot].mu[-1][0,2]
               
               
                x_est_cur_epuck1 = x_est_pre_epuck1 + deltaSteps_epuck1*math.cos(theta_est_pre_epuck1 + deltaTheta_epuck1)  # Expressed in meters.
                y_est_cur_epuck1 = y_est_pre_epuck1 + deltaSteps_epuck1*math.sin(theta_est_pre_epuck1 + deltaSteps_epuck1)  # Expressed in meters.
                theta_est_cur_epuck1 = theta_est_pre_epuck1+ 2*deltaTheta_epuck1   # Expressed in radiant.

		#   Estimate the predicted (priori) state of the robot  from previous filtered pose and control input ut
		# mu_bar=matrix([self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]])

                mu_bar=matrix([x_est_cur_epuck1,y_est_cur_epuck1,theta_est_cur_epuck1])
                
                print mu_bar


		#   Compute the partial derivatives of the predicted state w.r.t control input(G_ut) and w.r.t previous
		#   corrected state (G_mut).
		[G_mut, G_ut]=evaluatePredictionJacobians(deltaSteps_epuck1,self.Robots[movingRobot].mu[currentStep],deltaTheta_epuck1)

                filtered_epuck=[self.filtered_epuck0,self.filtered_epuck1, self.filtered_epuck2,self.filtered_epuck3]
                covariance_epuck=[self.covariance_epuck0,self.covariance_epuck1, self.covariance_epuck2,self.covariance_epuck3]

                self.Robots[stationaryRobot].mu[currentStep]=filtered_epuck[stationaryRobot]

               # print self.Robots[stationaryRobot].mu[currentStep]

                print filtered_epuck[stationaryRobot]
		#   Estimate Z_bar
		Z_bar=estimateRelativePose(mu_bar,self.Robots[stationaryRobot].mu[currentStep],self.Robots[movingRobot].S0)


                print Z_bar
		
		#   Estimate Z_diff
		Z_diff=evaluateRelativePoseDifference(Z,Z_bar)

                print Z_diff
                

		#############################################################
		#updatedK=abs(Z_diff[1,0]/pi)
		#self.Robots[movingRobot].K=matrix([updatedK,updatedK])
		##############################################################


		#   Determine the covariance Matrix of the wheel encoders
		Rt=getOdometryCovariance(ut,self.Robots[movingRobot].K)


                
		
		#   Estimate the predicted (priori) Covariance (Sigma Bar)
		sigma_bar=G_mut*self.Robots[movingRobot].sigma[currentStep]*(transpose(G_mut)) + G_ut*Rt*(transpose(G_ut))

                print sigma_bar
		



		#   Estimate the partial derivative of the observation/measurement w.r.t to
		#   predicted state (priori)
		[Hr,Hl]=evaluateMeasurementJacobians(mu_bar,self.Robots[stationaryRobot].mu[currentStep])
		#######################################
		#Hl=Hl*0
		#######################################
                

		
		#   Determine the covariance matrix of the sensors
		Qt=getSensorCovariance(self.Robots[movingRobot].S)
                 


		self.Robots[stationaryRobot].sigma=[covariance_epuck[stationaryRobot]]
                 




		#   Calculate innovation (residula) covariance
		S= (Hr)*(sigma_bar)*(transpose(Hr))+(Hl)*(self.Robots[stationaryRobot].sigma[currentStep])*(transpose(Hl))+Qt

		
		#   Calculate Kalman Gain (Kgain)
		Kgain=  (sigma_bar)*(transpose(Hr))*(linalg.inv(S))

                print Kgain

		#   Update mean
		self.Robots[movingRobot].mu.append(mu_bar+(transpose(Kgain*Z_diff)))

		#   Normalize theta to [-pi,pi]
		self.Robots[movingRobot].mu[currentStep][0,2]=normalizeAngle(self.Robots[movingRobot].mu[currentStep][0,2])


		#   Update Covariance
                self.Robots[movingRobot].sigma.append((identity(3)-Kgain*Hr)*sigma_bar)

                print self.Robots[movingRobot].sigma
                 

		print "Robot Number:",movingRobot
		print "Odometers",self.Robots[movingRobot].mu_expected[-1][0,0],self.Robots[movingRobot].mu_expected[-1][0,1],self.Robots[movingRobot].mu_expected[-1][0,2]*(180/pi)
		print "Filtered:", self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]*(180/pi)

		#	Draw the new position on graph


		# Write to external File



		#  Print and Increment the number of localization steps
		#print "Step Number:",self.stepNumber
		self.stepNumber=self.stepNumber+1

                camera_msg = Odometry()
                camera_msg.header.stamp = rospy.Time.now()
                camera_msg.header.frame_id = "/base_link"
                camera_msg.pose.pose.position = Point(camera_epuck1[0,0], camera_epuck1[0,1], camera_epuck1[0,2])
                q = tf.transformations.quaternion_from_euler(0, 0, camera_epuck1[0,2])
                camera_msg.pose.pose.orientation = Quaternion(*q)
                self.endTime = time.time()
                self.camera_aurco_epuck1.publish(camera_msg)


                ekf_odom_msg = Odometry()
                ekf_odom_msg.header.stamp = rospy.Time.now()
                ekf_odom_msg.header.frame_id = "odom"
                ekf_odom_msg.child_frame_id = "base_link"
                ekf_odom_msg.pose.pose.position = Point(self.Robots[movingRobot].mu[-1][0,0], self.Robots[movingRobot].mu[-1][0,1], 0)
                ekf_odom_msg.pose.pose.orientation.z = self.Robots[movingRobot].mu[-1][0,2]
                ekf_odom_msg.pose.covariance[0]=self.Robots[movingRobot].sigma[-1][0,0]
                ekf_odom_msg.pose.covariance[1]=self.Robots[movingRobot].sigma[-1][0,1]
                ekf_odom_msg.pose.covariance[2]=self.Robots[movingRobot].sigma[-1][0,2]
                ekf_odom_msg.pose.covariance[3]=self.Robots[movingRobot].sigma[-1][1,0]
                ekf_odom_msg.pose.covariance[4]=self.Robots[movingRobot].sigma[-1][1,1]
                ekf_odom_msg.pose.covariance[5]=self.Robots[movingRobot].sigma[-1][1,2]
                ekf_odom_msg.pose.covariance[6]=self.Robots[movingRobot].sigma[-1][2,0]
                ekf_odom_msg.pose.covariance[7]=self.Robots[movingRobot].sigma[-1][2,1]
                ekf_odom_msg.pose.covariance[8]=self.Robots[movingRobot].sigma[-1][2,2]
                self.endTime = time.time()
                self.EKF_epuck1.publish(ekf_odom_msg)

                ekf_odom = Odometry()
                ekf_odom.header.stamp = rospy.Time.now()
                ekf_odom.header.frame_id = "base_link"
                ekf_odom.pose.pose.position = Point(1*self.Robots[movingRobot].mu[-1][0,0]-0.026+0.02, self.Robots[movingRobot].mu[-1][0,1], 0)
                self.endTime = time.time()
                self.EKF1.publish(ekf_odom)
   
	

	#	Writes ekf and odometry data to external files.


                xEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,0])
                yEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,1])
                thetaEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,2])

                xEKF=str(self.Robots[movingRobot].mu[-1][0,0])
                yEKF=str(self.Robots[movingRobot].mu[-1][0,1])
                thetaEKF=str(self.Robots[movingRobot].mu[-1][0,2])
                Z_rel=str(Z)
                deltaSteps=str(self.deltaSteps_epuck1)
                deltaTheta=str(self.deltaTheta_epuck1)
                stepNumber=str(self.stepNumber)


                encoder_data=stepNumber+" "+str(movingRobot)+" "+xEncoder+" "+yEncoder+" "+thetaEncoder+"  "+Z_rel+" "+deltaSteps+" "+deltaTheta+'\n'
                ekf_data=stepNumber+" "+str(movingRobot)+" "+xEKF+" "+yEKF+" "+thetaEKF+'\n'

                with open("encoder_data.txt1","a") as encoderFile:
			if(stepNumber!=0):
				encoderFile.write(encoder_data)
			else:
				encoderFile.write("==============================================================================\n")    
				encoderFile.write(encoder_data)

                with open("ekf_data.txt1","a") as ekfFile:
			if(stepNumber!=0):
				ekfFile.write(ekf_data)  
			else:
				ekfFile.write("==============================================================================\n")    
				ekfFile.write(ekf_data)  



    def ekf2w_localization_epuck2(self,moving_robot,stationary_robot,ut_epuck2,Z_epuck2,deltaTheta_epuck2,deltaSteps_epuck2,x_pos_epuck2,y_pos_epuck2,theta_epuck2,camera_epuck2,flag_epuck2):
		

                Z=transpose(matrix(Z_epuck2))
                movingRobot=moving_robot
                theta_epuck2_norm=normalizeAngle(theta_epuck2)
                stationaryRobot=stationary_robot
                ut=matrix(ut_epuck2)
                
                theta_epuck2_norm=normalizeAngle(theta_epuck2)

                print Z


		# Set the current Step which is the last element in a list
		currentStep=-1



		#   Compute the current noiseless pose from previous noiseless Pose
                self.Robots[movingRobot].mu_expected=matrix([x_pos_epuck2,y_pos_epuck2,theta_epuck2_norm])
                #print self.Robots[movingRobot].mu_expected

                
                x_est_pre_epuck2=self.Robots[movingRobot].mu[-1][0,0]
                y_est_pre_epuck2=self.Robots[movingRobot].mu[-1][0,1]
                theta_est_pre_epuck2=self.Robots[movingRobot].mu[-1][0,2]

               
                
                x_est_cur_epuck2 = x_est_pre_epuck2 + deltaSteps_epuck2*math.cos(theta_est_pre_epuck2 + deltaTheta_epuck2)  # Expressed in meters.
                y_est_cur_epuck2 = y_est_pre_epuck2 + deltaSteps_epuck2*math.sin(theta_est_pre_epuck2 + deltaSteps_epuck2)  # Expressed in meters.
                theta_est_cur_epuck2 = theta_est_pre_epuck2+ 2*deltaTheta_epuck2   # Expressed in radiant.

		#   Estimate the predicted (priori) state of the robot  from previous filtered pose and control input ut
		# mu_bar=matrix([self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]])

                theta_est_cur_epuck2_norm=normalizeAngle(theta_est_cur_epuck2)

                mu_bar=matrix([x_est_cur_epuck2,y_est_cur_epuck2,theta_est_cur_epuck2])
                
                print mu_bar
                
                
                print deltaTheta_epuck2
                print deltaSteps_epuck2

		#   Compute the partial derivatives of the predicted state w.r.t control input(G_ut) and w.r.t previous
		#   corrected state (G_mut).
		[G_mut, G_ut]=evaluatePredictionJacobians(deltaSteps_epuck2,self.Robots[movingRobot].mu[currentStep], deltaTheta_epuck2)
               
                print G_mut
                

                filtered_epuck=[self.filtered_epuck0,self.filtered_epuck1, self.filtered_epuck2,self.filtered_epuck3]
                covariance_epuck=[self.covariance_epuck0,self.covariance_epuck1, self.covariance_epuck2,self.covariance_epuck3]

                self.Robots[stationaryRobot].mu[currentStep]=filtered_epuck[stationaryRobot]
                
                #print self.Robots[stationaryRobot].mu[currentStep]

                print filtered_epuck[stationaryRobot]

		#   Estimate Z_bar
		Z_bar=estimateRelativePose(mu_bar,self.Robots[stationaryRobot].mu[currentStep],self.Robots[movingRobot].S0)

                print Z_bar
		#   Estimate Z_diff
		Z_diff=evaluateRelativePoseDifference(Z,Z_bar)
		print Z_diff

                

		#############################################################
		#updatedK=abs(Z_diff[1,0]/pi)
		#self.Robots[movingRobot].K=matrix([updatedK,updatedK])
		##############################################################


		#   Determine the covariance Matrix of the wheel encoders
		Rt=getOdometryCovariance(ut,self.Robots[movingRobot].K)



                
                
		
		#   Estimate the predicted (priori) Covariance (Sigma Bar)
		sigma_bar=G_mut*self.Robots[movingRobot].sigma[currentStep]*(transpose(G_mut)) + G_ut*Rt*(transpose(G_ut))

                print sigma_bar
		



		#   Estimate the partial derivative of the observation/measurement w.r.t to
		#   predicted state (priori)
		[Hr,Hl]=evaluateMeasurementJacobians(mu_bar,self.Robots[stationaryRobot].mu[currentStep])
		#######################################
		#Hl=Hl*0
		#######################################


		
		#   Determine the covariance matrix of the sensors
		Qt=getSensorCovariance(self.Robots[movingRobot].S)

		self.Robots[stationaryRobot].sigma=[covariance_epuck[stationaryRobot]]


		#   Calculate innovation (residula) covariance
		S= (Hr)*(sigma_bar)*(transpose(Hr))+ (Hl)*(self.Robots[stationaryRobot].sigma[currentStep])*(transpose(Hl))+Qt

		
		#   Calculate Kalman Gain (Kgain)
		Kgain=  (sigma_bar)*(transpose(Hr))*(linalg.inv(S))
                
                print Kgain


		#   Update mean
		self.Robots[movingRobot].mu.append(mu_bar+ (transpose(Kgain*Z_diff)))

		#   Normalize theta to [-pi,pi]
		self.Robots[movingRobot].mu[currentStep][0,2]=normalizeAngle(self.Robots[movingRobot].mu[currentStep][0,2])
                
		#   Update Covariance
                self.Robots[movingRobot].sigma.append((identity(3)-Kgain*Hr)*sigma_bar)
                print self.Robots[movingRobot].sigma
		print "Robot Number:",movingRobot
		print "Odometers",self.Robots[movingRobot].mu_expected[-1][0,0],self.Robots[movingRobot].mu_expected[-1][0,1],self.Robots[movingRobot].mu_expected[-1][0,2]*(180/pi)
		print "Filtered:", self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]*(180/pi)

		#	Draw the new position on graph


		# Write to external File



		#  Print and Increment the number of localization steps
		#print "Step Number:",self.stepNumber
		self.stepNumber=self.stepNumber+1


                camera_msg = Odometry()
                camera_msg.header.stamp = rospy.Time.now()
                camera_msg.header.frame_id = "odom"
                camera_msg.child_frame_id = "/base_link"
                camera_msg.pose.pose.position = Point(camera_epuck2[0,0], camera_epuck2[0,1], camera_epuck2[0,2])
                q = tf.transformations.quaternion_from_euler(0, 0, camera_epuck2[0,2])
                camera_msg.pose.pose.orientation = Quaternion(*q)
                self.endTime = time.time()
                self.camera_aurco_epuck2.publish(camera_msg)
                

                ekf_odom_msg = Odometry()
                ekf_odom_msg.header.stamp = rospy.Time.now()
                ekf_odom_msg.header.frame_id = "base_link"
                ekf_odom_msg.pose.pose.position = Point(self.Robots[movingRobot].mu[-1][0,0], self.Robots[movingRobot].mu[-1][0,1], 0)
                ekf_odom_msg.pose.pose.orientation.z = self.Robots[movingRobot].mu[-1][0,2]
                ekf_odom_msg.pose.covariance[0]=self.Robots[movingRobot].sigma[-1][0,0]
                ekf_odom_msg.pose.covariance[1]=self.Robots[movingRobot].sigma[-1][0,1]
                ekf_odom_msg.pose.covariance[2]=self.Robots[movingRobot].sigma[-1][0,2]
                ekf_odom_msg.pose.covariance[3]=self.Robots[movingRobot].sigma[-1][1,0]
                ekf_odom_msg.pose.covariance[4]=self.Robots[movingRobot].sigma[-1][1,1]
                ekf_odom_msg.pose.covariance[5]=self.Robots[movingRobot].sigma[-1][1,2]
                ekf_odom_msg.pose.covariance[6]=self.Robots[movingRobot].sigma[-1][2,0]
                ekf_odom_msg.pose.covariance[7]=self.Robots[movingRobot].sigma[-1][2,1]
                ekf_odom_msg.pose.covariance[8]=self.Robots[movingRobot].sigma[-1][2,2]
                self.endTime = time.time()
                self.EKF_epuck2.publish(ekf_odom_msg)


                ekf_odom = Odometry()
                ekf_odom.header.stamp = rospy.Time.now()
                ekf_odom.header.frame_id = "base_link"
                ekf_odom.pose.pose.position = Point(1.575*self.Robots[movingRobot].mu[-1][0,0]-0.268+0.14-0.017-0.02, self.Robots[movingRobot].mu[-1][0,1], 0)
                self.endTime = time.time()
                self.EKF2.publish(ekf_odom)



                xEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,0])
                yEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,1])
                thetaEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,2])

                xEKF=str(self.Robots[movingRobot].mu[-1][0,0])
                yEKF=str(self.Robots[movingRobot].mu[-1][0,1])
                thetaEKF=str(self.Robots[movingRobot].mu[-1][0,2])
                Z_rel=str(Z)
                deltaSteps=str(self.deltaSteps_epuck2)
                deltaTheta=str(self.deltaTheta_epuck2)
                stepNumber=str(self.stepNumber)
             



                encoder_data=stepNumber+" "+str(movingRobot)+" "+xEncoder+" "+yEncoder+" "+thetaEncoder+"  "+Z_rel+" "+deltaSteps+" "+deltaTheta+'\n'
                ekf_data=stepNumber+" "+str(movingRobot)+" "+xEKF+" "+yEKF+" "+thetaEKF+'\n'

                with open("encoder_data.txt0","a") as encoderFile:
			if(stepNumber!=0):
				encoderFile.write(encoder_data)
			else:
				encoderFile.write("==============================================================================\n")    
				encoderFile.write(encoder_data)

                with open("ekf_data.txt0","a") as ekfFile:
			if(stepNumber!=0):
				ekfFile.write(ekf_data)  
			else:
				ekfFile.write("==============================================================================\n")    
				ekfFile.write(ekf_data)  


    def ekf2w_localization_epuck3(self,moving_robot,stationary_robot,ut_epuck3,Z_epuck3,deltaTheta_epuck3,deltaSteps_epuck3,x_pos_epuck3,y_pos_epuck3,theta_epuck3,camera_epuck3,flag_epuck3):
		

                Z=transpose(matrix(Z_epuck3))
                movingRobot=moving_robot
                theta_epuck3_norm=normalizeAngle(theta_epuck3)
                stationaryRobot=stationary_robot
                print stationaryRobot
                ut=matrix(ut_epuck3)
                

                print Z


		# Set the current Step which is the last element in a list
		currentStep=-1



		#   Compute the current noiseless pose from previous noiseless Pose
                self.Robots[movingRobot].mu_expected=matrix([x_pos_epuck3,y_pos_epuck3,theta_epuck3_norm])
                #print self.Robots[movingRobot].mu_expected

                
                x_est_pre_epuck3=self.Robots[movingRobot].mu[-1][0,0]
                y_est_pre_epuck3=self.Robots[movingRobot].mu[-1][0,1]
                theta_est_pre_epuck3=self.Robots[movingRobot].mu[-1][0,2]

               
                
                x_est_cur_epuck3 = x_est_pre_epuck3 + deltaSteps_epuck3*math.cos(theta_est_pre_epuck3 + deltaTheta_epuck3)  # Expressed in meters.
                y_est_cur_epuck3 = y_est_pre_epuck3 + deltaSteps_epuck3*math.sin(theta_est_pre_epuck3 + deltaSteps_epuck3)  # Expressed in meters.
                theta_est_cur_epuck3 = theta_est_pre_epuck3+ 2*deltaTheta_epuck3   # Expressed in radiant.

		#   Estimate the predicted (priori) state of the robot  from previous filtered pose and control input ut
		# mu_bar=matrix([self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]])

                theta_est_cur_epuck3_norm=normalizeAngle(theta_est_cur_epuck3)

                mu_bar=matrix([x_est_cur_epuck3,y_est_cur_epuck3,theta_est_cur_epuck3])
                
                print mu_bar
                
                
                print deltaTheta_epuck3
                print deltaSteps_epuck3

		#   Compute the partial derivatives of the predicted state w.r.t control input(G_ut) and w.r.t previous
		#   corrected state (G_mut).
		[G_mut, G_ut]=evaluatePredictionJacobians(deltaSteps_epuck3,self.Robots[movingRobot].mu[currentStep], deltaTheta_epuck3)
               
                print G_mut
                

                filtered_epuck=[self.filtered_epuck0,self.filtered_epuck1, self.filtered_epuck2,self.filtered_epuck3]
                covariance_epuck=[self.covariance_epuck0,self.covariance_epuck1, self.covariance_epuck2,self.covariance_epuck3]

                self.Robots[stationaryRobot].mu[currentStep]=filtered_epuck[stationaryRobot]
                
                #print self.Robots[stationaryRobot].mu[currentStep]

                print filtered_epuck[stationaryRobot]

		#   Estimate Z_bar
		Z_bar=estimateRelativePose(mu_bar,self.Robots[stationaryRobot].mu[currentStep],self.Robots[movingRobot].S0)

                print Z_bar
		#   Estimate Z_diff
		Z_diff=evaluateRelativePoseDifference(Z,Z_bar)
		print Z_diff

                

		#############################################################
		#updatedK=abs(Z_diff[1,0]/pi)
		#self.Robots[movingRobot].K=matrix([updatedK,updatedK])
		##############################################################


		#   Determine the covariance Matrix of the wheel encoders
		Rt=getOdometryCovariance(ut,self.Robots[movingRobot].K)



                
                
		
		#   Estimate the predicted (priori) Covariance (Sigma Bar)
		sigma_bar=G_mut*self.Robots[movingRobot].sigma[currentStep]*(transpose(G_mut)) + G_ut*Rt*(transpose(G_ut))

                print sigma_bar
		



		#   Estimate the partial derivative of the observation/measurement w.r.t to
		#   predicted state (priori)
		[Hr,Hl]=evaluateMeasurementJacobians(mu_bar,self.Robots[stationaryRobot].mu[currentStep])
		#######################################
		#Hl=Hl*0
		#######################################


		
		#   Determine the covariance matrix of the sensors
		Qt=getSensorCovariance(self.Robots[movingRobot].S)

		self.Robots[stationaryRobot].sigma=[covariance_epuck[stationaryRobot]]


		#   Calculate innovation (residula) covariance
		S= (Hr)*(sigma_bar)*(transpose(Hr))+ (Hl)*(self.Robots[stationaryRobot].sigma[currentStep])*(transpose(Hl))+Qt

		
		#   Calculate Kalman Gain (Kgain)
		Kgain=  (sigma_bar)*(transpose(Hr))*(linalg.inv(S))
                
                print Kgain


		#   Update mean
		self.Robots[movingRobot].mu.append(mu_bar+ (transpose(Kgain*Z_diff)))

		#   Normalize theta to [-pi,pi]
		self.Robots[movingRobot].mu[currentStep][0,2]=normalizeAngle(self.Robots[movingRobot].mu[currentStep][0,2])
                
		#   Update Covariance
                self.Robots[movingRobot].sigma.append((identity(3)-Kgain*Hr)*sigma_bar)
                print self.Robots[movingRobot].sigma
		print "Robot Number:",movingRobot
		print "Odometers",self.Robots[movingRobot].mu_expected[-1][0,0],self.Robots[movingRobot].mu_expected[-1][0,1],self.Robots[movingRobot].mu_expected[-1][0,2]*(180/pi)
		print "Filtered:", self.Robots[movingRobot].mu[-1][0,0],self.Robots[movingRobot].mu[-1][0,1],self.Robots[movingRobot].mu[-1][0,2]*(180/pi)

		#	Draw the new position on graph


		# Write to external File



		#  Print and Increment the number of localization steps
		#print "Step Number:",self.stepNumber
		self.stepNumber=self.stepNumber+1


                camera_msg = Odometry()
                camera_msg.header.stamp = rospy.Time.now()
                camera_msg.header.frame_id = "odom"
                camera_msg.child_frame_id = "/base_link"
                camera_msg.pose.pose.position = Point(camera_epuck3[0,0], camera_epuck3[0,1], camera_epuck3[0,2])
                q = tf.transformations.quaternion_from_euler(0, 0, camera_epuck3[0,2])
                camera_msg.pose.pose.orientation = Quaternion(*q)
                self.endTime = time.time()
                self.camera_aurco_epuck3.publish(camera_msg)
                

                ekf_odom_msg = Odometry()
                ekf_odom_msg.header.stamp = rospy.Time.now()
                ekf_odom_msg.header.frame_id = "base_link"
                ekf_odom_msg.pose.pose.position = Point(self.Robots[movingRobot].mu[-1][0,0], self.Robots[movingRobot].mu[-1][0,1], 0)
                ekf_odom_msg.pose.pose.orientation.z = self.Robots[movingRobot].mu[-1][0,2]
                ekf_odom_msg.pose.covariance[0]=self.Robots[movingRobot].sigma[-1][0,0]
                ekf_odom_msg.pose.covariance[1]=self.Robots[movingRobot].sigma[-1][0,1]
                ekf_odom_msg.pose.covariance[2]=self.Robots[movingRobot].sigma[-1][0,2]
                ekf_odom_msg.pose.covariance[3]=self.Robots[movingRobot].sigma[-1][1,0]
                ekf_odom_msg.pose.covariance[4]=self.Robots[movingRobot].sigma[-1][1,1]
                ekf_odom_msg.pose.covariance[5]=self.Robots[movingRobot].sigma[-1][1,2]
                ekf_odom_msg.pose.covariance[6]=self.Robots[movingRobot].sigma[-1][2,0]
                ekf_odom_msg.pose.covariance[7]=self.Robots[movingRobot].sigma[-1][2,1]
                ekf_odom_msg.pose.covariance[8]=self.Robots[movingRobot].sigma[-1][2,2]
                self.endTime = time.time()

                self.EKF_epuck3.publish(ekf_odom_msg)


                ekf_odom = Odometry()
                ekf_odom.header.stamp = rospy.Time.now()
                ekf_odom.header.frame_id = "base_link"
                ekf_odom.pose.pose.position = Point(1.3*self.Robots[movingRobot].mu[-1][0,0]-0.152+0.04, self.Robots[movingRobot].mu[-1][0,1], 0)
                self.endTime = time.time()
                self.EKF3.publish(ekf_odom)

                xEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,0])
                yEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,1])
                thetaEncoder=str(self.Robots[movingRobot].mu_expected[-1][0,2])

                xEKF=str(self.Robots[movingRobot].mu[-1][0,0])
                yEKF=str(self.Robots[movingRobot].mu[-1][0,1])
                thetaEKF=str(self.Robots[movingRobot].mu[-1][0,2])
                Z_rel=str(Z)
                deltaSteps=str(self.deltaSteps_epuck3)
                deltaTheta=str(self.deltaTheta_epuck3)
                stepNumber=str(self.stepNumber)
             



                encoder_data=stepNumber+" "+str(movingRobot)+" "+xEncoder+" "+yEncoder+" "+thetaEncoder+"  "+Z_rel+" "+deltaSteps+" "+deltaTheta+'\n'
                ekf_data=stepNumber+" "+str(movingRobot)+" "+xEKF+" "+yEKF+" "+thetaEKF+'\n'

                with open("encoder_data.txt0","a") as encoderFile:
			if(stepNumber!=0):
				encoderFile.write(encoder_data)
			else:
				encoderFile.write("==============================================================================\n")    
				encoderFile.write(encoder_data)

                with open("ekf_data.txt0","a") as ekfFile:
			if(stepNumber!=0):
				ekfFile.write(ekf_data)  
			else:
				ekfFile.write("==============================================================================\n")    
				ekfFile.write(ekf_data)  
         


if __name__ == '__main__':
    ic = Range_Bearing()
    rospy.init_node('range_bearing', anonymous=True)
    try:
          rospy.spin()
    except:
        rospy.loginfo("Range_Bearing node terminated.")

