#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped, Pose, Vector3, Pose2D
from visualization_msgs.msg import Marker
import numpy as np
import tf
import tf2_ros
from collections import deque

def get_yaw_from_quaternion(quat):
    return tf.transformations.euler_from_quaternion([quat.x,
                                                     quat.y,
                                                     quat.z,
                                                     quat.w])[2]

def create_transform_msg(translation, rotation, child_frame, base_frame, time=None):
    t = TransformStamped()
    t.header.stamp = time if time else rospy.Time.now()
    t.header.frame_id = base_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t

class MarkerViz:

    def __init__(self):
        rospy.init_node('rviz_markers')
        

        ## Use simulation time (i.e. get time from rostopic /clock)
        rospy.set_param('use_sim_time', 'true')
        rate = rospy.Rate(10)
        while rospy.Time.now() == rospy.Time(0):
            rate.sleep()


        self.marker_publisher = rospy.Publisher('/turtlebot_marker', Marker, queue_size = 10)
        self.goal_marker_publisher = rospy.Publisher('/turtlebot_nav_goal_marker', Marker, queue_size = 10) 
 

        rospy.Subscriber("/cmd_nav",Pose2D,self.nav_goal_callback) 

        self.got_goal_flag = False   
        
    
        


        print("successfully initialized rviz_markers.py")


    def nav_goal_callback(self,msg):

        self.nav_goal_position = msg
        self.got_goal_flag = True




    def run(self):

        rate = rospy.Rate(100)

        

        while not rospy.is_shutdown():

            self.trans_listener = tf.TransformListener()

            rate = rospy.Rate(10)

               
            while True:
                try:
                    (position,orientation) = self.trans_listener.lookupTransform('/map','/base_footprint',rospy.Time(0))
                    #print("got tf data,break")
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #print("sleeping")
                    rate.sleep()


           
        
            marker = Marker()
            marker.header.frame_id = "/base_footprint" #very important that this should match with the tf transform frame_id
            marker.header.stamp = rospy.Time()

            marker.ns = "robot"
            marker.id = 100		#arbitrary

            marker.type = marker.CUBE
            marker.action = marker.ADD

            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.08

            marker.color.a = 0.6
            marker.color.g = 0.0
            marker.color.r = 1.0
            marker.color.b = 0.5

            #print("pose of robot wrt odom =",position,orientation)

            marker.pose.orientation.x = 0 #orientation[0]
            marker.pose.orientation.y = 0 #orientation[1]
            marker.pose.orientation.z = 0 #orientation[2]
            marker.pose.orientation.w = 1 #orientation[3]

            #print("position",position)

            marker.pose.position.x = 0 #position[0]
            marker.pose.position.y = 0 #position[1]
            marker.pose.position.z = 0 #position[2]

            self.marker_publisher.publish(marker)




            marker_goal = Marker()
            marker_goal.header.frame_id = "/map" #very important that this should match with the tf transform frame_id
            marker_goal.header.stamp = rospy.Time()

            marker_goal.ns = "navgoal"
            marker_goal.id = 110 #arbitrary

            marker_goal.type = marker_goal.SPHERE
            marker_goal.action = marker_goal.ADD

            marker_goal.scale.x = 0.06
            marker_goal.scale.y = 0.06
            marker_goal.scale.z = 0.02

            marker_goal.color.a = 1.0
            marker_goal.color.g = 1.0
            marker_goal.color.r = 0.0
            marker_goal.color.b = 0.2

            #print("pose of robot wrt odom =",position,orientation)

            marker_goal.pose.orientation.x = 0 #orientation[0]
            marker_goal.pose.orientation.y = 0 #orientation[1]
            marker_goal.pose.orientation.z = 0 #orientation[2]
            marker_goal.pose.orientation.w = 1 #orientation[3]

            #print("position",position)

            

            if self.got_goal_flag:

                marker_goal.pose.position.x = self.nav_goal_position.x #position[0]
                marker_goal.pose.position.y = self.nav_goal_position.y #position[1]
                marker_goal.pose.position.z = 0 #position[2]

            else:

                marker_goal.pose.position.x = 0
                marker_goal.pose.position.y = 0
                marker_goal.pose.position.z = 0 #position[2]
                


            self.goal_marker_publisher.publish(marker_goal)




       

if __name__ == '__main__':
    #print("init marker_viz")
    marker_viz = MarkerViz()
    #print("localization running")

    marker_viz.run()
