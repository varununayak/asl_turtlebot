#!/usr/bin/env python
'''
Changes Made:

What																						When					Who
Added Enum type for parallel state machine													3/10,1:45PM				Varun
Added food_items data type to store coordinates												3/10 4:29PM				Varun
Added subscriber callback functions for	all food items										3/10 5:21PM				Varun
Added MACROS for food labels																3/10 5:45PM				Varun
self.state defaults to MAN_EXPLORATION														3/10 8:45PM				Varun
Added WAIT4FOOD Mode and extended state functionality										3/10 9:14PM				Kshitij
Added Order string subscriber and string processig call back								3/10 9:15PM				Kshitij

'''


import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# time to stop at a food joint
STOP4FOOD_TIME = 5 

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
	IDLE = 1
	POSE = 2
	STOP = 3
	CROSS = 4
	NAV = 5
	MANUAL = 6
	WAIT4FOOD = 7

class State(Enum):
	MAN_EXPLORATION = 1
	AUT_EXPLORATION = 2
	PICKUP = 3
	DELIVERY = 4
	WAIT4ORDER = 5
	


BANANA_LABEL = 52
APPLE_LABEL = 53
SANDWICH_LABEL = 54
ORANGE_LABEL = 55
BROCCOLI_LABEL = 56
CARROT_LABEL = 57
HOT_DOG_LABEL = 58
PIZZA_LABEL = 59
DONUT_LABEL = 60
CAKE_LABEL = 61

FOOD_THRESHOLD_DISTANCE = 1	#if item is closer than this only then register
SAME_ITEM_THRESHOLD = 2 #if same food item was seen within this distance it is not unique

labels = {
		"banana": 52,
		"apple": 53,
		"sandwich": 54,
		"orange": 55,
		"broccoli": 56,
		"carrot": 57,
		"hot_dog": 58,
		"pizza": 59,
		"donut": 60,
		"cake": 61,
	}




print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

	def __init__(self):
		rospy.init_node('turtlebot_supervisor', anonymous=True)
		# initialize variables
		self.x = 0
		self.y = 0
		self.theta = 0
		self.mode = Mode.IDLE
		self.state = State.MAN_EXPLORATION	# defaults to manual exploration
		self.last_mode_printed = None
		self.trans_listener = tf.TransformListener()
		# command pose for controller
		self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
		# nav pose for controller
		self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
		# command vel (used for idling)
		self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		self.food_items = []	#list of tuples for food items, (label,2Darray)
		

		# subscribers
		# food item detectors
		rospy.Subscriber('/detector/banana', DetectedObject, self.banana_detected_callback)
		rospy.Subscriber('/detector/hot_dog', DetectedObject, self.hot_dog_detected_callback)
		rospy.Subscriber('/detector/apple', DetectedObject, self.apple_detected_callback)
		rospy.Subscriber('/detector/sandwich', DetectedObject, self.sandwich_detected_callback)
		rospy.Subscriber('/detector/orange', DetectedObject, self.orange_detected_callback)
		rospy.Subscriber('/detector/pizza', DetectedObject, self.pizza_detected_callback)
		rospy.Subscriber('/detector/donut', DetectedObject, self.donut_detected_callback)
		rospy.Subscriber('/detector/cake', DetectedObject, self.cake_detected_callback)
		rospy.Subscriber('/detector/broccoli', DetectedObject, self.broccoli_detected_callback)
		rospy.Subscriber('/detector/carrot', DetectedObject, self.carrot_detected_callback)

		# stop sign detector
		rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
		# high-level navigation pose
		rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
		# if using gazebo, we have access to perfect state
		if use_gazebo:
			rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
		# we can subscribe to nav goal click
		rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
		# subscribe to order message
		rospy.Subscriber('/delivery_request', String, self.message_processing_callback)
		
		# subscriber to receive message that exploration is done
		rospy.Subscriber('/exploration_complete', Bool, self.exploration_complete_callback)


	def gazebo_callback(self, msg):
		pose = msg.pose[msg.name.index("turtlebot3_burger")]
		twist = msg.twist[msg.name.index("turtlebot3_burger")]
		self.x = pose.position.x
		self.y = pose.position.y
		quaternion = (
					pose.orientation.x,
					pose.orientation.y,
					pose.orientation.z,
					pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.theta = euler[2]


	def rviz_goal_callback(self, msg):
		""" callback for a pose goal sent through rviz """
		
		# Only follow rviz command when in manual exploration state

		if not self.state == State.MAN_EXPLORATION:
			return
		
		origin_frame = "/map" if mapping else "/odom"
		print("rviz command received!")
		try:
			
			nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
			self.x_g = nav_pose_origin.pose.position.x
			self.y_g = nav_pose_origin.pose.position.y
			quaternion = (
					nav_pose_origin.pose.orientation.x,
					nav_pose_origin.pose.orientation.y,
					nav_pose_origin.pose.orientation.z,
					nav_pose_origin.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			self.theta_g = euler[2]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
		self.mode = Mode.NAV
	

	def nav_pose_callback(self, msg):
		self.x_g = msg.x
		self.y_g = msg.y
		self.theta_g = msg.theta
		self.mode = Mode.NAV

	

	def stop_sign_detected_callback(self, msg):
		""" callback for when the detector has found a stop sign. Note that
		a distance of 0 can mean that the lidar did not pickup the stop sign at all """

		# distance of the stop sign
		dist = msg.distance

		# if close enough and in nav mode, stop
		if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
			self.init_stop_sign()
			
	def message_processing_callback(self,msg):
		orderString = msg.data
		self.orderList = orderString.split(',')
		self.state = State.PICKUP
		
	def exploration_completed_callback(self, msg):
	  """
	  Check's if exploration is completed. If it is: drive back home.
	  """
	  
	  exploration_complete = msg.data
	  
	  if exploration_complete:
		self.state = State.GETORDER	#go home first


	def banana_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):			
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == BANANA_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (BANANA_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
		
				
	def apple_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == APPLE_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break				
				if(new_item):
					item = (APPLE_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
	def sandwich_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == SANDWICH_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (SANDWICH_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
	def orange_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == ORANGE_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (ORANGE_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
				
	def broccoli_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):
		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == BROCCOLI_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (BROCCOLI_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
	def carrot_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == CARROT_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (CARROT_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
				
	def hot_dog_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == HOT_DOG_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (HOT_DOG_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
				
	def pizza_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == PIZZA_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break				
				if(new_item):
					item = (PIZZA_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
				
	def donut_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == DONUT_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (DONUT_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)
				
				
	def cake_detected_callback(self,msg):
		dist = msg.distance
		if(self.state == AUT_EXPLORATION or self.state == MAN_EXPLORATION):		
			if(dist < FOOD_THRESHOLD_DISTANCE):
				size = len(self.food_items)
				new_item = True	#assume new food item
				for i in range(size):
					if(self.food_items[i][0] == CAKE_LABEL ):
						if(np.linalg.norm(self.food_items[i][1] - np.array([self.x,self.y]) ) < SAME_ITEM_THRESHOLD):
							new_item = False	#same food item if found in somewhat same location
							break					
				if(new_item):
					item = (CAKE_LABEL,np.array([self.x,self.y]))
					self.food_items.append(item)




	def go_to_pose(self):
		""" sends the current desired pose to the pose controller """

		pose_g_msg = Pose2D()
		pose_g_msg.x = self.x_g
		pose_g_msg.y = self.y_g
		pose_g_msg.theta = self.theta_g

		self.pose_goal_publisher.publish(pose_g_msg)

	def nav_to_pose(self):
		""" sends the current desired pose to the naviagtor """

		nav_g_msg = Pose2D()
		nav_g_msg.x = self.x_g
		nav_g_msg.y = self.y_g
		nav_g_msg.theta = self.theta_g

		self.nav_goal_publisher.publish(nav_g_msg)

	def stay_idle(self):
		""" sends zero velocity to stay put """

		vel_g_msg = Twist()
		self.cmd_vel_publisher.publish(vel_g_msg)

	def close_to(self,x,y,theta):
		""" checks if the robot is at a pose within some threshold """

		return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

	def init_stop_sign(self):
		""" initiates a stop sign maneuver """

		self.stop_sign_start = rospy.get_rostime()
		self.mode = Mode.STOP

	def has_stopped(self):
		""" checks if stop sign maneuver is over """

		return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

	def init_crossing(self):
		""" initiates an intersection crossing maneuver """

		self.cross_start = rospy.get_rostime()
		self.mode = Mode.CROSS

	def has_crossed(self):
		""" checks if crossing maneuver is over """

		return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

	def stopped_for_food(self):
		""" stops for food pickup """
		self.stop4food_start = rospy.get_rostime()
		self.mode = Mode.WAIT4FOOD
	
	def has_stopped4food(self):
		""" checks if stopping for food is over """

		return (self.mode == Mode.WAIT4FOOD and (rospy.get_rostime()-self.self.stop4food_start)>rospy.Duration.from_sec(STOP4FOOD_TIME))
	  
	def set_nav_goal(self,x,y,theta):
		"changes the navigation goal pose based on State"
		self.x_g = x
		self.y_g = y
		self.theta_g = theta
	
	def get_next_label(self):
		"gets the index number of food_items that matches with label of first item in orderlist"
		for i in range(len(self.food_items)):
			if(label[self.orderlist[0]] == self.food_items[i][0]):
				print("Found "+ self.orderlist[0] + " in list")
				return i
			else:
				print("Food Item Requested Not in List")
	

	def loop(self):
		""" the main loop of the robot. At each iteration, depending on its
		mode (i.e. the finite state machine's state), if takes appropriate
		actions. This function shouldn't return anything """

		if not use_gazebo:
			try:
				origin_frame = "/map" if mapping else "/odom"
				(translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
				self.x = translation[0]
				self.y = translation[1]
				euler = tf.transformations.euler_from_quaternion(rotation)
				self.theta = euler[2]
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				pass
		
		if(self.last_state != self.state):
		  
		  if(self.state == State.PICKUP):
			first_label = self.get_next_label()
			self.set_nav_goal(self.food_items[next_food_label][1][0],self.food_items[next_food_label][1][1],0)	#go to the next food item
			self.mode = Mode.NAV
			print("Going to PICKUP state")
			
		  elif(self.state == State.GETORDER):
			self.set_nav_goal(0,0,0)
			self.mode = Mode.NAV
			print("Going to GETORDER state")
		
		# logs the current mode
		if not(self.last_mode_printed == self.mode):
			rospy.loginfo("Current Mode: %s", self.mode)
			self.last_mode_printed = self.mode

		# checks which mode it is in and acts accordingly
		if self.mode == Mode.IDLE:
			# send zero velocity
			self.stay_idle()

		elif self.mode == Mode.POSE:
			# moving towards a desired pose
			if self.close_to(self.x_g,self.y_g,self.theta_g):
				self.mode = Mode.IDLE
			else:
				self.go_to_pose()

		elif self.mode == Mode.STOP:
			# at a stop sign
			if self.has_stopped():
				self.init_crossing()
			else:
				self.stay_idle()

		elif self.mode == Mode.CROSS:
			# crossing an intersection
			if self.has_crossed():
				self.mode = Mode.NAV
			else:
				self.nav_to_pose()
		
		elif self.mode == Mode.WAIT4FOOD and self.state == Self.PICKUP:
			# at a stop sign
			if self.has_stopped4food():
				# self.food_items.remove(self.food_items[0])
				self.orderList.remove(self.orderList[0])
				next_food_label = self.get_next_label()
				self.set_nav_goal(self.food_items[next_food_label][1][0],self.food_items[next_food_label][1][1],0)	#go to the next food item
				self.mode = Mode.NAV
				if not orderList:
				  self.state = State.DELIVERY
				  print("Project Over, Going Home... \m/")
				  self.set_nav_goal(0,0,0)
			else:
				self.stay_idle()

		elif self.mode == Mode.NAV:
			if self.close_to(self.x_g,self.y_g,self.theta_g):
				if(self.state != State.PICKUP):
					self.mode = Mode.IDLE
				else:
					self.stopped_for_food()
			else:
				self.nav_to_pose()

		else:
			raise Exception('This mode is not supported: %s'
				% str(self.mode))
			
		self.last_state = self.state	#store last state to check for transition

	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__ == '__main__':
	sup = Supervisor()
	sup.run()