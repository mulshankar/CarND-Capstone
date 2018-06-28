#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import numpy as np
'''
This node will waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
MAX_DECEL=0.5 # setting some deceleration rate for the car

class WaypointUpdater(object):
	def __init__(self):

		rospy.init_node('waypoint_updater')

		# All subscribers are here
		
		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
		
		# Way point publisher

		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# Other member variables you need below

		self.pose = None
		self.base_lane = None
		self.waypoints_2d = None
		self.waypoint_tree = None
		self.loop()

    def loop(self):
        rate = rospy.Rate(40) # 40 Hz loop.. waypoint follower running at 30 Hz.. needs to run at least as fast 
        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg
        
    def waypoints_cb(self, waypoints): #setting up KDTree with waypoints
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d) # setting up KDTree to identify nearest neighbour - log(n) vs N operation

    def get_closest_waypoint_id(self):
        
        x = self.pose.pose.position.x
        y = self.pose.pose.position.x
        
        closest_idx = self.waypoint_tree.query([x, y],1)[1] # returns distance and index.. just getting index of coordinate

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
		
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
		final_lane=self.generate_Lane()
		self.final_waypoints_pub.publish(final_lane)

    def generate_Lane(self):

        lane=Lane() # create lane object
        closest_idx=self.get_closest_waypoint_id() # get closest way point index
        farthest_idx=closest_idx+LOOKAHEAD_WPS # calculate farthest index 
        base_waypoints=self.base_lane.waypoints[closest_idx:farthest_idx] # python slicing automatically takes care of end of list at end of way-points

        if self.stopline_wp_idx==-1 or (self.stopline_wp_idx>=farthest_idx): # stopline way point index comes from the traffic lights subscriber
            lane.waypoints=base_waypoints
        else:
            lane.waypoints=self.decelerate_waypoints(base_waypoints,closest_idx) # need to slow down since there is a traffic light that within the vicinity
            
        return lane

    def decelerate_waypoints(self,waypoints,closest_idx): # deceleration logic

        temp=[]

        for i,wp in enumerate(waypoints):
            p=Waypoint() #instantiate a waypoint object
            p.pose=wp.pose

            stop_idx=max(self.stopline_wp_idx - closest_idx -2, 0) # subtract two way-points to ensure nose of car stops at line
            dist=self.distance(waypoints,i,stop_idx) # calculate distance from current index to stop line index
            vel=2.0*MAX_DECEL*dist # some deceleration rate based on distance
            if vel<1.0:
                vel=0.0
            p.twist.twist.linear.x=min(vel,wp.twist.twist.linear.x) #set way point velocity to min of calculated vs speed limit (default value)
            temp.append(p)

        return temp        

    def traffic_cb(self, msg):
        self.stopline_wp_idx=msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
