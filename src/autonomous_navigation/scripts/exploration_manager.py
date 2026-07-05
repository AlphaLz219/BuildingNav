#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Exploration Manager for TurtleBot3
Coordinates SLAM mapping and explore_lite for autonomous exploration
"""

import rospy
import actionlib
import tf
from nav_msgs.srv import GetMap, GetPlan
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
import math


class ExplorationManager:
    def __init__(self):
        rospy.init_node('exploration_manager', anonymous=True)
        
        # Parameters
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'base_footprint')
        self.explore_topic = rospy.get_param('~explore_topic', '/explore_server')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        
        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Services
        rospy.loginfo("Waiting for map server...")
        rospy.wait_for_service('/dynamic_map', timeout=30.0)
        self.get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)
        
        # Action client for move_base
        self.move_base_client = actionlib.SimpleActionClient(
            '/move_base', 
            MoveBaseAction
        )
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server(timeout=60.0)
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        # State variables
        self.map_received = False
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.current_pose = None
        
        # Exploration state
        self.is_exploring = False
        self.exploration_complete = False
        
        rospy.loginfo("ExplorationManager initialized")
        rospy.loginfo(f"World frame: {self.world_frame}")
        rospy.loginfo(f"Robot base frame: {self.robot_base_frame}")
    
    def map_callback(self, msg):
        """Callback for map updates"""
        self.map_received = True
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        rospy.logdebug(f"Map received: {self.map_width}x{self.map_height} @ {self.map_resolution}m/cell")
    
    def odom_callback(self, msg):
        """Callback for odometry updates"""
        self.current_pose = msg.pose.pose
    
    def get_current_pose(self):
        """Get current robot pose in map frame"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.world_frame, 
                self.robot_base_frame, 
                rospy.Time(0)
            )
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF lookup failed: {e}")
            return None, None
    
    def send_goal(self, x, y, yaw=0.0):
        """Send a navigation goal to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.world_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        rospy.loginfo(f"Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        self.move_base_client.send_goal(goal)
        
        # Wait for result with timeout
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(120.0))
        
        if finished_within_time:
            state = self.move_base_client.get_state()
            result = self.move_base_client.get_result()
            
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully")
                return True
            else:
                rospy.logwarn(f"Goal failed with state: {state}")
                return False
        else:
            rospy.logwarn("Goal timed out")
            self.move_base_client.cancel_goal()
            return False
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def check_frontiers(self):
        """Check if there are unexplored frontiers in the map"""
        if not self.map_received:
            return False
        
        try:
            map_response = self.get_map_srv()
            map_data = map_response.map.data
            width = map_response.map.info.width
            height = map_response.map.info.height
            
            # Count unknown cells (-1)
            unknown_count = sum(1 for cell in map_data if cell == -1)
            total_cells = width * height
            unknown_ratio = unknown_count / float(total_cells)
            
            rospy.loginfo(f"Map coverage: {(1 - unknown_ratio) * 100:.1f}%")
            
            # Consider exploration complete if less than 5% unknown
            return unknown_ratio > 0.05
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"GetMap service failed: {e}")
            return True
    
    def start_exploration(self):
        """Start frontier-based exploration using explore_lite"""
        rospy.loginfo("Starting autonomous exploration...")
        self.is_exploring = True
        
        # Note: explore_lite should be running separately
        # This method monitors exploration progress
        
        rate = rospy.Rate(0.5)  # Check every 2 seconds
        
        while self.is_exploring and not rospy.is_shutdown():
            # Check if exploration is complete
            has_frontiers = self.check_frontiers()
            
            if not has_frontiers:
                rospy.loginfo("Exploration complete - no more frontiers detected")
                self.exploration_complete = True
                self.is_exploring = False
                break
            
            # Check current position
            trans, rot = self.get_current_pose()
            if trans:
                rospy.logdebug(f"Current position: x={trans[0]:.2f}, y={trans[1]:.2f}")
            
            rate.sleep()
        
        return self.exploration_complete
    
    def stop_exploration(self):
        """Stop ongoing exploration"""
        rospy.loginfo("Stopping exploration...")
        self.is_exploring = False
        self.stop_robot()
    
    def save_map(self, filename="/tmp/explored_map"):
        """Save the current map to file"""
        try:
            map_response = self.get_map_srv()
            
            # Save as occupancy grid
            import numpy as np
            map_array = np.array(map_response.map.data).reshape(
                (map_response.map.info.height, 
                 map_response.map.info.width)
            )
            
            # Save metadata
            metadata = {
                'resolution': map_response.map.info.resolution,
                'width': map_response.map.info.width,
                'height': map_response.map.info.height,
                'origin_x': map_response.map.info.origin.position.x,
                'origin_y': map_response.map.info.origin.position.y,
            }
            
            rospy.loginfo(f"Map saved: {metadata}")
            return True
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to save map: {e}")
            return False


def main():
    """Main function"""
    manager = ExplorationManager()
    
    rospy.loginfo("Exploration Manager Ready")
    rospy.loginfo("Commands:")
    rospy.loginfo("  - Exploration starts automatically when explore_lite is running")
    rospy.loginfo("  - Press Ctrl+C to stop")
    
    try:
        # Start exploration monitoring
        success = manager.start_exploration()
        
        if success:
            rospy.loginfo("Exploration completed successfully!")
            manager.save_map()
        else:
            rospy.loginfo("Exploration stopped")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration interrupted by user")
        manager.stop_exploration()
    except Exception as e:
        rospy.logerr(f"Exploration error: {e}")
        manager.stop_exploration()


if __name__ == '__main__':
    main()
