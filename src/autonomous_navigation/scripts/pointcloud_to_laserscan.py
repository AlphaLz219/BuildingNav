#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PointCloud2 to LaserScan converter for Mid360 LiDAR
Converts 3D point cloud from Mid360 to 2D laser scan for GMapping
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2
import tf.transformations as tf_trans


class PointCloudToLaserScan:
    def __init__(self):
        rospy.init_node('pointcloud_to_laserscan_node', anonymous=True)
        
        # Parameters
        self.scan_angle_min = rospy.get_param('~angle_min', -np.pi)
        self.scan_angle_max = rospy.get_param('~angle_max', np.pi)
        self.scan_angle_increment = rospy.get_param('~angle_increment', np.deg2rad(1.0))
        self.scan_range_min = rospy.get_param('~range_min', 0.1)
        self.scan_range_max = rospy.get_param('~range_max', 30.0)
        self.scan_height = rospy.get_param('~scan_height', 0.1)  # Height tolerance in meters
        self.target_frame = rospy.get_param('~target_frame', 'base_link')
        
        # Calculate number of scan readings
        self.scan_num_readings = int((self.scan_angle_max - self.scan_angle_min) / 
                                      self.scan_angle_increment)
        
        # Publisher and subscriber
        self.scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)
        self.pc_sub = rospy.Subscriber('/scan', PointCloud2, self.pointcloud_callback, 
                                        queue_size=10)
        
        self.tf_listener = None
        try:
            import tf
            self.tf_listener = tf.TransformListener()
        except ImportError:
            rospy.logwarn("tf package not available, skipping transform")
        
        rospy.loginfo("PointCloudToLaserScan node initialized")
        rospy.loginfo(f"Scan range: [{self.scan_angle_min:.2f}, {self.scan_angle_max:.2f}] rad")
        rospy.loginfo(f"Angle increment: {np.rad2deg(self.scan_angle_increment):.2f} deg")
        rospy.loginfo(f"Number of readings: {self.scan_num_readings}")
    
    def pointcloud_callback(self, pc_msg):
        """Convert PointCloud2 to LaserScan"""
        try:
            # Transform point cloud to target frame if tf available
            if self.tf_listener is not None:
                try:
                    self.tf_listener.waitForTransform(
                        self.target_frame, 
                        pc_msg.header.frame_id, 
                        pc_msg.header.stamp, 
                        rospy.Duration(0.5)
                    )
                    pc_msg = self.tf_listener.transformPointCloud(
                        self.target_frame, 
                        pc_msg
                    )
                except (tf.Exception, rospy.ROSException) as e:
                    rospy.logwarn_throttle(5.0, f"Transform failed: {e}")
            
            # Extract points from PointCloud2
            points = list(point_cloud2.read_points(pc_msg, skip_nans=True))
            
            if len(points) == 0:
                rospy.logwarn_throttle(5.0, "Empty point cloud received")
                return
            
            # Initialize ranges array
            ranges = [float('inf')] * self.scan_num_readings
            
            # Process each point
            for point in points:
                x, y, z = point[0], point[1], point[2]
                
                # Filter by height (keep points near ground plane for 2D SLAM)
                if abs(z) > self.scan_height:
                    continue
                
                # Convert to polar coordinates
                range_ = np.sqrt(x*x + y*y)
                angle = np.arctan2(y, x)
                
                # Skip points outside scan range
                if range_ < self.scan_range_min or range_ > self.scan_range_max:
                    continue
                
                # Normalize angle to [-pi, pi]
                while angle > np.pi:
                    angle -= 2 * np.pi
                while angle < -np.pi:
                    angle += 2 * np.pi
                
                # Find corresponding scan index
                if angle >= self.scan_angle_min and angle <= self.scan_angle_max:
                    index = int((angle - self.scan_angle_min) / self.scan_angle_increment)
                    if 0 <= index < self.scan_num_readings:
                        # Keep minimum range for each angle bin
                        if range_ < ranges[index]:
                            ranges[index] = range_
            
            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header = pc_msg.header
            scan_msg.header.frame_id = self.target_frame
            scan_msg.angle_min = self.scan_angle_min
            scan_msg.angle_max = self.scan_angle_max
            scan_msg.angle_increment = self.scan_angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = self.scan_range_min
            scan_msg.range_max = self.scan_range_max
            scan_msg.ranges = ranges
            scan_msg.intensities = []
            
            # Publish scan
            self.scan_pub.publish(scan_msg)
            
        except Exception as e:
            rospy.logerr(f"Error converting point cloud: {e}")
    
    def run(self):
        """Run the node"""
        rospy.spin()


if __name__ == '__main__':
    try:
        converter = PointCloudToLaserScan()
        converter.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Node error: {e}")
