#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Keyboard teleoperation script for wheeled robot.
Control with WSAD keys:
  W - Move forward
  S - Move backward
  A - Turn left
  D - Turn right
  Q - Stop and exit
"""

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Control parameters
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 1.0

# Key bindings
KEYS = {
    'w': (1, 0),   # Forward
    's': (-1, 0),  # Backward
    'a': (0, 1),   # Turn left
    'd': (0, -1),  # Turn right
    'q': (0, 0),   # Stop and quit
}

def get_key():
    """Get a single key press from terminal."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_instructions():
    """Print control instructions."""
    print("=" * 50)
    print("Wheeled Robot Keyboard Control")
    print("=" * 50)
    print("W - Move forward")
    print("S - Move backward")
    print("A - Turn left")
    print("D - Turn right")
    print("Q - Stop and exit")
    print("=" * 50)

if __name__ == "__main__":
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    # Initialize ROS node
    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    print_instructions()
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key in KEYS:
                linear, angular = KEYS[key]
                
                twist = Twist()
                twist.linear.x = linear * LINEAR_SPEED
                twist.angular.z = angular * ANGULAR_SPEED
                
                pub.publish(twist)
                
                if key == 'q':
                    print("\nStopping robot and exiting...")
                    break
                
                # Print current command
                cmd_str = []
                if linear > 0:
                    cmd_str.append("Forward")
                elif linear < 0:
                    cmd_str.append("Backward")
                if angular > 0:
                    cmd_str.append("Left")
                elif angular < 0:
                    cmd_str.append("Right")
                
                if cmd_str:
                    print(f"Command: {' '.join(cmd_str)}")
            
            rospy.sleep(0.1)
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # Stop the robot
        twist = Twist()
        pub.publish(twist)
        print("Robot stopped.")
