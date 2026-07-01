#!/usr/bin/env python3
from __future__ import division
# Qt
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QTreeWidgetItem, QWidget
# ROS
import roslib
import roslib.message
import roslib.names
import rospkg
import rospy
import rostopic
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
# Others
import os

arm_start_pub = rospy.Publisher('/ask/arm/go_bool', Bool, queue_size=1)
dog_start_pub = rospy.Publisher('/ask/dog/start', Bool, queue_size=1)
dog_walk_pub = rospy.Publisher('/ask/dog/walk', Bool, queue_size=1)
dog_stand_pub = rospy.Publisher('/ask/dog/stand', Bool, queue_size=1)
dog_left_right_pub = rospy.Publisher('/ask/dog/left_right', Float32, queue_size=1)
dog_forward_back_pub = rospy.Publisher('/ask/dog/forward_back', Float32, queue_size=1)
dog_yaw_pub = rospy.Publisher('/ask/dog/yaw', Float32, queue_size=1)


class RobotWidget(QWidget):

    def __init__(self):
        super(RobotWidget,self).__init__()
        # read UI file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_gui'), 'resource', 'RobotGui.ui')
        loadUi(ui_file, self)
        self.setObjectName('RobotPluginUi')
        # connect widget with slot function
        # self.open_pushButton.clicked.connect(self.open_button_slot)
        self.ArmStartBtn.clicked.connect(self.arm_start_button_slot)
        # self.ArmStateBtn.clicked.connect(self.arm_state_button_slot)
        self.DogStateBtn.clicked.connect(self.dog_state_button_slot)
        self.DogStartBtn.clicked.connect(self.dog_start_button_slot)
        self.DogWalkBtn.clicked.connect(self.dog_walk_button_slot)
        self.DogStandBtn.clicked.connect(self.dog_stand_button_slot)
        self.DogLeftBtn.setAutoRepeat(True)
        self.DogLeftBtn.setAutoRepeatInterval(50)
        self.DogRightBtn.setAutoRepeat(True)
        self.DogRightBtn.setAutoRepeatInterval(50)
        self.DogForwardBtn.setAutoRepeat(True)
        self.DogForwardBtn.setAutoRepeatInterval(50)
        self.DogBackBtn.setAutoRepeat(True)
        self.DogBackBtn.setAutoRepeatInterval(50)
        self.DogLeftBtn.clicked.connect(self.dog_left_button_slot)
        self.DogRightBtn.clicked.connect(self.dog_right_button_slot)
        self.DogForwardBtn.clicked.connect(self.dog_forward_button_slot)
        self.DogBackBtn.clicked.connect(self.dog_back_button_slot)
        self.forward_val.setRange(0.00, 1.00)
        self.back_val.setRange(0.00, 1.00)
        self.left_val.setRange(0.00, 1.00)
        self.right_val.setRange(0.00, 1.00)
        # 转弯按钮
        self.DogTurnLeftBtn.setAutoRepeat(True)
        self.DogTurnLeftBtn.setAutoRepeatInterval(50)
        self.DogTurnRightBtn.setAutoRepeat(True)
        self.DogTurnRightBtn.setAutoRepeatInterval(50)
        self.DogTurnLeftBtn.clicked.connect(self.dog_turn_left_button_slot)
        self.DogTurnRightBtn.clicked.connect(self.dog_turn_right_button_slot)
        self.turn_left_val.setRange(0.00, 1.00)
        self.turn_right_val.setRange(0.00, 1.00)




    # @Slot()  # 按钮的回调函数
    def dog_left_button_slot(self):

        left = self.left_val.value()
        msg = Float32()
        msg.data = left
        dog_left_right_pub.publish(msg)


    def dog_right_button_slot(self):

        right = self.right_val.value()
        msg = Float32()
        msg.data = -right
        dog_left_right_pub.publish(msg)


    def dog_forward_button_slot(self):

        forward = self.forward_val.value()
        msg = Float32()
        msg.data = forward
        dog_forward_back_pub.publish(msg)


    def dog_back_button_slot(self):

        back = self.back_val.value()
        msg = Float32()
        msg.data = -back
        dog_forward_back_pub.publish(msg)


    def dog_turn_left_button_slot(self):
        turn_left = self.turn_left_val.value()
        msg = Float32()
        msg.data = turn_left
        dog_yaw_pub.publish(msg)


    def dog_turn_right_button_slot(self):
        turn_right = self.turn_right_val.value()
        msg = Float32()
        msg.data = -turn_right
        dog_yaw_pub.publish(msg)


    def arm_start_button_slot(self):

        msg = Bool()
        msg.data = True
        arm_start_pub.publish(msg)


    def dog_start_button_slot(self):

        msg = Bool()
        msg.data = True
        dog_start_pub.publish(msg)

    def dog_walk_button_slot(self):
        msg = Bool()
        msg.data = True
        dog_walk_pub.publish(msg)


    def dog_stand_button_slot(self):
        msg = Bool()
        msg.data = True
        dog_stand_pub.publish(msg)


    def dog_state_button_slot(self):

        topic_type, real_topic, fields = rostopic.get_topic_type("/mydog/joint_states")
        data_class = roslib.message.get_message_class(topic_type)
        rospy.wait_for_message(real_topic, data_class)
        self.dog_joint_sub = rospy.Subscriber(real_topic, data_class, self.dog_joint_callback)



    def dog_joint_callback(self, msg):

        self.degree1.setText(str(msg.position[0]))
        self.degree2.setText(str(msg.position[1]))
        self.degree3.setText(str(msg.position[2]))
        self.degree4.setText(str(msg.position[3]))
        self.degree5.setText(str(msg.position[4]))
        self.degree6.setText(str(msg.position[5]))
        self.degree7.setText(str(msg.position[6]))
        self.degree8.setText(str(msg.position[7]))
        self.degree9.setText(str(msg.position[8]))
        self.degree10.setText(str(msg.position[9]))
        self.degree11.setText(str(msg.position[10]))
        self.degree12.setText(str(msg.position[11]))

        self.velocity1.setText(str(msg.velocity[0]))
        self.velocity2.setText(str(msg.velocity[1]))
        self.velocity3.setText(str(msg.velocity[2]))
        self.velocity4.setText(str(msg.velocity[3]))
        self.velocity5.setText(str(msg.velocity[4]))
        self.velocity6.setText(str(msg.velocity[5]))
        self.velocity7.setText(str(msg.velocity[6]))
        self.velocity8.setText(str(msg.velocity[7]))
        self.velocity9.setText(str(msg.velocity[8]))
        self.velocity10.setText(str(msg.velocity[9]))
        self.velocity11.setText(str(msg.velocity[10]))
        self.velocity12.setText(str(msg.velocity[11]))

    def close_plugin(self):

        self.dog_joint_sub.unregister()




    # def open_button_slot(self):
    #     topic_type, real_topic, fields = rostopic.get_topic_type("your_topic_name")
    #     data_class = roslib.message.get_message_class(topic_type)
    #     # 订阅了一个话题
    #     self.mysub = rospy.Subscriber(real_topic, data_class, self.my_callback)
	#
	# # 话题的回调函数
    # def my_callback(self, msg):
    #     x = 1
	#
	# # 关闭插件时, 注销订阅的话题
    # def close_plugin(self):
    #    try:
    #         self.mysub.unregister()
    #     except AttributeError as e:
    #         rospy.logerr("Subscriber doesn't open.")

