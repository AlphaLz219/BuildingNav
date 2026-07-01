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


dog_start_pub = rospy.Publisher('/ask/dog/start', Bool, queue_size=1)
dog_walk_pub = rospy.Publisher('/ask/dog/walk', Bool, queue_size=1)
dog_stand_pub = rospy.Publisher('/ask/dog/stand', Bool, queue_size=1)
dog_left_right_pub = rospy.Publisher('/ask/dog/left_right', Float32, queue_size=1)
dog_forward_back_pub = rospy.Publisher('/ask/dog/forward_back', Float32, queue_size=1)
dog_yaw_pub = rospy.Publisher('/ask/dog/yaw', Float32, queue_size=1)


class DogWidget(QWidget):

    def __init__(self):
        super(DogWidget,self).__init__()
        # read UI file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_dog_gui'), 'resource', 'DogGui.ui')
        loadUi(ui_file, self)
        self.setObjectName('DogPluginUi')
        # connect widget with slot function
        # self.open_pushButton.clicked.connect(self.open_button_slot)

        # self.ArmStateBtn.clicked.connect(self.arm_state_button_slot)
        self.DogStateShow.clicked.connect(self.show_state_button_slot)
        self.DogStateHide.clicked.connect(self.hide_state_button_slot)
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


    def show_state_button_slot(self):

        topic_type, real_topic, fields = rostopic.get_topic_type("/mydog/joint_states")
        data_class = roslib.message.get_message_class(topic_type)
        #rospy.wait_for_message(real_topic, data_class)
        self.dog_joint_sub = rospy.Subscriber(real_topic, data_class, self.dog_joint_callback)



    def dog_joint_callback(self, msg):

        # self.degree1.setText(str(msg.position[0]).split('.')[0] + '.' + str(msg.position[0]).split('.')[1][:2])
        # self.degree2.setText(str(msg.position[1]).split('.')[0] + '.' + str(msg.position[1]).split('.')[1][:2])
        # self.degree3.setText(str(msg.position[2]).split('.')[0] + '.' + str(msg.position[2]).split('.')[1][:2])
        # self.degree4.setText(str(msg.position[3]).split('.')[0] + '.' + str(msg.position[3]).split('.')[1][:2])
        # self.degree5.setText(str(msg.position[4]).split('.')[0] + '.' + str(msg.position[4]).split('.')[1][:2])
        # self.degree6.setText(str(msg.position[5]).split('.')[0] + '.' + str(msg.position[5]).split('.')[1][:2])
        # self.degree7.setText(str(msg.position[6]).split('.')[0] + '.' + str(msg.position[6]).split('.')[1][:2])
        # self.degree8.setText(str(msg.position[7]).split('.')[0] + '.' + str(msg.position[7]).split('.')[1][:2])
        # self.degree9.setText(str(msg.position[8]).split('.')[0] + '.' + str(msg.position[8]).split('.')[1][:2])
        # self.degree10.setText(str(msg.position[9]).split('.')[0] + '.' + str(msg.position[9]).split('.')[1][:2])
        # self.degree11.setText(str(msg.position[10]).split('.')[0] + '.' + str(msg.position[10]).split('.')[1][:2])
        # self.degree12.setText(str(msg.position[11]).split('.')[0] + '.' + str(msg.position[11]).split('.')[1][:2])
        #
        # self.velocity1.setText(str(msg.velocity[0]).split('.')[0] + '.' + str(msg.velocity[0]).split('.')[1][:2])
        # self.velocity2.setText(str(msg.velocity[1]).split('.')[0] + '.' + str(msg.velocity[1]).split('.')[1][:2])
        # self.velocity3.setText(str(msg.velocity[2]).split('.')[0] + '.' + str(msg.velocity[2]).split('.')[1][:2])
        # self.velocity4.setText(str(msg.velocity[3]).split('.')[0] + '.' + str(msg.velocity[3]).split('.')[1][:2])
        # self.velocity5.setText(str(msg.velocity[4]).split('.')[0] + '.' + str(msg.velocity[4]).split('.')[1][:2])
        # self.velocity6.setText(str(msg.velocity[5]).split('.')[0] + '.' + str(msg.velocity[5]).split('.')[1][:2])
        # self.velocity7.setText(str(msg.velocity[6]).split('.')[0] + '.' + str(msg.velocity[6]).split('.')[1][:2])
        # self.velocity8.setText(str(msg.velocity[7]).split('.')[0] + '.' + str(msg.velocity[7]).split('.')[1][:2])
        # self.velocity9.setText(str(msg.velocity[8]).split('.')[0] + '.' + str(msg.velocity[8]).split('.')[1][:2])
        # self.velocity10.setText(str(msg.velocity[9]).split('.')[0] + '.' + str(msg.velocity[9]).split('.')[1][:2])
        # self.velocity11.setText(str(msg.velocity[10]).split('.')[0] + '.' + str(msg.velocity[10]).split('.')[1][:2])
        # self.velocity12.setText(str(msg.velocity[11]).split('.')[0] + '.' + str(msg.velocity[11]).split('.')[1][:2])

        self.degree1.setText(str(round(msg.position[0], 2)))
        self.degree2.setText(str(round(msg.position[1], 2)))
        self.degree3.setText(str(round(msg.position[2], 2)))
        self.degree4.setText(str(round(msg.position[3], 2)))
        self.degree5.setText(str(round(msg.position[4], 2)))
        self.degree6.setText(str(round(msg.position[5], 2)))
        self.degree7.setText(str(round(msg.position[6], 2)))
        self.degree8.setText(str(round(msg.position[7], 2)))
        self.degree9.setText(str(round(msg.position[8], 2)))
        self.degree10.setText(str(round(msg.position[9], 2)))
        self.degree11.setText(str(round(msg.position[10], 2)))
        self.degree12.setText(str(round(msg.position[11], 2)))

        self.velocity1.setText(str(round(msg.velocity[0], 2)))
        self.velocity2.setText(str(round(msg.velocity[1], 2)))
        self.velocity3.setText(str(round(msg.velocity[2], 2)))
        self.velocity4.setText(str(round(msg.velocity[3], 2)))
        self.velocity5.setText(str(round(msg.velocity[4], 2)))
        self.velocity6.setText(str(round(msg.velocity[5], 2)))
        self.velocity7.setText(str(round(msg.velocity[6], 2)))
        self.velocity8.setText(str(round(msg.velocity[7], 2)))
        self.velocity9.setText(str(round(msg.velocity[8], 2)))
        self.velocity10.setText(str(round(msg.velocity[9], 2)))
        self.velocity11.setText(str(round(msg.velocity[10], 2)))
        self.velocity12.setText(str(round(msg.velocity[11], 2)))

    def hide_state_button_slot(self):
        self.dog_joint_sub.unregister()
        
        self.degree1.clear()
        self.degree2.clear()
        self.degree3.clear()
        self.degree4.clear()
        self.degree5.clear()
        self.degree6.clear()
        self.degree7.clear()
        self.degree8.clear()
        self.degree9.clear()
        self.degree10.clear()
        self.degree11.clear()
        self.degree12.clear()

        self.velocity1.clear()
        self.velocity2.clear()
        self.velocity3.clear()
        self.velocity4.clear()
        self.velocity5.clear()
        self.velocity6.clear()
        self.velocity7.clear()
        self.velocity8.clear()
        self.velocity9.clear()
        self.velocity10.clear()
        self.velocity11.clear()
        self.velocity12.clear()




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

