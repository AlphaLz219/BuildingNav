#!/usr/bin/env python3
import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from rqt_robot_gui.robot_gui_widget import RobotWidget

class RobotPlugin(Plugin):
    def __init__(self, context):
        super(RobotPlugin, self).__init__(context)
        self.setObjectName('RobotPlugin')
        self._widget = RobotWidget()  # 定义GUI Widget类
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)  # Add widget to the user interface

    def shutdown_plugin(self):
        self._widget.close_plugin()  # 当关闭插件时, 调用这个函数, 通常是注销一些回调函数
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass
        
    def restore_settings(self, plugin_settings, instance_settings):
        pass
