#pragma once

#include <battery_widget/BatteryVisualization.h>
#include <battery_widget/ui_BatteryWidget.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/builtin_double.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>

#include <QWidget>
#include <QLabel>
#include <QMovie>

namespace battery_widget {

class BatteryWidget : public QWidget {
Q_OBJECT

public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  /**
   * @brief Initialize the ui.
   * @param parent
   */
  BatteryWidget(QWidget *parent)
      : QWidget(parent) {
    ui_.setupUi(this);
  }

  /**
   * @brief Shutdown the subscribers.
   */
  ~BatteryWidget() {
    sub_.shutdown();
    subSimple_.shutdown();
  }

  /***************************************************************************/
  /** Initialize                                                            **/
  /***************************************************************************/

  /**
   * @brief Initialize the widget to use sensor_msgs/BatteryState subscriber.
   * This way the widget updates itself.
   * @param nodeHandle
   * @param topic
   * @param isBmsState
   */
  void initWidget(ros::NodeHandle nodeHandle, std::string topic);

  /**
   * @brief Initialize the widget to use a simple Float32 subscriber. This way
   * the widget updates itself.
   * @param nodeHandle
   * @param topic
   * @param lowerLimit
   * @param upperLimit
   */
  void initWidget(ros::NodeHandle nodeHandle, std::string topic,
                  float lowerLimit, float upperLimit);

  /**
   * @brief Initialize the widget to use with setBatteryState() accessor. This
   * way the widget has to be updated from the parent widget.
   */
  void initWidget();

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  /**
   * @brief Set a new battery state. Works only if the widget was initialized
   * without subscribers.
   * @param voltage
   * @param charge
   * @param message
   */
  void setBatteryState(double voltage, double charge, std::string message);

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void updateBatteryStateSimple(std_msgs::Float32 batteryState);

  void updateBatteryState(double voltage, double charge, std::string message);

protected:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  Ui::BatteryStateWidget ui_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Subscriber subSimple_;

  /***************************************************************************/
  /** Callbacks                                                             **/
  /***************************************************************************/

  /**
   * @brief ROS subscriber callback to update the battery state via
   * sensor_msgs/BatteryState state.
   * @param msg
   */
  void batteryStateCallback(const sensor_msgs::BatteryStateConstPtr &msg);

  /**
   * @brief ROS subscriber callback to update the battery state via a simple
   * Float32 voltage value.
   * @param msg
   */
  void batteryStateSimpleCallback(const std_msgs::Float32ConstPtr &msg);
};

} // namespace
