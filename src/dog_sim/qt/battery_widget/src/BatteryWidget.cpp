#include "battery_widget/BatteryWidget.h"

namespace battery_widget {

/*****************************************************************************/
/** Initialize                                                              **/
/*****************************************************************************/

void BatteryWidget::initWidget(ros::NodeHandle nodeHandle, std::string topic) {
  nh_ = nodeHandle;

  connect(this, SIGNAL(updateBatteryState(double, double, std::string)),
          ui_.Battery, SLOT(updateBatteryState(double, double, std::string)));
  sub_ = nh_.subscribe<sensor_msgs::BatteryState>(
      topic, 1, &BatteryWidget::batteryStateCallback, this);
  qRegisterMetaType<std::string>("std::string");
}

void BatteryWidget::initWidget(ros::NodeHandle nodeHandle, std::string topic,
                               float lowerLimit, float upperLimit) {
  nh_ = nodeHandle;

  connect(this, SIGNAL(updateBatteryStateSimple(std_msgs::Float32)),
          ui_.Battery, SLOT(updateBatteryStateSimple(std_msgs::Float32)));
  subSimple_ = nh_.subscribe<std_msgs::Float32>(
      topic, 1, &BatteryWidget::batteryStateSimpleCallback, this);
  qRegisterMetaType<std_msgs::Float32>("std_msgs::Float32");

  ui_.Battery->setVoltageLimits(lowerLimit, upperLimit);
}

void BatteryWidget::initWidget() {
  connect(this, SIGNAL(updateBatteryState(double, double, std::string)),
          ui_.Battery, SLOT(updateBatteryState(double, double, std::string)));
  qRegisterMetaType<std::string>("std::string");
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void BatteryWidget::setBatteryState(double voltage, double charge,
                                    std::string message) {
  emit updateBatteryState(voltage, charge, message);
}

/*****************************************************************************/
/** Callbacks                                                               **/
/*****************************************************************************/

void BatteryWidget::batteryStateCallback(
    const sensor_msgs::BatteryStateConstPtr &msg) {
  emit updateBatteryState((double)msg->voltage, (double)msg->percentage, "");
}

void BatteryWidget::batteryStateSimpleCallback(
    const std_msgs::Float32ConstPtr &msg) {
  emit updateBatteryStateSimple(*msg);
}

} // namespace
