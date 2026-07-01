#pragma once

#include <QLabel>
#include <QPainter>
#include <QEvent>
#include <iostream>

#include <std_msgs/Float32.h>

namespace battery_widget {

class BatteryVisualization : public QLabel {
Q_OBJECT

public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  /**
   * @brief Initialize QLabel and set min/max size.
   * @param parent
   */
  BatteryVisualization(QWidget *parent);

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  /**
   * @brief Set the voltage limits for the linear mapping.
   * @param lower
   * @param upper
   */
  void setVoltageLimits(float lower, float upper);

protected:

  /***************************************************************************/
  /** Events                                                                **/
  /***************************************************************************/

  /**
   * @brief Draws the battery with the current #batteryLevel_. Can be called by
   * calling update().
   * @param event
   */
  void paintEvent(QPaintEvent *event);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  double batteryLevel_ = 1;

  float lowerVoltage_ = 39.0;
  float upperVoltage_ = 50.0;

  int batteryWidth_ = 100;
  int batteryHeight_ = 32;

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  /**
   * @brief Linear mapping of value #x.
   * @param x
   * @param inMin
   * @param inMax
   * @param outMin
   * @param outMax
   * @return
   */
  float map(float x, float inMin, float inMax, float outMin, float outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
  }

protected slots:

  /***************************************************************************/
  /** Slots                                                                 **/
  /***************************************************************************/

  /**
   * @brief Computes the #batteryLevel_ with a linear mapping from the
   * #batteryState and constructs a tool tip with the battery voltage.
   * @param batteryState
   */
  void updateBatteryStateSimple(std_msgs::Float32 batteryState);

  /**
   * @brief Sets the #batteryLevel_ and constructs a tool tip with the message
   * and battery #voltage.
   * @param voltage
   * @param charge
   * @param message
   */
  void updateBatteryState(double voltage, double charge, std::string message);
};

} // namespace
