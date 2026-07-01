#include "battery_widget/BatteryVisualization.h"

namespace battery_widget {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

BatteryVisualization::BatteryVisualization(QWidget *parent)
    : QLabel(parent) {

  this->setMinimumSize(batteryWidth_ + 20, batteryHeight_ + 10);
  this->setMaximumSize(batteryWidth_ + 20, batteryHeight_ + 10);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void BatteryVisualization::setVoltageLimits(float lower, float upper) {
  lowerVoltage_ = lower;
  upperVoltage_ = upper;
}

/*****************************************************************************/
/** Events                                                                  **/
/*****************************************************************************/

void BatteryVisualization::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  QPen blackPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  QPen greenPen(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  QPen redPen(Qt::red, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  QRectF rectangleMain(1, 1, batteryWidth_, batteryHeight_);
  QRectF rectangleHead(batteryWidth_ + 2, 11, 3, 11);
  QRectF rectangleState(1, 1, batteryLevel_ * batteryWidth_, batteryHeight_);

  QPainterPath pathBorder;
  pathBorder.addRoundedRect(rectangleMain, 5, 5);

  if (batteryLevel_ < 0.20) {
    painter.fillRect(rectangleState, QColor(255, 0, 0, 255));
  } else if (batteryLevel_ < 0.35) {
    painter.fillRect(rectangleState, QColor(255, 120, 0, 255));
  } else {
    painter.fillRect(rectangleState, QColor(17, 171, 30, 255));
  }

  painter.setPen(blackPen);
  painter.drawPath(pathBorder);
  painter.drawRect(rectangleHead);
  painter.fillRect(rectangleHead, Qt::black);

  painter.setFont(QFont("Arial", 11));
  QString strBatteryState = QString::number(100.0 * batteryLevel_, 'f', 0) +"%";
  painter.drawText(rectangleMain, Qt::AlignCenter, strBatteryState);

  painter.end();
}

/*****************************************************************************/
/** Slots                                                                   **/
/*****************************************************************************/

void BatteryVisualization::updateBatteryStateSimple(
    std_msgs::Float32 batteryState) {
  if (batteryState.data < lowerVoltage_) {
    batteryLevel_ = 0.0;
  } else if (batteryState.data > upperVoltage_) {
    batteryLevel_ = 1.0;
  } else {
    batteryLevel_ = map(batteryState.data, lowerVoltage_,
                        upperVoltage_, 0.0, 1.0);
  }

  QString batteryString;
  batteryString
      .append("Battery voltage: \t")
      .append(QString::number(batteryState.data, 'f', 2))
      .append("V\n");
  this->setToolTip(batteryString);

  update();
}

void BatteryVisualization::updateBatteryState(double voltage, double charge,
                                              std::string message) {
  if (charge < 0.0) {
    batteryLevel_ = 0.0;
  } else if (charge > 1.0) {
    batteryLevel_ = 1.0;
  } else {
    batteryLevel_ = charge;
  }

  QString batteryString;
  if (message.size() > 0) {
    batteryString.append(QString::fromStdString(message + "\n"));
  }
  batteryString
      .append("Battery voltage: \t")
      .append(QString::number(voltage, 'f', 2))
      .append("V");
  this->setToolTip(batteryString);

  update();
}

} // namespace
