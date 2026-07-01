#include <cmath>

#include "joystick_label/Joystick.hpp"

namespace joystick_label {

Joystick::Joystick(QWidget* parent) : QLabel(parent) {
  this->installEventFilter(this);

  this->setToolTip(
      "<p>Shift+Mouse: snap to lines</p>"
      "<p>Double click to reset</p>");

  this->setText("");
  this->setScaledContents(true);
  this->setPixmap(QPixmap(":/images/background.png"));
  this->setFixedSize(160, 160);

  knob_ = new QLabel();
  knob_->setParent(this);
  knob_->setGeometry(radius_ - knobRadius_, radius_ - knobRadius_, knobWidth_, knobHeight_);
  knob_->setScaledContents(true);
  knob_->setAttribute(Qt::WA_TranslucentBackground, true);
  knob_->setPixmap(QPixmap(":/images/knob.png"));
}

Joystick::~Joystick() {
  if (knob_ != nullptr) {
    delete knob_;
    knob_ = nullptr;
  }
}

bool Joystick::eventFilter(QObject* ob, QEvent* e) {
  // mouse double click -> reset knob to center
  if (e->type() == QEvent::MouseButtonDblClick) {
    mouseReleased_ = true;
    knob_->move(computeKnobPosition(radius_, radius_));
    updateJoystickValues(knob_->pos());

    return QLabel::eventFilter(ob, e);
  }

  if (e->type() == QEvent::MouseButtonPress) {
    auto mouseEvent = dynamic_cast<QMouseEvent*>(e);
    mouseReleased_ = false;

    if (Qt::ShiftModifier == QApplication::queryKeyboardModifiers()) {
      if (abs(mouseEvent->x() - radius_) > abs(mouseEvent->y() - radius_)) {
        knob_->move(computeKnobPosition(mouseEvent->x(), radius_));
      } else {
        knob_->move(computeKnobPosition(radius_, mouseEvent->y()));
      }
    } else {
      knob_->move(computeKnobPosition(mouseEvent->x(), mouseEvent->y()));
    }
    updateJoystickValues(knob_->pos());
  }

  if (e->type() == QEvent::MouseButtonRelease) {
    mouseReleased_ = true;
    // If autoReset_ is true, return the knob to center on mouse button release
    if (autoReset_) {
      knob_->move(computeKnobPosition(radius_, radius_));
      updateJoystickValues(knob_->pos());
    }
  }

  if (e->type() == QEvent::MouseMove && !mouseReleased_) {
    auto mouseEvent = dynamic_cast<QMouseEvent*>(e);

    if (Qt::ShiftModifier == QApplication::queryKeyboardModifiers()) {
      if (abs(mouseEvent->x() - radius_) > abs(mouseEvent->y() - radius_)) {
        knob_->move(computeKnobPosition(mouseEvent->x(), radius_));
      } else {
        knob_->move(computeKnobPosition(radius_, mouseEvent->y()));
      }
    } else {
      knob_->move(computeKnobPosition(mouseEvent->x(), mouseEvent->y()));
    }
    updateJoystickValues(knob_->pos());
  }

  return QLabel::eventFilter(ob, e);
}

void Joystick::setAutoReset(bool autoReset) {
  autoReset_ = autoReset;
  if (autoReset_) {
    // Centers the knob
    knob_->move(computeKnobPosition(radius_, radius_));
    updateJoystickValues(knob_->pos());
  }
}

void Joystick::setKnobPosition(double x, double y) {
  double span = knobRadius_ - radius_;
  double knobX = span * x + radius_ - knobRadius_;
  double knobY = span * y + radius_ - knobRadius_;
  QPoint knobPosition(std::lround(knobX), std::lround(knobY));
  knob_->move(knobPosition);
  updateJoystickValues(knob_->pos());
}

QPoint Joystick::computeKnobPosition(int mouseX, int mouseY) {
  if (std::sqrt(std::pow(mouseX - radius_, 2.0) + std::pow(mouseY - radius_, 2.0)) > radius_ - knobRadius_) {
    double v_x = mouseX - radius_;
    double v_y = mouseY - radius_;
    double x = v_x / std::sqrt(std::pow(v_x, 2.0) + std::pow(v_y, 2.0));
    double y = v_y / std::sqrt(std::pow(v_x, 2.0) + std::pow(v_y, 2.0));
    x *= static_cast<double>(radius_ - knobRadius_);
    y *= static_cast<double>(radius_ - knobRadius_);
    return {static_cast<int>(radius_ + x - knobRadius_), static_cast<int>(radius_ + y - knobRadius_)};
  }
  return {mouseX - knobRadius_, mouseY - knobRadius_};
}

void Joystick::updateJoystickValues(QPoint knobPosition) {
  double x = knobPosition.x() + knobRadius_ - radius_;
  double y = knobPosition.y() + knobRadius_ - radius_;
  double span = radius_ - knobRadius_;
  x /= -span;
  y /= -span;
  x = (x == 0 ? -x : x);
  y = (y == 0 ? -y : y);

  emit joystickMoved(x, y);
}

}  // namespace joystick_label
