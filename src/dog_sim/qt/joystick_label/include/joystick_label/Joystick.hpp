#pragma once

#include <QApplication>
#include <QEvent>
#include <QLabel>
#include <QMouseEvent>

namespace joystick_label {

class Joystick : public QLabel {
  Q_OBJECT
 public:
  /**
   * @brief Initialize joystick.
   * @param parent
   */
  explicit Joystick(QWidget* parent = nullptr);

  /**
   * @brief Destructor. Deallocate memory.
   */
  ~Joystick() override;

  /**
   * @brief React on mouse events to move around the joystick.
   * @param ob
   * @param e
   * @return
   */
  bool eventFilter(QObject* ob, QEvent* e) override;

  /**
   * @brief Sets the autoReset variable (if the knob should re-center on mouse release).
   * @param autoReset Knob gets centered if true.
   */
  void setAutoReset(bool autoReset);

  /**
   * @brief Set knob position.
   * @param x New horizontal coordinate.
   * @param y New vertical coordinate.
   */
  void setKnobPosition(double x, double y);

 private:
  QLabel* knob_ = nullptr;

  int radius_ = 80;

  int knobRadius_ = 26;
  int knobWidth_ = 52;
  int knobHeight_ = 52;

  bool mouseReleased_ = true;
  bool autoReset_ = false;

  /**
   * @brief Compute knob position from mouse inputs.
   * @param mouseX Mouse horizontal coordinate.
   * @param mouseY Mouse vertical coordinate.
   * @return New knob position.
   */
  QPoint computeKnobPosition(int mouseX, int mouseY);

  void updateJoystickValues(QPoint knobPosition);

 signals:
  void joystickMoved(double x, double y);
};

}  // namespace joystick_label
