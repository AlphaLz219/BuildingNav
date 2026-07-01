/* Original Work Copyright (c) 2012-2014 Alexander Turkin
   Modified 2014 by William Hallatt
   Modified 2015 by Jacob Dawid

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <QColor>
#include <QTimer>
#include <QWidget>

namespace spinner_widget {

class WaitingSpinnerWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructor for "standard" widget behaviour - use this
   * constructor if you wish to, e.g. embed your widget in another.
   */
  explicit WaitingSpinnerWidget(QWidget* parent = nullptr, bool centerOnParent = false, bool disableParentWhenSpinning = true);

  /**
   * @brief Constructor - use this constructor to automatically create a modal
   * ("blocking") spinner on top of the calling widget/window.  If a valid
   * parent widget is provided, "centreOnParent" will ensure that
   * QtWaitingSpinner automatically centres itself on it, if not,
   * "centreOnParent" is ignored.
   */
  explicit WaitingSpinnerWidget(Qt::WindowModality modality, QWidget* parent = nullptr, bool centerOnParent = false,
                                bool disableParentWhenSpinning = true);

  void setColor(QColor color);

  void setRoundness(qreal roundness);

  void setMinimumTrailOpacity(qreal minimumTrailOpacity);

  void setTrailFadePercentage(qreal trail);

  void setRevolutionsPerSecond(qreal revolutionsPerSecond);

  void setNumberOfLines(int lines);

  void setLineLength(int length);

  void setLineWidth(int width);

  void setInnerRadius(int radius);

  void setDisableParentWhenSpinning(bool disable);

  void setCenterOnParent(bool center);

  QColor color();

  qreal roundness();

  qreal minimumTrailOpacity();

  qreal trailFadePercentage();

  qreal revolutionsPersSecond();

  int numberOfLines();

  int lineLength();

  int lineWidth();

  int innerRadius();

  bool isSpinning() const;

 private slots:
  void rotate();

 public slots:
  void start();

  void stop();

 protected:
  void paintEvent(QPaintEvent* paintEvent) override;

 private:
  static int lineCountDistanceFromPrimary(int current, int primary, int totalNrOfLines);

  static QColor currentLineColor(int distance, int totalNrOfLines, qreal trailFadePerc, qreal minOpacity, QColor color);

  void initialize();

  void updateSize();

  void updateTimer();

  void updatePosition();

  WaitingSpinnerWidget(const WaitingSpinnerWidget&);

  WaitingSpinnerWidget& operator=(const WaitingSpinnerWidget&);

  QColor color_;
  qreal roundness_{0.0};  // 0..100
  qreal minimumTrailOpacity_{0.0};
  qreal trailFadePercentage_{0.0};
  qreal revolutionsPerSecond_{0.0};
  int numberOfLines_{0};
  int lineLength_{0};
  int lineWidth_{0};
  int innerRadius_{0};

  QTimer* timer_ = nullptr;
  bool centerOnParent_{false};
  bool disableParentWhenSpinning_{false};
  int currentCounter_{0};
  bool isSpinning_{false};
};

}  // namespace spinner_widget
