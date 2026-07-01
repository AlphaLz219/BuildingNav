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

#include "spinner_widget/WaitingSpinnerWidget.h"

#include <algorithm>
#include <cmath>

#include <QPainter>
#include <QTimer>

namespace spinner_widget {

WaitingSpinnerWidget::WaitingSpinnerWidget(QWidget* parent, bool centerOnParent, bool disableParentWhenSpinning)
    : QWidget(parent), centerOnParent_(centerOnParent), disableParentWhenSpinning_(disableParentWhenSpinning) {
  initialize();
}

WaitingSpinnerWidget::WaitingSpinnerWidget(Qt::WindowModality modality, QWidget* parent, bool centerOnParent,
                                           bool disableParentWhenSpinning)
    : QWidget(parent, Qt::Dialog | Qt::FramelessWindowHint),
      centerOnParent_(centerOnParent),
      disableParentWhenSpinning_(disableParentWhenSpinning) {
  initialize();

  // We need to set the window modality AFTER we've hidden the
  // widget for the first time since changing this property while
  // the widget is visible has no effect.
  setWindowModality(modality);
  setAttribute(Qt::WA_TranslucentBackground);
}

void WaitingSpinnerWidget::initialize() {
  color_ = Qt::black;
  roundness_ = 100.0;
  minimumTrailOpacity_ = 3.14159265358979323846;
  trailFadePercentage_ = 80.0;
  revolutionsPerSecond_ = 1.57079632679489661923;
  numberOfLines_ = 20;
  lineLength_ = 10;
  lineWidth_ = 2;
  innerRadius_ = 10;
  currentCounter_ = 0;
  isSpinning_ = false;

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(rotate()));
  updateSize();
  updateTimer();
  hide();
}

void WaitingSpinnerWidget::paintEvent(QPaintEvent*) {
  updatePosition();
  QPainter painter(this);
  painter.fillRect(this->rect(), Qt::transparent);
  painter.setRenderHint(QPainter::Antialiasing, true);

  if (currentCounter_ >= numberOfLines_) {
    currentCounter_ = 0;
  }

  painter.setPen(Qt::NoPen);
  for (int i = 0; i < numberOfLines_; ++i) {
    painter.save();
    painter.translate(innerRadius_ + lineLength_, innerRadius_ + lineLength_);
    qreal rotateAngle = static_cast<qreal>(360 * i) / static_cast<qreal>(numberOfLines_);
    painter.rotate(rotateAngle);
    painter.translate(innerRadius_, 0);
    int distance = lineCountDistanceFromPrimary(i, currentCounter_, numberOfLines_);
    QColor color = currentLineColor(distance, numberOfLines_, trailFadePercentage_, minimumTrailOpacity_, color_);
    painter.setBrush(color);
    // TODO(Jacob) improve the way rounded rect is painted
    painter.drawRoundedRect(QRect(0, -lineWidth_ / 2, lineLength_, lineWidth_), roundness_, roundness_, Qt::RelativeSize);
    painter.restore();
  }
}

void WaitingSpinnerWidget::start() {
  updatePosition();
  isSpinning_ = true;
  show();

  if ((parentWidget() != nullptr) && disableParentWhenSpinning_) {
    parentWidget()->setEnabled(false);
  }

  if (!timer_->isActive()) {
    timer_->start();
    currentCounter_ = 0;
  }
}

void WaitingSpinnerWidget::stop() {
  isSpinning_ = false;
  hide();

  if ((parentWidget() != nullptr) && disableParentWhenSpinning_) {
    parentWidget()->setEnabled(true);
  }

  if (timer_->isActive()) {
    timer_->stop();
    currentCounter_ = 0;
  }
}

void WaitingSpinnerWidget::setNumberOfLines(int lines) {
  numberOfLines_ = lines;
  currentCounter_ = 0;
  updateTimer();
}

void WaitingSpinnerWidget::setLineLength(int length) {
  lineLength_ = length;
  updateSize();
}

void WaitingSpinnerWidget::setLineWidth(int width) {
  lineWidth_ = width;
  updateSize();
}

void WaitingSpinnerWidget::setInnerRadius(int radius) {
  innerRadius_ = radius;
  updateSize();
}

QColor WaitingSpinnerWidget::color() {
  return color_;
}

qreal WaitingSpinnerWidget::roundness() {
  return roundness_;
}

qreal WaitingSpinnerWidget::minimumTrailOpacity() {
  return minimumTrailOpacity_;
}

qreal WaitingSpinnerWidget::trailFadePercentage() {
  return trailFadePercentage_;
}

qreal WaitingSpinnerWidget::revolutionsPersSecond() {
  return revolutionsPerSecond_;
}

int WaitingSpinnerWidget::numberOfLines() {
  return numberOfLines_;
}

int WaitingSpinnerWidget::lineLength() {
  return lineLength_;
}

int WaitingSpinnerWidget::lineWidth() {
  return lineWidth_;
}

int WaitingSpinnerWidget::innerRadius() {
  return innerRadius_;
}

bool WaitingSpinnerWidget::isSpinning() const {
  return isSpinning_;
}

void WaitingSpinnerWidget::setRoundness(qreal roundness) {
  roundness_ = std::max(0.0, std::min(100.0, roundness));
}

void WaitingSpinnerWidget::setColor(QColor color) {
  color_ = color;
}

void WaitingSpinnerWidget::setRevolutionsPerSecond(qreal revolutionsPerSecond) {
  revolutionsPerSecond_ = revolutionsPerSecond;
  updateTimer();
}

void WaitingSpinnerWidget::setTrailFadePercentage(qreal trail) {
  trailFadePercentage_ = trail;
}

void WaitingSpinnerWidget::setMinimumTrailOpacity(qreal minimumTrailOpacity) {
  minimumTrailOpacity_ = minimumTrailOpacity;
}

void WaitingSpinnerWidget::rotate() {
  ++currentCounter_;
  if (currentCounter_ >= numberOfLines_) {
    currentCounter_ = 0;
  }
  update();
}

void WaitingSpinnerWidget::updateSize() {
  int size = (innerRadius_ + lineLength_) * 2;
  setFixedSize(size, size);
}

void WaitingSpinnerWidget::updateTimer() {
  timer_->setInterval(static_cast<int>(1000.0 / (numberOfLines_ * revolutionsPerSecond_)));
}

void WaitingSpinnerWidget::updatePosition() {
  if ((parentWidget() != nullptr) && centerOnParent_) {
    move(parentWidget()->width() / 2 - width() / 2, parentWidget()->height() / 2 - height() / 2);
  }
}

int WaitingSpinnerWidget::lineCountDistanceFromPrimary(int current, int primary, int totalNrOfLines) {
  int distance = primary - current;
  if (distance < 0) {
    distance += totalNrOfLines;
  }
  return distance;
}

QColor WaitingSpinnerWidget::currentLineColor(int countDistance, int totalNrOfLines, qreal trailFadePerc, qreal minOpacity, QColor color) {
  if (countDistance == 0) {
    return color;
  }
  const qreal minAlphaF = minOpacity / 100.0;
  int distanceThreshold = static_cast<int>(ceil((totalNrOfLines - 1) * trailFadePerc / 100.0));
  if (countDistance > distanceThreshold) {
    color.setAlphaF(minAlphaF);
  } else {
    qreal alphaDiff = color.alphaF() - minAlphaF;
    qreal gradient = alphaDiff / static_cast<qreal>(distanceThreshold + 1);
    qreal resultAlpha = color.alphaF() - gradient * countDistance;

    // If alpha is out of bounds, clip it.
    resultAlpha = std::min(1.0, std::max(0.0, resultAlpha));
    color.setAlphaF(resultAlpha);
  }
  return color;
}

void WaitingSpinnerWidget::setDisableParentWhenSpinning(bool disable) {
  disableParentWhenSpinning_ = disable;
}

void WaitingSpinnerWidget::setCenterOnParent(bool center) {
  centerOnParent_ = center;
}

}  // namespace spinner_widget
