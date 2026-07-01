
/**
 * @authors     Linus Isler
 * @affiliation ANYbotics
 * @brief       Implementation of a slider UI component to move over a range of values.
 */

#include "double_slider/RangeSlider.h"

namespace double_slider {

// TODO(ynava) Convert static variables to class members.
static const int scHandleSize = 10;
static const int scSliderBarHeight = 3;
static const int scMargin = 20;


RangeSlider::RangeSlider(QWidget *parent)
    : QWidget(parent)
    , minimum_(0)
    , maximum_(100)
    , lowerValue_(0)
    , upperValue_(100)
    , firstHandlePressed_(false)
    , secondHandlePressed_(false)
    , interval_(maximum_ - minimum_)
    , backgroundColorEnabled_(QColor(0x1E, 0x90, 0xFF))
    , backgroundColorDisabled_(Qt::darkGray)
    , backgroundColor_(backgroundColorEnabled_)
{
  setMouseTracking(true);
}


void RangeSlider::paintEvent(QPaintEvent *event) {
//  Q_UNUSED(event);
  QPainter painter(this);
  float firstRelativeValue = (lowerValue_ - minimum_) * 1.0 / interval_;
  float secondRelativeValue = (upperValue_ - minimum_) * 1.0 / interval_;
  paintSlider(&painter, rect(), backgroundColor_, firstRelativeValue, secondRelativeValue);
}


void RangeSlider::resizeEvent( QResizeEvent* event )
{
  QWidget::resizeEvent(event);
}


void RangeSlider::paintSlider(QPainter* painter,
                              const QRect& frame,
                              const QColor& bgColor,
                              const float firstRelativeValue,
                              const float secondRelativeValue) {
  painter->save();
  // Background
  QRectF backgroundRect =
      QRectF(frame.x() + scMargin, frame.y() + (frame.height() - scSliderBarHeight) / 2,
             frame.width() - scMargin * 2, scSliderBarHeight);
  QPen pen(Qt::gray, 0.8);
  painter->setPen(pen);
  painter->setRenderHint(QPainter::Qt4CompatiblePainting);
  QBrush backgroundBrush(QColor(0xD0, 0xD0, 0xD0));
  painter->setBrush(backgroundBrush);
  painter->drawRoundedRect(backgroundRect, 1, 1);

  // First value handle rect
  pen.setColor(Qt::darkGray);
  pen.setWidth(0.5);
  painter->setPen(pen);
  painter->setRenderHint(QPainter::Antialiasing);
  QBrush handleBrush(QColor(0xFA, 0xFA, 0xFA));
  painter->setBrush(handleBrush);
  QRectF leftHandleRect = handleRect(firstRelativeValue, frame);
  painter->drawRoundedRect(leftHandleRect, 2, 2);

  // Second value handle rect
  QRectF rightHandleRect = handleRect(secondRelativeValue, frame);
  painter->drawRoundedRect(rightHandleRect, 2, 2);

  // Handles
  painter->setRenderHint(QPainter::Antialiasing, false);
  QRectF selectedRect(backgroundRect);
  selectedRect.setLeft(leftHandleRect.right() + 0.5);
  selectedRect.setRight(rightHandleRect.left() - 0.5);
  QBrush selectedBrush(bgColor);
  painter->setBrush(selectedBrush);
  painter->drawRect(selectedRect);
  painter->restore();
}


QRectF RangeSlider::firstHandleRect() const {
  float percentage = (lowerValue_ - minimum_) * 1.0 / interval_;
  return handleRect(percentage, rect());
}


QRectF RangeSlider::secondHandleRect() const {
  float percentage = (upperValue_ - minimum_) * 1.0 / interval_;
  return handleRect(percentage, rect());
}


QRectF RangeSlider::handleRect(const float relativeValue, const QRectF& frame){
  float pixelValueX = frame.x() - scHandleSize/2 + scMargin + relativeValue * (frame.width() - scMargin * 2);
  float pixelValueY = frame.y() + (frame.height() - scHandleSize) / 2;
  return QRect(pixelValueX, pixelValueY, scHandleSize, scHandleSize);
}


void RangeSlider::mousePressEvent(QMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    secondHandlePressed_ = secondHandleRect().contains(event->pos());
    firstHandlePressed_ =
        !secondHandlePressed_ && firstHandleRect().contains(event->pos());

    // calculates delta between handle center and mouse click in x
    if (firstHandlePressed_) {
      delta_ = event->pos().x() - (firstHandleRect().x() + scHandleSize / 2);
    }
    else if (secondHandlePressed_) {
      delta_ = event->pos().x() - (secondHandleRect().x() + scHandleSize / 2);
    }

    if (event->pos().y() >= 2 && event->pos().y() <= height() - 2) {
      int step = interval_ / 10 < 1 ? 1 : interval_ / 10;
      if (event->pos().x() < firstHandleRect().x()) {
        setLowerValue(lowerValue_ - step);
      }
      else if (event->pos().x() >
                     firstHandleRect().x() + scHandleSize &&
                 event->pos().x() < secondHandleRect().x()) {
        if (event->pos().x() - (firstHandleRect().x() + scHandleSize) <
            (secondHandleRect().x() -
             (firstHandleRect().x() + scHandleSize)) /
                2) {
          if (lowerValue_ + step < upperValue_) {
            setLowerValue(lowerValue_ + step);
          }
          else {
            setLowerValue(upperValue_);
          }
        }
        else {
          if (upperValue_ - step > lowerValue_) {
            setUpperValue(upperValue_ - step);
          } else {
            setUpperValue(lowerValue_);
          }
        }
      }
      else if (event->pos().x() >
                 secondHandleRect().x() + scHandleSize) {
        setUpperValue(upperValue_ + step);
      }
    }
  }
}


void RangeSlider::mouseMoveEvent(QMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    if (firstHandlePressed_) {
      // Set the lower value only if first handle is left of second handle.
      float firstHandleRightSideX = event->pos().x() - delta_ + 0.5*scHandleSize;
      if (firstHandleRightSideX <= secondHandleRect().x()) {
        float firstHandlePositionX = event->pos().x() - delta_;
        float relativeLowerValue = (firstHandlePositionX - scMargin) / (width() - 2.0 * scMargin);
        setLowerValue((int) (relativeLowerValue * (float)interval_ + (float)minimum_));
      }
      // Otherwise set it same as the upper value.
      else {
        setLowerValue(upperValue_);
      }
    }
    else if (secondHandlePressed_) {
      // Set the upper value only if second handle is right of first handle.
      float secondHandleLeftSideX =  event->pos().x() - 0.5*scHandleSize;
      if (firstHandleRect().x() + scHandleSize <= secondHandleLeftSideX) {
        float secondHandlePositionX = event->pos().x() - delta_;
        float relativeUpperValue = (secondHandlePositionX - scMargin) / (width() - 2.0 * scMargin);
        setUpperValue((int) (relativeUpperValue * (float)interval_ + (float)minimum_));
      }
      else {
        setUpperValue(lowerValue_);
      }
    }
  }
}


void RangeSlider::mouseReleaseEvent(QMouseEvent *event) {
  Q_UNUSED(event);

  firstHandlePressed_ = false;
  secondHandlePressed_ = false;
}


void RangeSlider::changeEvent(QEvent *event) {
  if (event->type() == QEvent::EnabledChange) {
    if (isEnabled()) {
      backgroundColor_ = backgroundColorEnabled_;
    }
    else {
      backgroundColor_ = backgroundColorDisabled_;
    }
    update();
  }
}


QSize RangeSlider::minimumSizeHint() const {
  return QSize(scHandleSize * 2 + scMargin * 2,
               scHandleSize);
}


void RangeSlider::setLowerValue(int lowerValue) {
  if (lowerValue > maximum_) {
    lowerValue = maximum_;
  }

  if (lowerValue < minimum_) {
    lowerValue = minimum_;
  }

  lowerValue_ = lowerValue;
  emit lowerValueChanged(lowerValue_);

  update();
}


void RangeSlider::setUpperValue(int upperValue) {
  if (upperValue > maximum_) {
    upperValue = maximum_;
  }

  if (upperValue < minimum_) {
    upperValue = minimum_;
  }

  upperValue_ = upperValue;
  emit upperValueChanged(upperValue_);

  update();
}


void RangeSlider::setRange(int minimum, int maximum) {
  if (minimum <= maximum_) {
    minimum_ = minimum;
  }
  else {
    int oldMax = maximum_;
    minimum_ = oldMax;
    maximum_ = minimum;
  }
  if (maximum >= minimum_) {
    maximum_ = maximum;
  }
  else {
    int oldMin = minimum_;
    maximum_ = oldMin;
    minimum_ = maximum;
  }
  interval_ = maximum_ - minimum_;
  emit rangeChanged(minimum_, maximum_);
}

} // end namespace double_slider
