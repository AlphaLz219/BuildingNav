/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "double_slider/DoubleSlider.h"

namespace double_slider {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

DoubleSlider::DoubleSlider(QWidget *parent) : QSlider(parent) {
  connect(this, SIGNAL(valueChanged(int)),
          this, SLOT(notifyValueChanged(int)));
  connect(this, SIGNAL(sliderMoved(int)),
          this, SLOT(notifySliderMoved(int)));
  connect(this, SIGNAL(rangeChanged(int, int)),
          this, SLOT(notifyRangeChanged(int, int)));

  resolution_ = 0.1;
  value_ = QSlider::value();
  doubleValue_ = (double)QSlider::value();
  doubleMin_ = (double)QSlider::minimum();
  doubleMax_ = (double)QSlider::maximum();
  doubleTickInterval_ = resolution_;
  getMinMaxInt();
  getTickIntervalInt();
  updateRange_ = false;
  QSlider::setRange(min_, max_);
  QSlider::setTickInterval(tickInterval_);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

double DoubleSlider::value() {
    return doubleValue_;
}

void DoubleSlider::setRange(double min, double max) {
  doubleMin_ = min;
  doubleMax_ = max;
  getMinMaxInt();
  getTickIntervalInt();
  updateRange_ = false;
  QSlider::setRange(min_, max_);
  QSlider::setTickInterval(tickInterval_);
}

void DoubleSlider::setResolution(double resolution) {
  resolution_ = resolution;
  getMinMaxInt();
  getTickIntervalInt();
  updateRange_ = false;
  QSlider::setRange(min_, max_);
  QSlider::setTickInterval(tickInterval_);
}

void DoubleSlider::setTickInterval(double tickInterval) {
  doubleTickInterval_ = tickInterval;
  getTickIntervalInt();
  QSlider::setTickInterval(tickInterval_);
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void DoubleSlider::getMinMaxInt() {
  min_ = 0;
  max_ = (int)((doubleMax_ - doubleMin_) / resolution_);
}

double DoubleSlider::map(double x, double inMin, double inMax,
                         double outMin, double outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void DoubleSlider::getTickIntervalInt() {
  tickInterval_ = (int)(doubleTickInterval_ /
                        (doubleMax_ - doubleMin_) * (double)max_);
}

/*****************************************************************************/
/** Slots                                                                   **/
/*****************************************************************************/

void DoubleSlider::notifyValueChanged(int value) {
  value_ = value;
  doubleValue_ = map((double)value_, (double)min_, (double)max_,
                     doubleMin_, doubleMax_);
  emit valueChanged(doubleValue_);
}

void DoubleSlider::notifySliderMoved(int value) {
  value_ = value;
  doubleValue_ = map((double)value_, (double)min_, (double)max_,
                     doubleMin_, doubleMax_);
  emit sliderMoved(doubleValue_);
}

void DoubleSlider::notifyRangeChanged(int min, int max) {
  if (!updateRange_) {
    updateRange_ = true;
    return;
  }
  doubleMin_ = (double)min;
  doubleMax_ = (double)max;
  getMinMaxInt();
  getTickIntervalInt();
  updateRange_ = false;
  QSlider::setRange(min_, max_);
  QSlider::setTickInterval(tickInterval_);
}

} // namespace
