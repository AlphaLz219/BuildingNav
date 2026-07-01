/**
 * @authors     Linus Isler
 * @affiliation ANYbotics
 * @brief       Implementation of a Qt double slider.
 */

#include "double_slider/DoubleRangeSlider.h"

namespace double_slider {

DoubleRangeSlider::DoubleRangeSlider(QWidget *parent)
  : RangeSlider(parent)
{
  connect(this, SIGNAL(lowerValueChanged(int)), this, SLOT(notifyLowerValueChanged(int)));
  connect(this, SIGNAL(upperValueChanged(int)), this, SLOT(notifyUpperValueChanged(int)));

  connect(this, SIGNAL(lowerSliderMoved(int)), this, SLOT(notifyLowerSliderMoved(int)));
  connect(this, SIGNAL(upperSliderMoved(int)), this, SLOT(notifyUpperSliderMoved(int)));

  doubleLowerValue_ = (double)RangeSlider::lowerValue();
  doubleUpperValue_ = (double)RangeSlider::upperValue();
  doubleMinimum_ = (double)RangeSlider::minimum();
  doubleMaximum_ = (double)RangeSlider::maximum();
  updateRange_ = false;
}


double DoubleRangeSlider::map(double x, double inMin, double inMax,
                         double outMin, double outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


void DoubleRangeSlider::notifyLowerValueChanged(int value) {
  doubleLowerValue_ = map((double)value, (double)minimum(), (double)maximum(),
                     doubleMinimum_, doubleMaximum_);
  emit lowerValueChanged(doubleLowerValue_);
}


void DoubleRangeSlider::notifyUpperValueChanged(int value) {
  doubleUpperValue_ = map((double)value, (double)minimum(), (double)maximum(),
                     doubleMinimum_, doubleMaximum_);
  emit upperValueChanged(doubleUpperValue_);
}


void DoubleRangeSlider::notifyLowerSliderMoved(int value) {
  doubleLowerValue_ = map((double)value, (double)minimum(), (double)maximum(),
                     doubleMinimum_, doubleMaximum_);
  emit lowerSliderMoved(doubleLowerValue_);
}


void DoubleRangeSlider::notifyUpperSliderMoved(int value) {
  doubleUpperValue_ = map((double)value, (double)minimum(), (double)maximum(),
                     doubleMinimum_, doubleMaximum_);
  emit upperSliderMoved(doubleUpperValue_);
}


void DoubleRangeSlider::setLowerValue(double value) {
  if (value > doubleMaximum_) {
    value = doubleMaximum_;
  }
  if (value < doubleMinimum_) {
    value = doubleMinimum_;
  }
  int intLowerValue = (int) map(value, doubleMinimum_, doubleMaximum_,
                                (double) minimum(), (double) maximum());
  RangeSlider::setLowerValue(intLowerValue);
}


void DoubleRangeSlider::setUpperValue(double value) {
  if (value > doubleMaximum_) {
    value = doubleMaximum_;
  }
  if (value < doubleMinimum_) {
    value = doubleMinimum_;
  }
  int intUpperValue = (int) map(value, doubleMinimum_, doubleMaximum_,
                                (double) minimum(), (double) maximum());
  RangeSlider::setUpperValue(intUpperValue);
}


void DoubleRangeSlider::setRange(double min, double max) {
  doubleMinimum_ = min;
  doubleMaximum_ = max;
  setLowerValue(doubleLowerValue_);
  setUpperValue(doubleUpperValue_);

}


} // namespace
