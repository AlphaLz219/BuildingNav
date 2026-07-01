/**
 * @authors     Linus Isler
 * @affiliation ANYbotics
 * @brief       Definition of a Qt double slider.
 */
#pragma once

#include <atomic>

#include <QApplication>
#include <QtGui>
#include <QWidget>
#include <QObject>
#include <QInputDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QTimer>
#include <QSlider>
#include <QLabel>
#include <QSizePolicy>

#include "double_slider/RangeSlider.h"


namespace double_slider {

class DoubleRangeSlider : public RangeSlider {
  Q_OBJECT
public:
  explicit DoubleRangeSlider(QWidget *parent = Q_NULLPTR);

  double lowerDoubleValue() const { return doubleLowerValue_; }
  double upperDoubleValue() const { return doubleUpperValue_; }

protected:
signals:
  void lowerSliderMoved(double value);
  void upperSliderMoved(double value);
  void lowerValueChanged(double value);
  void upperValueChanged(double value);

public slots:
  void notifyLowerValueChanged(int value);
  void notifyUpperValueChanged(int value);
  void notifyLowerSliderMoved(int value);
  void notifyUpperSliderMoved(int value);
  void setLowerValue(double value);
  void setUpperValue(double value);
  void setRange(double min, double max);

private:
  double map(double x, double inMin, double inMax, double outMin,
             double outMax);

  double doubleLowerValue_;
  double doubleUpperValue_;
  double doubleMinimum_;
  double doubleMaximum_;
  double doubleInterval_;
  std::atomic<bool> updateRange_;
};

} // end namespace double_slider
