/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

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

namespace double_slider {

class DoubleSlider : public QSlider {
  Q_OBJECT
public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  DoubleSlider(QWidget *parent = 0);

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  double value();

  void setRange(double min, double max);

  void setResolution(double resolution);

  void setTickInterval(double tickInterval);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  int value_;
  int min_;
  int max_;
  int tickInterval_;
  double doubleValue_;
  double doubleMin_;
  double doubleMax_;
  double resolution_;
  double doubleTickInterval_;
  std::atomic<bool> updateRange_;

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void getMinMaxInt();

  void getTickIntervalInt();

  double map(double x, double inMin, double inMax,
             double outMin, double outMax);

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void sliderMoved(double value);

  void valueChanged(double value);

public slots:

  /***************************************************************************/
  /** Slots                                                                 **/
  /***************************************************************************/

  void notifyValueChanged(int value);

  void notifySliderMoved(int value);

  void notifyRangeChanged(int min, int max);
};

} // namespace

