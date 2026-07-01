/**
 * @authors     Linus Isler
 * @affiliation ANYbotics
 * @brief       Definition of a slider UI component to move over a range of values.
 */
#pragma once

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>


namespace double_slider {

class RangeSlider : public QWidget {
  Q_OBJECT

public:
  RangeSlider(QWidget *parent = Q_NULLPTR);

  QSize minimumSizeHint() const override;

  int minimum() const { return minimum_; }
  int maximum() const { return maximum_; }
  int lowerValue() const { return lowerValue_; }
  int upperValue() const  { return upperValue_; }
  static void paintSlider(QPainter* painter, const QRect& frame, const QColor& bgColor,
                          const float firstRelativeValue = 0.0, const float secondRelativeValue = 1.0);
  static QRectF handleRect(const float relativeValue, const QRectF& frame = QRectF());


protected:
  virtual void paintEvent(QPaintEvent *event) override;
  virtual void resizeEvent(QResizeEvent* event);
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void changeEvent(QEvent *event) override;

  QRectF firstHandleRect() const;
  QRectF secondHandleRect() const;

signals:
  void lowerValueChanged(int lowerValue);
  void upperValueChanged(int upperValue);
  void lowerSliderMoved(int lowerValue);
  void upperSliderMoved(int upperValue);
  void rangeChanged(int min, int max);

public slots:
  void setLowerValue(int lowerValue);
  void setUpperValue(int upperValue);
  void setRange(int minimum, int maximum);

private:
  Q_DISABLE_COPY(RangeSlider)

  int minimum_;
  int maximum_;
  int lowerValue_;
  int upperValue_;
  bool firstHandlePressed_;
  bool secondHandlePressed_;
  int interval_;
  int delta_;
  QColor backgroundColorEnabled_;
  QColor backgroundColorDisabled_;
  QColor backgroundColor_;
};

} // end namespace double_slider