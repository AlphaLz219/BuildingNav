/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include <ratio_layouted_frame/RatioLayoutedFrame.h>
#include <cmath>

namespace ratio_layouted_frame {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

RatioLayoutedFrame::RatioLayoutedFrame(QWidget *parent, Qt::WindowFlags flags)
    : QFrame(parent), aspectRatio_(4, 2) {
  this->setAttribute(Qt::WA_Hover);
  this->setMouseTracking(true);
  this->installEventFilter(this);

  connect(this, SIGNAL(delayedUpdate()),
          this, SLOT(update()), Qt::QueuedConnection);
}

RatioLayoutedFrame::~RatioLayoutedFrame() {
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

const QImage &RatioLayoutedFrame::getImage() const {
  return qImage_;
}

void RatioLayoutedFrame::setImage(const QImage &image) {
  {
    QMutexLocker locker(&qImageMutex_);
    qImage_ = image.copy();
  }
  setAspectRatio(qImage_.width(), qImage_.height());
  emit delayedUpdate();
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize &size) {
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMinimumSize(new_size);
  emit delayedUpdate();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize &size) {
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMaximumSize(new_size);
  emit delayedUpdate();
}

void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize &size) {
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

void RatioLayoutedFrame::setOuterLayout(QHBoxLayout *outerLayout) {
  outerLayout_ = outerLayout;
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void RatioLayoutedFrame::resizeToFitAspectRatio() {
  QRect rect = contentsRect();

  // reduce longer edge to aspect ration
  double width;
  double height;
  if (outerLayout_) {
    width = outerLayout_->contentsRect().width();
    height = outerLayout_->contentsRect().height();
  } else {
    // if outer layout isn't available, this will use the old
    // width and height, but this can shrink the display image if the
    // aspect ratio changes.
    width = rect.width();
    height = rect.height();
  }

  double layout_ar = width / height;
  const double imageAspectRatio =
      (double)aspectRatio_.width() / (double)aspectRatio_.height();
  if (layout_ar > imageAspectRatio) {
    // too large width
    width = height * imageAspectRatio;
  } else {
    // too large height
    height = width / imageAspectRatio;
  }

  rect.setWidth(std::round(width));
  rect.setHeight(std::round(height));

  // resize taking the border line into account
  int border = lineWidth();
  resize(rect.width() + 2 * border, rect.height() + 2 * border);
}

void RatioLayoutedFrame::setAspectRatio(int width, int height) {
  int divisor = greatestCommonDivisor(width, height);
  if (divisor != 0) {
    aspectRatio_.setWidth(std::round((double)width / (double)divisor));
    aspectRatio_.setHeight(std::round((double)height / (double)divisor));
  }
}

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b) {
  if (b == 0) {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */

void RatioLayoutedFrame::paintEvent(QPaintEvent *event) {
  QPainter painter(this);

  QMutexLocker locker(&qImageMutex_);

  if (!qImage_.isNull()) {
    resizeToFitAspectRatio();
    // TODO: check if full draw is really necessary
    //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
    //painter.drawImage(paint_event->rect(), qimage_);
    if (contentsRect().width() == qImage_.width()) {
      painter.drawImage(contentsRect(), qImage_);
    } else {
      QImage image = qImage_.scaled(contentsRect().width(),
                                    contentsRect().height(),
                                    Qt::KeepAspectRatio,
                                    Qt::SmoothTransformation);
      painter.drawImage(contentsRect(), image);
    }
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
}

/* ========================================================================== */
/* Event filter                                                               */
/* ========================================================================== */

bool RatioLayoutedFrame::eventFilter(QObject *object, QEvent *event) {
//        ROS_INFO_STREAM("event..\n");
//    if(e->type() == QEvent::KeyPress){
//        ROS_INFO_STREAM("KeyPress");
//        const QKeyEvent *ke = static_cast<QKeyEvent *>(e);

  if (event->type() == QEvent::HoverEnter) {
//        QHoverEvent *hoverEvent = static_cast<QHoverEvent *>(e);
//        std::cout << "frame hover enter\n";
  }
  if (event->type() == QEvent::HoverLeave) {
//        QHoverEvent *hoverEvent = static_cast<QHoverEvent *>(e);
//        std::cout << "frame hover leave\n";
  }

  return QWidget::eventFilter(object, event);
}

} // namespace
