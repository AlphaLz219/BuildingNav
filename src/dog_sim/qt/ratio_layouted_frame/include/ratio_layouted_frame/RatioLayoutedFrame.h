/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <QFrame>
#include <QImage>
#include <QLayout>
#include <QLayoutItem>
#include <QMutex>
#include <QPainter>
#include <QRect>
#include <QSize>
#include <QEvent>

#include <iostream>

namespace ratio_layouted_frame {

/**
 * @brief RatioLayoutedFrame is a layout containing a single frame with a
 * fixed aspect ratio. The default aspect ratio is 4:3.
 */
class RatioLayoutedFrame : public QFrame {
Q_OBJECT
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  /**
   * @brief Constructor.
   * @param parent
   * @param flags
   * @return
   */
  explicit RatioLayoutedFrame(QWidget *parent, Qt::WindowFlags flags = 0);

  /**
   * @brief Destructor.
   */
  ~RatioLayoutedFrame() override;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  /**
   * @brief Get the image previously set by setImage().
   * @return the image.
   */
  const QImage &getImage() const;

  /**
   * @brief Set a new image to be displayed.
   * @param image
   */
  virtual void setImage(const QImage &image);

  /**
   * @brief Set the minimum inner frame size.
   * @param size
   */
  void setInnerFrameMinimumSize(const QSize &size);

  /**
   * @brief Set the maximum inner frame size.
   * @param size
   */
  void setInnerFrameMaximumSize(const QSize &size);

  /**
   * @brief Set inner frame to a fixed size.
   * @param size
   */
  void setInnerFrameFixedSize(const QSize &size);

  /**
   * @brief Set the outer layout.
   * @param outerLayout
   */
  void setOuterLayout(QHBoxLayout *outerLayout);

  /* ======================================================================== */
  /* Event filter                                                             */
  /* ======================================================================== */

  /**
   * @brief Qt event filter, not used so far.
   * @param object
   * @param event
   * @return
   */
  bool eventFilter(QObject *object, QEvent *event) override;

signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */

  void delayedUpdate();

protected:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  QSize aspectRatio_;

  QImage qImage_;

  QMutex qImageMutex_;

  QHBoxLayout *outerLayout_ = nullptr;

  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */

  /**
   * @brief Draw the #qimage_. Can be triggerd by calling update().
   * @param event
   */
  void paintEvent(QPaintEvent *event) override;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  /**
   * @brief Set a new aspect ration.
   * @param width
   * @param height
   */
  void setAspectRatio(int width, int height);

  /**
   * @brief Resize frame depending on the aspect ratio.
   */
  void resizeToFitAspectRatio();

  /**
   * @brief Compute the greatest common divisor.
   * @param a
   * @param b
   * @return
   */
  static int greatestCommonDivisor(int a, int b);
};

} // namespace
