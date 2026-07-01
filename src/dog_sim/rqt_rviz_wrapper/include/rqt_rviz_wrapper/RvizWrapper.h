#pragma once

#include <atomic>
#include <mutex>

#include <QApplication>
#include <QCoreApplication>
#include <QEvent>
#include <QHBoxLayout>
#include <QHoverEvent>
#include <QLabel>
#include <QLayoutItem>
#include <QObject>
#include <QPushButton>
#include <QSpacerItem>
#include <QTimer>
#include <QWidget>
#include <QWindow>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <rqt_rviz_wrapper/ui_RvizWrapper.h>

namespace rqt_rviz_wrapper {

class RvizWrapper : public rqt_gui_cpp::Plugin {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  RvizWrapper();

  ~RvizWrapper() override;

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void shutdownPlugin() override;

 private:
  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::RvizWrapperWidget ui_{};
  QWidget* widget_;

  QWindow* rvizWindow_ = nullptr;
  QWidget* container_ = nullptr;

  QTimer* findWindowTimer_ = nullptr;

  bool isContainerLoaded_ = false;
  std::mutex mutexFindWindow_;

  int windowId_{0};

  bool isUnityDesktop_ = false;
  std::atomic<bool> isFirstPainted_{false};

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void replaceSubstring(std::string src, std::string& dst, const std::string& substring, const std::string& replacement);

  bool findWindow(int& windowId, const std::string& windowName);

  std::string getWindows();

  bool splitString(const std::string& string, const std::string& delimiter, std::string& firstString, std::string& secondString);

  void clearLayout(QLayout* layout);

  void releaseRvizWindow();

  void checkDesktopEnvironment();

  void handleSignal(const int signum);

  /* ======================================================================== */
  /* Event Filters                                                            */
  /* ======================================================================== */

  bool eventFilter(QObject* object, QEvent* event) override;

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void onLoadRviz();

  void onPushButtonLoadRvizClicked();

  void onPushButtonDiscardRvizClicked();

  void onFindWindowTimerTimeout();

  void onStartLoadRviz();

  void onFixOffset();

 signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */

  void loadRviz();
};

}  // namespace rqt_rviz_wrapper
