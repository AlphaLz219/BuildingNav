#include <pluginlib/class_list_macros.h>

#include <message_logger/message_logger.hpp>
#include <signal_handler/SignalHandler.hpp>

#include "rqt_rviz_wrapper/RvizWrapper.h"

namespace rqt_rviz_wrapper {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

RvizWrapper::RvizWrapper() : rqt_gui_cpp::Plugin(), widget_(nullptr) {
  setObjectName("RvizWrapper");
}

RvizWrapper::~RvizWrapper() {
  // Release RViz window.
  releaseRvizWindow();
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void RvizWrapper::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Hide elements.
  ui_.widgetWaitingScreen->hide();
  ui_.widgetRviz->hide();

  // Enable hover and event filter.
  ui_.widgetRviz->setAttribute(Qt::WA_Hover);
  ui_.widgetRviz->installEventFilter(this);
  widget_->installEventFilter(this);

  // Initialize signal handler.
  signal_handler::SignalHandler::bindAll(&RvizWrapper::handleSignal, this);

  // Connect.
  connect(ui_.pushButtonLoadRviz, &QPushButton::clicked, this, &RvizWrapper::onPushButtonLoadRvizClicked);
  connect(this, &RvizWrapper::loadRviz, this, &RvizWrapper::onLoadRviz);
  connect(ui_.pushButtonDiscardRviz, &QPushButton::clicked, this, &RvizWrapper::onPushButtonDiscardRvizClicked);

  // Setup timer.
  findWindowTimer_ = new QTimer(this);
  connect(findWindowTimer_, &QTimer::timeout, this, &RvizWrapper::onFindWindowTimerTimeout);
  findWindowTimer_->stop();

  // Check desktop environment (Unity, Gnome).
  checkDesktopEnvironment();

  // Start searching for RViz.
  onPushButtonLoadRvizClicked();
}

void RvizWrapper::shutdownPlugin() {
  findWindowTimer_->stop();

  // Release RViz window.
  releaseRvizWindow();
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

bool RvizWrapper::findWindow(int& windowId, const std::string& windowName) {
  // Get all windows in a string.
  std::string windowsString = getWindows();
  // Replace ".." by ".".
  replaceSubstring(windowsString, windowsString, "..", ".");
  // Split string.
  std::vector<std::array<std::string, 4> > windows;
  std::string token;
  while (splitString(windowsString, "\n", token, windowsString)) {
    std::array<std::string, 4> window;
    for (int i = 0; i < 3; ++i) {
      std::string token2;
      if (splitString(token, " ", token2, token)) {
        window[i] = token2;
      }
    }
    window[3] = token;
    windows.push_back(window);
  }
  // Search windows for desired window (RViz).
  for (auto window : windows) {
    MELO_DEBUG_STREAM("Window: " << window[3] << " with ID: " << window[0] << " (" << windowId << ")")
    if (window[3].find(windowName) != std::string::npos) {
      windowId = static_cast<int>(std::stoul(window[0], nullptr, 16));
      MELO_DEBUG_STREAM("Found window: " << window[3] << " with ID: " << window[0] << " (" << windowId << ")")
      return true;
    }
  }
  return false;
}

void RvizWrapper::replaceSubstring(std::string src, std::string& dst, const std::string& substring, const std::string& replacement) {
  size_t index = 0;
  while (true) {
    // Locate the substring to replace.
    index = src.find(substring, index);
    if (index == std::string::npos) {
      break;
    }
    // Make the replacement.
    src.replace(index, substring.length(), replacement);
    // Advance index forward so the next iteration doesn't pick it up as well.
    index += replacement.length();
  }
  dst = src;
}

std::string RvizWrapper::getWindows() {
  const int MAX_BUFFER = 255;
  std::string stdout;
  char buffer[MAX_BUFFER];
  FILE* stream = popen("wmctrl -l", "r");
  while (fgets(buffer, MAX_BUFFER, stream) != nullptr) {
    stdout.append(buffer);
  }
  pclose(stream);
  return stdout;
}

bool RvizWrapper::splitString(const std::string& string, const std::string& delimiter, std::string& firstString,
                              std::string& secondString) {
  size_t pos = 0;
  std::string str = string;
  if ((pos = str.find(delimiter)) != std::string::npos) {
    firstString = str.substr(0, pos);
    str.erase(0, pos + delimiter.length());
    secondString = str;
    return true;
  }
  return false;
}

void RvizWrapper::clearLayout(QLayout* layout) {
  QLayoutItem* item;
  while ((item = layout->takeAt(0)) != nullptr) {
    if (item->layout() != nullptr) {
      clearLayout(item->layout());
      delete item->layout();
    }
    if (item->widget() != nullptr) {
      delete item->widget();
    }
    delete item;
  }
}

void RvizWrapper::releaseRvizWindow() {
  if (rvizWindow_ != nullptr) {
    rvizWindow_->setParent(nullptr);
    if (!isUnityDesktop_) {
      rvizWindow_->setFlags(Qt::Window);
    }
    rvizWindow_->close();
    rvizWindow_ = nullptr;
  }
}

void RvizWrapper::checkDesktopEnvironment() {
  // GNOME
  // Name: GNOME Shell
  // Class: N/A
  // PID: N/A
  // Window manager's "showing the desktop" mode: N/A
  // Unity
  // Name: Compiz
  // Class: N/A
  // PID: N/A
  // Window manager's "showing the desktop" mode: OFF
  const int MAX_BUFFER = 255;
  std::string stdout;
  char buffer[MAX_BUFFER];
  FILE* stream = popen("wmctrl -m", "r");
  while (fgets(buffer, MAX_BUFFER, stream) != nullptr) {
    stdout.append(buffer);
  }
  pclose(stream);
  QString wmctrlOutput = QString::fromStdString(stdout);

  QRegExp exp(R"(Name: ([\w\s\d]{1,200})[\n|\r\n|\r])");
  int pos = exp.indexIn(wmctrlOutput);
  QString desktopWm;
  if (pos > -1) {
    desktopWm = exp.cap(1);
  }

  MELO_DEBUG_STREAM("Desktop environment: " << desktopWm.toStdString())

  // Get current desktop (GNOME, KDE, Unity).
  QString desktop = qgetenv("XDG_CURRENT_DESKTOP");
  isUnityDesktop_ = desktopWm.indexOf("gnome", 0, Qt::CaseInsensitive) == -1;
}

void RvizWrapper::handleSignal(const int signum) {
  findWindowTimer_->stop();
  releaseRvizWindow();

  signal(signum, SIG_DFL);
  kill(getpid(), signum);
}

/* ========================================================================== */
/* Event Filters                                                              */
/* ========================================================================== */

bool RvizWrapper::eventFilter(QObject* object, QEvent* event) {
  if (event->type() == QEvent::Paint && object == widget_ && !isFirstPainted_) {
    isFirstPainted_ = true;
  }
  return QObject::eventFilter(object, event);
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void RvizWrapper::onLoadRviz() {
  // Remove spinner from layout.
  ui_.spinner->stop();
  ui_.widgetLoadRviz->hide();
  ui_.widgetWaitingScreen->hide();
  ui_.widgetRviz->show();

  // Embed window.
  if (rvizWindow_ == nullptr) {
    MELO_ERROR_STREAM("Cannot embed RViz window. QWindow is not initialized.")
    return;
  }

  container_ = QWidget::createWindowContainer(rvizWindow_, widget_);
  ui_.verticalLayoutRviz->addWidget(container_);
  QTimer::singleShot(500, this, SLOT(onFixOffset()));
  isContainerLoaded_ = true;
}

void RvizWrapper::onFixOffset() {
  int width = widget_->width();
  widget_->resize(width - 1, widget_->height());
  widget_->resize(width + 1, widget_->height());
}

void RvizWrapper::onPushButtonLoadRvizClicked() {
  ui_.widgetRviz->hide();
  ui_.widgetLoadRviz->hide();

  ui_.widgetWaitingScreen->show();

  ui_.spinner->setRoundness(70.0);
  ui_.spinner->setMinimumTrailOpacity(15.0);
  ui_.spinner->setTrailFadePercentage(70.0);
  ui_.spinner->setNumberOfLines(12);
  ui_.spinner->setLineLength(10);
  ui_.spinner->setLineWidth(5);
  ui_.spinner->setInnerRadius(10);
  ui_.spinner->setRevolutionsPerSecond(1);
  ui_.spinner->setColor(QColor(0, 0, 0));
  ui_.spinner->start();

  // Start timer to wait for RViz.
  findWindowTimer_->start(1000);
}

void RvizWrapper::onPushButtonDiscardRvizClicked() {
  if (isContainerLoaded_) {
    // Release RViz window.
    releaseRvizWindow();

    // Remove container from layout.
    ui_.verticalLayoutRviz->removeWidget(container_);
    container_ = nullptr;

    // Hide elements.
    ui_.widgetWaitingScreen->hide();
    ui_.widgetRviz->hide();
    ui_.widgetLoadRviz->show();

    // Reset.
    isContainerLoaded_ = false;
  }
}

void RvizWrapper::onFindWindowTimerTimeout() {
  std::lock_guard<std::mutex> lockGuard(mutexFindWindow_);
  // Try to find a window, that starts with RViz.
  if (findWindow(windowId_, "RViz") && !isContainerLoaded_ && isFirstPainted_) {
    // Wait a bit (for RViz to open fully) and then load RViz.
    QTimer::singleShot(3000, this, SLOT(onStartLoadRviz()));

    // Stop timer.
    findWindowTimer_->stop();
  } else {
    MELO_DEBUG_STREAM("Waiting for RViz ...")
  }
}

void RvizWrapper::onStartLoadRviz() {
  // Assign RViz window to QWindow, set FramelessWindowHint and define as
  // widget type.
  rvizWindow_ = QWindow::fromWinId(static_cast<WId>(windowId_));
  if (!isUnityDesktop_) {
    rvizWindow_->setFlags(Qt::Widget | Qt::FramelessWindowHint);
  }

  // Load RViz into the container a bit later, such that FramelessWindowHint
  // is actually executed.
  QTimer::singleShot(100, this, SLOT(onLoadRviz()));
}

}  // namespace rqt_rviz_wrapper

PLUGINLIB_EXPORT_CLASS(rqt_rviz_wrapper::RvizWrapper, rqt_gui_cpp::Plugin)
