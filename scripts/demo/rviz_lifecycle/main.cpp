// Repository-local embedding of installed RViz; no patched system libraries.
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <QApplication>
#include <QTimer>
#include <OgreRoot.h>
#include <OgreRenderSystem.h>
#include <OgreRenderSystemCapabilities.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>

namespace {
volatile std::sig_atomic_t requested_signal = 0;
void request_exit(int value) { requested_signal = value; }
void stage(const char *value) { std::cerr << "RVIZ_LIFECYCLE " << value << std::endl; }
}

int main(int argc, char **argv) {
  // Handle non-graphical validation without constructing QApplication/ROS/GL.
  if (argc == 2 && std::string(argv[1]) == "--check-build") {
    std::cout << "t1_demo_rviz build OK; no display/context created" << std::endl;
    return 0;
  }
  if (argc < 3 || std::string(argv[1]) != "-d") {
    std::cerr << "Usage: t1_demo_rviz -d <config> [--ros-args ...]" << std::endl;
    return 2;
  }
  const QString config = QString::fromLocal8Bit(argv[2]);
  if (!std::ifstream(argv[2]).good()) { return 2; }
  // Set only this application's Qt backend; never a system display setting.
  qputenv("QT_QPA_PLATFORM", "xcb");
  auto non_ros = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> qt_argv;
  for (auto &arg : non_ros) { qt_argv.push_back(arg.data()); }
  int qt_argc = static_cast<int>(qt_argv.size());
  QApplication app(qt_argc, qt_argv.data());
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_exit);
  std::signal(SIGTERM, request_exit);
  auto ros_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz");
  auto frame = std::make_unique<rviz_common::VisualizationFrame>(ros_node);
  frame->setApp(&app);
  frame->setDisplayTitleFormat("T1 Demo - RViz");
  frame->setSplashPath("");
  frame->initialize(ros_node, config);
  frame->show();
  const auto *caps = Ogre::Root::getSingleton().getRenderSystem()->getCapabilities();
  const std::string renderer = caps->getDeviceName();
  std::cout << "OpenGL renderer string: " << renderer << std::endl;
  if (renderer.find("NVIDIA") == std::string::npos || renderer.find("4050") == std::string::npos ||
      renderer.find("Intel") != std::string::npos) {
    std::cerr << "RVIZ_RENDERER_REJECTED: expected NVIDIA RTX4050" << std::endl;
    requested_signal = SIGTERM;
  }
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    if (const char *path = std::getenv("T1_RVIZ_HEARTBEAT")) {
      std::ofstream beat(path); beat << "qt-event-loop-responsive\n";
    }
    if (requested_signal) {
      stage("exit_request_on_qt_thread");
      frame->setWindowModified(false);
      app.quit();
    }
  });
  timer.start(200);
  stage("ready");
  const int code = app.exec();
  timer.stop();
  stage("stop_updates_context_valid");
  if (!rclcpp::ok()) { throw std::runtime_error("ROS context invalid before display teardown"); }
  frame->getManager()->stopUpdate();
  // Remove rows individually: avoid nested row-removal notifications in stock
  // DisplayGroup::removeAllDisplays. Libraries remain loaded until frame reset.
  auto *root = frame->getManager()->getRootDisplayGroup();
  while (root->numDisplays()) {
    auto *display = root->getDisplayAt(root->numDisplays() - 1);
    stage("destroy_display_context_valid");
    delete root->takeDisplay(display);
  }
  stage("ros_shutdown_after_displays");
  rclcpp::shutdown();
  stage("destroy_frame_after_ros");
  frame.reset();
  ros_node.reset();
  stage("clean_exit");
  return renderer.find("NVIDIA") != std::string::npos && renderer.find("4050") != std::string::npos ? code : 3;
}
