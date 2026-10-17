#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "io/daheng/daheng.hpp"
#include "tools/exiter.hpp"

namespace fs = std::filesystem;

int main(int argc, char ** argv)
{
  /* --- config --- */
  double exposure_ms = 10.0;
  double gamma = 0.5;
  double gain = -1.0;
  std::string vid_pid = "2b03:f680";   // Daheng default
  std::string save_dir = "assets/captured";
  int interval_s = 2;                   // auto-capture interval
  std::string prefix = "capture";       // filename prefix

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--exp" && i + 1 < argc) exposure_ms = std::stod(argv[++i]);
    else if (arg == "--gamma" && i + 1 < argc) gamma = std::stod(argv[++i]);
    else if (arg == "--gain" && i + 1 < argc) gain = std::stod(argv[++i]);
    else if (arg == "--vidpid" && i + 1 < argc) vid_pid = argv[++i];
    else if (arg == "--dir" && i + 1 < argc) save_dir = argv[++i];
    else if (arg == "--interval" && i + 1 < argc) interval_s = std::stoi(argv[++i]);
    else if (arg == "--prefix" && i + 1 < argc) prefix = argv[++i];
  }

  /* --- create save directory with timestamp --- */
  auto now = std::chrono::system_clock::now();
  auto tt = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << save_dir << "_" << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S");
  auto dir = ss.str();

  try {
    fs::create_directories(dir);
  } catch (const fs::filesystem_error & e) {
    std::cerr << "[ERROR] Cannot create directory " << dir << ": " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Save dir: " << dir << std::endl;

  /* --- open camera --- */
  io::DaHeng camera(exposure_ms, gamma, gain, vid_pid);

  /* --- main loop --- */
  tools::Exiter exiter;
  int frame_count = 0;
  int auto_count = 0;
  auto last_auto = std::chrono::steady_clock::now();
  const std::string win = "Capture — Space: start/stop, A: auto-toggle, Q: quit";

  cv::namedWindow(win, cv::WINDOW_NORMAL);
  bool auto_mode = true;
  bool started = false;

  while (!exiter.exit()) {
    cv::Mat img;
    std::chrono::steady_clock::time_point ts;
    camera.read(img, ts);

    if (img.empty()) continue;

    /* auto-capture */
    if (started && auto_mode) {
      auto elapsed = std::chrono::duration<double>(ts - last_auto).count();
      if (elapsed >= interval_s) {
        auto_count++;
        std::string path = dir + "/" + prefix + "_" + std::to_string(auto_count) + ".jpg";
        cv::imwrite(path, img);
        std::cout << "[AUTO] saved: " << path << std::endl;
        last_auto = ts;
      }
    }

    /* show with overlay */
    cv::Mat display;
    int h = img.rows;
    if (h > 720) {
      double scale = 720.0 / h;
      cv::resize(img, display, cv::Size(), scale, scale);
    } else {
      display = img.clone();
    }

    std::string status = started ? (auto_mode ? "AUTO" : "MANUAL") : "STANDBY";
    std::string info = status + " | saved: " + std::to_string(auto_count);
    cv::putText(display, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.8, started ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255), 2);
    if (!started) {
      cv::putText(display, "Press SPACE to start capturing", cv::Point(10, 70),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }

    cv::imshow(win, display);
    frame_count++;

    /* key handling */
    int key = cv::waitKey(1);
    if (key == ' ' || key == 's') {
      if (!started) {
        started = true;
        last_auto = std::chrono::steady_clock::now();
        std::cout << "[START] Capturing started." << std::endl;
      } else {
        /* manual save */
        auto_count++;
        std::string path = dir + "/" + prefix + "_" + std::to_string(auto_count) + ".jpg";
        cv::imwrite(path, img);
        std::cout << "[SAVE] saved: " << path << std::endl;
      }
    } else if (key == 'a' || key == 'A') {
      if (!started) continue;
      auto_mode = !auto_mode;
      last_auto = std::chrono::steady_clock::now();
      std::cout << "[MODE] " << (auto_mode ? "AUTO" : "MANUAL") << std::endl;
    } else if (key == 27 || key == 'q') {
      break;
    }
  }

  std::cout << "\n=== Done. " << auto_count << " images saved to " << dir << " ===" << std::endl;
  cv::destroyAllWindows();
  return 0;
}
