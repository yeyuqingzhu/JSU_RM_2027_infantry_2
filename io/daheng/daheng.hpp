#ifndef IO__DAHENG_HPP
#define IO__DAHENG_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "GxIAPI.h"
#include "DxImageProc.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class DaHeng : public CameraBase
{
public:
  DaHeng(double exposure_ms, double gamma, double gain, const std::string & vid_pid);
  explicit DaHeng(const std::string & config_path);
  ~DaHeng() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_ms_, gamma_, gain_;
  GX_DEV_HANDLE handle_;
  int height_, width_;
  bool quit_, ok_;
  std::thread capture_thread_;
  std::thread daemon_thread_;
  tools::ThreadSafeQueue<CameraData> queue_;
  int vid_, pid_;
  static std::atomic<int> sdk_ref_count_;
  static std::mutex init_mutex_;
  
  DX_BAYER_CONVERT_TYPE emType;
  DX_PIXEL_COLOR_FILTER emBayerType;
  bool bFlip = false;
  DX_RGB_CHANNEL_ORDER emChannelOrder;

  static void sdk_open();
  static void sdk_close();

  void open();
  void try_open();
  void close();
  
  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;

};

}  // namespace io

#endif  // IO__DAHENG_HPP