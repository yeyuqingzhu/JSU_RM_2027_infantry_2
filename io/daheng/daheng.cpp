#include "daheng.hpp"

#include <libusb-1.0/libusb.h>

#include <stdexcept>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace
{
double yaml_get_double(const std::string & path, const std::string & key, double def)
{
  try {
    auto y = YAML::LoadFile(path);
    if (y[key]) return y[key].as<double>();
  } catch (const std::exception &) {}
  return def;
}
std::string yaml_get_str(const std::string & path, const std::string & key,
                         const std::string & def)
{
  try {
    auto y = YAML::LoadFile(path);
    if (y[key]) return y[key].as<std::string>();
  } catch (const std::exception &) {}
  return def;
}
}  // namespace

namespace io
{
std::atomic<int> DaHeng::sdk_ref_count_(0);
std::mutex g_init_mutex;
void DaHeng::sdk_open() {
  std::lock_guard<std::mutex> lock(g_init_mutex);
  if (sdk_ref_count_++ == 0) {
      GX_STATUS st = GXInitLib();
      if (st != GX_STATUS_SUCCESS) { 
        sdk_ref_count_ = 0; 
        throw std::runtime_error("GXInitLib failed"); 
      }
  }
}
void DaHeng::sdk_close() {
  std::lock_guard<std::mutex> lock(g_init_mutex);
  if (--sdk_ref_count_ == 0) GXCloseLib();
}
DaHeng::DaHeng(double exposure_ms, double gamma, double gain, const std::string & vid_pid)
: exposure_ms_(exposure_ms),
  gamma_(gamma),
  gain_(gain),
  handle_(nullptr),
  quit_(false),
  ok_(false),
  queue_(1),
  vid_(-1),
  pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  sdk_open();

  try_open();

  // 守护线程
  daemon_thread_ = std::thread{[this] {
    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) continue;

      if (capture_thread_.joinable()) capture_thread_.join();

      close();
      reset_usb();
      try_open();
    }
  }};
}

DaHeng::DaHeng(const std::string & config_path)
  : DaHeng(yaml_get_double(config_path, "exposure_ms", -1.0),
           yaml_get_double(config_path, "gamma", 0.5),
           yaml_get_double(config_path, "gain", -1.0),
           yaml_get_str(config_path, "vid_pid", "2ba2:4d55"))
{
}

DaHeng::~DaHeng()
{
  quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  close();
  sdk_close();
  tools::logger()->info("DaHeng destructed.");
}

void DaHeng::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void DaHeng::open()
{
  GX_STATUS emStatus;
  GX_OPEN_PARAM stOpenParam;
  GX_INT_VALUE stIntValue;
  uint32_t ui32DeviceNum = 0;
  
  emStatus = GXUpdateAllDeviceList(&ui32DeviceNum, 1000);
  if ((emStatus == GX_STATUS_SUCCESS) && (ui32DeviceNum > 0)) {
    for (uint32_t i = 0; i < ui32DeviceNum; ++i) {
      GX_DEVICE_INFO stDeviceInfo;
      GXGetDeviceInfo(i, &stDeviceInfo);
    }
  }

  if (ui32DeviceNum == 0) throw std::runtime_error("Not found camera!");

  stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE; //配置控制方式为独占
  stOpenParam.openMode = GX_OPEN_INDEX;         //配置打开方式为枚举
  stOpenParam.pszContent = "1";                 //配置枚举为第一个

  if (GXOpenDevice(&stOpenParam, &handle_) != GX_STATUS_SUCCESS)
    throw std::runtime_error("Failed to init camera!");

  emStatus = GXGetIntValue(handle_, "Width", &stIntValue);
  width_ = stIntValue.nCurValue;
  emStatus = GXGetIntValue(handle_, "Height", &stIntValue);
  height_ = stIntValue.nCurValue;

  GXSetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
  GXSetEnumValueByString(handle_, "TriggerMode", "Off");
  GXSetEnumValueByString(handle_, "PixelFormat", "BayerGR8");

  GXSetEnumValueByString(handle_, "BalanceWhiteAuto", "Off");

  GXSetEnumValueByString(handle_, "BalanceRatioSelector", "Red");
  GXSetFloatValue(handle_, "BalanceRatio", 1.5f);

  GXSetEnumValueByString(handle_, "BalanceRatioSelector", "Green");
  GXSetFloatValue(handle_, "BalanceRatio", 1.0f);

  GXSetEnumValueByString(handle_, "BalanceRatioSelector", "Blue");
  GXSetFloatValue(handle_, "BalanceRatio", 1.3984f);

  if (exposure_ms_ >= 0.0) {
    GXSetEnumValueByString(handle_, "ExposureAuto", "Off");
    GXSetFloatValue(handle_, "ExposureTime", exposure_ms_ * 1000.0);
  }
  GXSetFloatValue(handle_, "Gamma", gamma_);
  if (gain_ >= 0.0) {
    GXSetEnumValueByString(handle_, "GainAuto", "Off");
    GXSetFloatValue(handle_, "Gain", gain_);
  }

  GXStreamOn(handle_);

  // 取图线程
  capture_thread_ = std::thread{[this] {
    PGX_FRAME_DATA_EX pFrameBuffer;
    VxInt32 DxStatus;
    ok_ = true;

    while (!quit_) {

      auto img = cv::Mat(height_, width_, CV_8UC3);

      auto status = GXDQBufEx(handle_,&pFrameBuffer,1000);
      auto timestamp = std::chrono::steady_clock::now();

      if (status != GX_STATUS_SUCCESS) {
        tools::logger()->warn("Camera dropped!");
        ok_ = false;
        break;
      }

      DX_BAYER_CONVERT_TYPE emType = RAW2RGB_NEIGHBOUR; //选择插值算法
      DX_PIXEL_COLOR_FILTER emBayerType = BAYERRG;      //选择图像 Bayer 格式
      bool bFlip = false;
      DX_RGB_CHANNEL_ORDER emChannelOrder = DX_ORDER_BGR;

      if (pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
        GXQBufEx(handle_, pFrameBuffer);
        continue;
      }

      DxStatus = DxRaw8toRGB24Ex((void*)pFrameBuffer -> pImgBuf, img.data,
                      width_, height_,
                      emType, emBayerType, bFlip, emChannelOrder);
      if (DxStatus != DX_OK) {
        tools::logger()->warn("DxRaw8toRGB24Ex failed: {}", DxStatus);
        continue;
      }
      GXQBufEx(handle_,pFrameBuffer);
      queue_.push({img, timestamp});
    }
  }};

  tools::logger()->info("DaHeng opened.");
}

void DaHeng::try_open()
{
  try {
    open();
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void DaHeng::close()
{
  if (handle_ == nullptr) return;

  GXStreamOff(handle_);
  GXCloseDevice(handle_);
}

void DaHeng::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void DaHeng::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
