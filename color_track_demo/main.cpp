// =============================================================================
//  color_track_demo —— 颜色目标检测 + 云台追踪演示程序
//
//  功能：用 ONNX Runtime 加载 YOLO 模型（默认 assets/color_track_model.onnx），
//        检测画面中的目标、判断其类别（默认 RED_LIGHT / BLUE_LIGHT），
//        按检测框偏差换算成绝对目标角，经 io::LaserGimbal 串口下发给云台，
//        实现"识别类别 + 估计位置 + 自动追踪"。
//
//  许可：MIT License（见仓库根目录 LICENSE）
//        Copyright (c) 2026 JSU RoboMaster Team
//        本文件由本队编写（编码过程由 opencode 调用 DeepSeek 辅助生成），
//        不属于上游 sp_vision_25 的代码，本队保留其版权。
//  SPDX-License-Identifier: MIT
// =============================================================================

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include "io/daheng/daheng.hpp"
#include "io/gimbal/laser_gimbal.hpp"

namespace
{
/* =========================================================================
 * YOLO detector — ONNX Runtime with CUDA/CPU fallback.
 * Handles Ultralytics [1, N, 6] output (NMS pre-applied).
 * Preprocessing uses cv::dnn::blobFromImage (fast HWC→CHW).
 * ========================================================================= */
class YOLODetector
{
public:
  struct Box
  {
    cv::Rect bbox;
    int class_id;
    float confidence;
  };

  YOLODetector(const std::string & model_path, bool prefer_gpu = true,
               float conf_thresh = 0.5f)
    : conf_thresh_(conf_thresh)
  {
    if (prefer_gpu) {
      try {
        Ort::SessionOptions opts;
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        OrtCUDAProviderOptions cuda_opts;
        opts.AppendExecutionProvider_CUDA(cuda_opts);
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "yolo");
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
        load_metadata();
        std::cout << "  [ORT] Using CUDA (" << input_w_ << "x" << input_h_ << ")\n";
        return;
      } catch (const std::exception & e) {
        std::cout << "  [ORT] CUDA unavailable: " << e.what() << "\n";
        std::cout << "  [ORT] Falling back to CPU.\n";
        prefer_gpu = false;
      }
    }
    {
      Ort::SessionOptions cpu_opts;
      cpu_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "yolo");
      session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), cpu_opts);
      load_metadata();
      std::cout << "  [ORT] Using CPU (" << input_w_ << "x" << input_h_ << ")\n";
    }
  }

  std::vector<Box> detect(const cv::Mat & bgr_img)
  {
    auto prep = preprocess(bgr_img);
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem, prep.data.data(), prep.data.size(),
        prep.shape.data(), prep.shape.size());

    const char * in_names[] = {input_name_.c_str()};
    const char * out_names[] = {output_name_.c_str()};
    auto out_tensor = session_->Run(Ort::RunOptions{nullptr},
                                    in_names, &input_tensor, 1,
                                    out_names, 1);
    return postprocess(out_tensor, prep.scale, prep.pad_w, prep.pad_h);
  }

private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "yolo"};
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  int input_h_ = 0, input_w_ = 0;
  float conf_thresh_;

  struct PreprocResult
  {
    std::vector<float> data;
    std::vector<int64_t> shape;
    float scale;
    int pad_w, pad_h;
  };

  void load_metadata()
  {
    Ort::AllocatorWithDefaultOptions alloc;
    {
      auto in = session_->GetInputNameAllocated(0, alloc);
      auto out = session_->GetOutputNameAllocated(0, alloc);
      input_name_ = in.get();
      output_name_ = out.get();
    }
    auto info = session_->GetInputTypeInfo(0);
    auto shape = info.GetTensorTypeAndShapeInfo().GetShape();
    input_h_ = (shape.size() >= 3) ? static_cast<int>(shape[2]) : 960;
    input_w_ = (shape.size() >= 4) ? static_cast<int>(shape[3]) : 960;
  }

  PreprocResult preprocess(const cv::Mat & bgr_img)
  {
    int img_w = bgr_img.cols, img_h = bgr_img.rows;
    float scale = std::min(static_cast<float>(input_w_) / img_w,
                           static_cast<float>(input_h_) / img_h);
    int rw = static_cast<int>(img_w * scale);
    int rh = static_cast<int>(img_h * scale);
    int pw = (input_w_ - rw) / 2;
    int ph = (input_h_ - rh) / 2;

    cv::Mat resized, canvas(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::resize(bgr_img, resized, cv::Size(rw, rh));
    resized.copyTo(canvas(cv::Rect(pw, ph, rw, rh)));

    cv::Mat blob = cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(),
                                           cv::Scalar(), true, false);
    const float * ptr = blob.ptr<float>();
    std::vector<float> data(ptr, ptr + blob.total());
    return {data, {1, 3, input_h_, input_w_}, scale, pw, ph};
  }

  std::vector<Box> postprocess(std::vector<Ort::Value> & out, float scale,
                               int pad_w, int pad_h)
  {
    auto shape = out[0].GetTensorTypeAndShapeInfo().GetShape();
    int dets = (shape.size() >= 2) ? static_cast<int>(shape[1]) : 0;
    int feats = (shape.size() >= 3) ? static_cast<int>(shape[2]) : 6;
    const float * data = out[0].GetTensorData<float>();

    std::vector<Box> result;
    for (int i = 0; i < dets; ++i) {
      float conf = data[i * feats + 4];
      if (conf < conf_thresh_) continue;

      float x1 = (data[i * feats + 0] - pad_w) / scale;
      float y1 = (data[i * feats + 1] - pad_h) / scale;
      float x2 = (data[i * feats + 2] - pad_w) / scale;
      float y2 = (data[i * feats + 3] - pad_h) / scale;
      int cls = static_cast<int>(data[i * feats + 5]);

      result.push_back({
        cv::Rect(static_cast<int>(x1), static_cast<int>(y1),
                 static_cast<int>(x2 - x1), static_cast<int>(y2 - y1)),
        cls, conf
      });
    }
    return result;
  }
};

}  // anonymous namespace

namespace fs = std::filesystem;

std::string auto_detect_serial_port()
{
  if (!fs::is_directory("/dev")) return {};
  for (const auto & entry : fs::directory_iterator("/dev")) {
    auto name = entry.path().filename().string();
    if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0)
      return entry.path().string();
  }
  return {};
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(int argc, char ** argv)
{
  std::string model_path = "assets/color_track_model.onnx";
  std::string com_port = "/dev/ttyUSB0";
  std::vector<std::string> class_names = {"RED_LIGHT", "BLUE_LIGHT"};
  double exposure_ms = -1.0;
  double gamma = 0.5;
  double gain = -1.0;
  std::string vid_pid = "2ba2:4d55";
  std::string cam_config_path = "configs/daheng.yaml";
  float fov_h_deg = 60.0f;
  std::string config_path;
  bool exp_set = false, gain_set = false, gamma_set = false, vid_set = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-m" && i + 1 < argc)       model_path = argv[++i];
    else if (arg == "--names" && i + 1 < argc) {
      std::string raw = argv[++i];
      class_names.clear();
      size_t prev = 0, pos = 0;
      while ((pos = raw.find(',', prev)) != std::string::npos) {
        class_names.push_back(raw.substr(prev, pos - prev));
        prev = pos + 1;
      }
      class_names.push_back(raw.substr(prev));
    } else if (arg == "--config" && i + 1 < argc)
      config_path = argv[++i];
    else if (arg == "-p" && i + 1 < argc)  com_port = argv[++i];
    else if (arg == "--exposure" && i + 1 < argc) { exposure_ms = std::stod(argv[++i]); exp_set = true; }
    else if (arg == "--gamma" && i + 1 < argc)   { gamma = std::stod(argv[++i]); gamma_set = true; }
    else if (arg == "--gain" && i + 1 < argc)    { gain = std::stod(argv[++i]); gain_set = true; }
    else if (arg == "--vidpid" && i + 1 < argc)  { vid_pid = argv[++i]; vid_set = true; }
    else if (arg == "--cam-config" && i + 1 < argc) cam_config_path = argv[++i];
    else if (arg.find("--") != 0 && arg.find("-") != 0) com_port = arg;
  }

  if (!cam_config_path.empty()) {
    try {
      auto y = YAML::LoadFile(cam_config_path);
      if (!exp_set && y["exposure_ms"]) exposure_ms = y["exposure_ms"].as<double>();
      if (!gamma_set && y["gamma"]) gamma = y["gamma"].as<double>();
      if (!gain_set && y["gain"]) gain = y["gain"].as<double>();
      if (!vid_set && y["vid_pid"]) vid_pid = y["vid_pid"].as<std::string>();
    } catch (const std::exception &) {}
  }

  auto color_name = [&](int class_id) -> const std::string & {
    if (class_id < 0 || static_cast<size_t>(class_id) >= class_names.size()) {
      static const std::string unknown = "UNKNOWN";
      return unknown;
    }
    return class_names[class_id];
  };

  /* --- camera intrinsics --- */
  float cam_fx = 0, cam_fy = 0, cam_cx = 0, cam_cy = 0;
  bool intrinsics_loaded = false;

  if (!config_path.empty()) {
    try {
      auto yaml = YAML::LoadFile(config_path);
      if (yaml["camera_matrix"]) {
        auto data = yaml["camera_matrix"].as<std::vector<double>>();
        if (data.size() >= 6) {
          cam_fx = data[0];
          cam_fy = data[4];
          cam_cx = data[2];
          cam_cy = data[5];
          intrinsics_loaded = true;
        }
      }
    } catch (const std::exception & e) {
      std::cerr << "[WARN] Failed to load config: " << e.what() << std::endl;
    }
  }

  if (intrinsics_loaded) {
    std::cout << "Intrinsics: fx=" << cam_fx << " fy=" << cam_fy
              << " cx=" << cam_cx << " cy=" << cam_cy << std::endl;
  }

  std::cout << "=== Color Track Demo ===" << "\n"
            << "Model    : " << model_path << "\n"
            << "Config   : " << (config_path.empty() ? "(none, using FOV)" : config_path) << "\n"
            << "Port     : " << com_port << "\n"
            << "Classes  : ";
  for (size_t i = 0; i < class_names.size(); ++i)
    std::cout << (i ? ", " : "") << class_names[i];
  std::cout << "\nCamera   : Daheng (" << vid_pid << ")\n"
            << "CamCfg   : " << cam_config_path << "\n"
            << "Exposure : " << exposure_ms << " ms\n"
            << "Gain     : " << gain << " dB\n"
            << "FOV(H)   : " << fov_h_deg << " deg\n"
            << std::endl;

  /* --- YOLO (CUDA → CPU fallback) --- */
  YOLODetector detector(model_path, true, 0.5f);

  /* --- camera --- */
  io::DaHeng camera(exposure_ms, gamma, gain, vid_pid);

  /* --- gimbal (auto-detect port if needed) --- */
  if (!fs::exists(com_port)) {
    auto detected = auto_detect_serial_port();
    if (!detected.empty()) {
      std::cout << "Auto-detected serial port: " << detected << std::endl;
      com_port = detected;
    }
  }
  auto gimbal = std::make_unique<io::LaserGimbal>(com_port);
  if (!gimbal->is_open()) {
    std::cerr << "[ERROR] Cannot open gimbal on " << com_port << std::endl;
    return 1;
  }

  /* --- adaptive controller (inspired by Lracking) ---
   *  Sends ABSOLUTE target angles + velocity feedforward.
   *  Kp grows with error magnitude: farther from center = more aggressive.
   *  Lowpass on output, velocity ff from pixel motion.
   */
  const float kp_base = 1.5f;
  const float kp_extra = 3.0f;
  const float ki = 0.08f;
  const float kd = 0.12f;
  const float lowpass_alpha = 0.6f;

  auto prev_ts = std::chrono::steady_clock::now();
  float yaw_integral = 0, pitch_integral = 0;
  float yaw_prev_err = 0, pitch_prev_err = 0;
  float yaw_lp = 0, pitch_lp = 0;
  bool has_lp = false;
  float prev_px = 0, prev_py = 0;
  bool has_prev_uv = false;

  float fps = 0.0f;

  std::string disp_class = "WAITING";
  float disp_conf = 0;
  float disp_px_err = 0, disp_py_err = 0;
  float disp_yaw_err = 0, disp_pitch_err = 0;
  float disp_yaw_vel = 0, disp_pitch_vel = 0;
  float disp_kp = 0;

  const int cross_size = 15;

  std::cout << "=== Tracking started. Press Ctrl+C to stop. ===" << std::endl;

  while (true) {
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    camera.read(frame, ts);
    if (frame.empty()) continue;

    float dt = std::chrono::duration<float>(ts - prev_ts).count();
    prev_ts = ts;
    if (dt < 0.005f) dt = 0.033f;
    if (dt > 0.5f) dt = 0.5f;

    float instant_fps = 1.0f / dt;
    fps = 0.95f * fps + 0.05f * instant_fps;

    int img_w = frame.cols, img_h = frame.rows;
    float fx, fy, cx, cy;
    if (intrinsics_loaded) {
      fx = cam_fx; fy = cam_fy; cx = cam_cx; cy = cam_cy;
    } else {
      fx = static_cast<float>(img_w) / (2.0f * std::tan(fov_h_deg * CV_PI / 360.0f));
      fy = fx;
      cx = img_w / 2.0f;
      cy = img_h / 2.0f;
    }

    /* read current gimbal angles (rad) for absolute-angle command */
    io::LaserGimbalState gs = gimbal->read();

    auto dets = detector.detect(frame);

    float yaw_cmd = 0, pitch_cmd = 0;
    float yaw_vel = 0, pitch_vel = 0;

    if (!dets.empty()) {
      auto & best = dets[0];

      cv::rectangle(frame, best.bbox, cv::Scalar(0, 255, 0), 2);
      cv::putText(frame, color_name(best.class_id),
                  cv::Point(best.bbox.x, best.bbox.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

      float px = static_cast<float>(best.bbox.x + best.bbox.width / 2);
      float py = static_cast<float>(best.bbox.y + best.bbox.height / 2);

      cv::Scalar tgt_color(0, 0, 255);
      cv::line(frame, cv::Point(px - cross_size, py), cv::Point(px + cross_size, py), tgt_color, 2);
      cv::line(frame, cv::Point(px, py - cross_size), cv::Point(px, py + cross_size), tgt_color, 2);
      cv::circle(frame, cv::Point(px, py), 4, tgt_color, 2);

      /* pixel error from screen center */
      float du = px - cx;
      float dv = py - cy;

      /* angular error in camera frame (rad) */
      float yaw_err = -std::atan2(du, fx);
      float pitch_err = -std::atan2(dv, fy);

      /* adaptive Kp: farther from center = more aggressive
       *  pitch gets ×2 multiplier to compensate for gravity & weaker servo */
      float error_norm = std::sqrt(yaw_err * yaw_err + pitch_err * pitch_err);
      float kp_yaw = kp_base + kp_extra * error_norm;
      float kp_pitch = kp_base * 2.0f + kp_extra * error_norm;
      float kp = kp_yaw;

      /* PI */
      yaw_integral += yaw_err * dt;
      pitch_integral += pitch_err * dt;
      { float i_limit = 2.0f;
        yaw_integral = std::clamp(yaw_integral, -i_limit, i_limit);
        pitch_integral = std::clamp(pitch_integral, -i_limit, i_limit); }

      /* D */
      float yaw_d = (yaw_err - yaw_prev_err) / dt;
      float pitch_d = (pitch_err - pitch_prev_err) / dt;
      yaw_prev_err = yaw_err;
      pitch_prev_err = pitch_err;

      /* velocity = PID + pixel-motion feedforward */
      yaw_vel = kp_yaw * yaw_err + ki * yaw_integral + kd * yaw_d;
      pitch_vel = kp_pitch * pitch_err + ki * 2.0f * pitch_integral + kd * pitch_d;
      if (has_prev_uv && dt > 0.005f) {
        float ff_yaw = -std::atan2((px - prev_px) / dt, fx);
        float ff_pitch = -std::atan2((py - prev_py) / dt, fy);
        yaw_vel += 0.5f * ff_yaw;
        pitch_vel += 0.5f * ff_pitch;
      }

      prev_px = px;
      prev_py = py;
      has_prev_uv = true;

      /* absolute target angle = gimbal angle + error
       *  pitch ×1.5 to overcome gravity sag */
      float abs_yaw = gs.yaw + yaw_err;
      float abs_pitch = gs.pitch + pitch_err * 1.5f;

      /* lowpass on output */
      if (!has_lp) {
        yaw_lp = abs_yaw;
        pitch_lp = abs_pitch;
        has_lp = true;
      } else {
        yaw_lp = lowpass_alpha * abs_yaw + (1.0f - lowpass_alpha) * yaw_lp;
        pitch_lp = lowpass_alpha * abs_pitch + (1.0f - lowpass_alpha) * pitch_lp;
      }
      yaw_cmd = yaw_lp;
      pitch_cmd = pitch_lp;

      disp_class = color_name(best.class_id);
      disp_conf = best.confidence;
      disp_px_err = du;
      disp_py_err = dv;
      disp_yaw_err = yaw_err;
      disp_pitch_err = pitch_err;
      disp_yaw_vel = yaw_vel;
      disp_pitch_vel = pitch_vel;
      disp_kp = kp;

      std::cout << "\r[TRACK] " << color_name(best.class_id)
                << "  conf=" << best.confidence
                << "  err=(" << yaw_err << "," << pitch_err << ")"
                << "  kp=" << kp
                << "  vel=(" << yaw_vel << "," << pitch_vel << ")  "
                << std::flush;
    } else {
      disp_class = "LOST";
      disp_conf = 0;
      disp_px_err = 0;
      disp_py_err = 0;
      disp_yaw_err = 0;
      disp_pitch_err = 0;
      disp_yaw_vel = 0;
      disp_pitch_vel = 0;
      disp_kp = 0;

      yaw_integral = 0;
      pitch_integral = 0;
      yaw_prev_err = 0;
      pitch_prev_err = 0;
      has_prev_uv = false;

      yaw_cmd = has_lp ? yaw_lp : gs.yaw;
      pitch_cmd = has_lp ? pitch_lp : gs.pitch;
      yaw_vel = 0;
      pitch_vel = 0;

      std::cout << "\r[LOST] FPS:" << static_cast<int>(fps) << "  "
                << std::flush;
    }

    gimbal->send(1, yaw_cmd, yaw_vel, 0, pitch_cmd, pitch_vel, 0);

    cv::Scalar cam_color(0, 255, 0);
    cv::line(frame, cv::Point(cx - cross_size, cy), cv::Point(cx + cross_size, cy), cam_color, 2);
    cv::line(frame, cv::Point(cx, cy - cross_size), cv::Point(cx, cy + cross_size), cam_color, 2);
    cv::circle(frame, cv::Point(cx, cy), 4, cam_color, 2);

    {
      int base_x = 10, base_y = img_h - 120;
      cv::rectangle(frame, cv::Point(base_x - 4, base_y - 4),
                    cv::Point(base_x + 260, base_y + 132),
                    cv::Scalar(0, 0, 0, 128), -1);
      int ly = base_y;
      auto txt = [&](const std::string & s, const cv::Scalar & c) {
        cv::putText(frame, s, cv::Point(base_x, ly), cv::FONT_HERSHEY_SIMPLEX,
                    0.45, c, 1);
        ly += 18;
      };
      txt("FPS: " + std::to_string(static_cast<int>(fps)), cv::Scalar(0, 255, 255));
      txt(disp_class + "  " + std::to_string(static_cast<int>(disp_conf * 100)) + "%",
          disp_conf > 0.5f ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
      txt("Offset: (" + std::to_string(static_cast<int>(disp_px_err)) + ", "
                       + std::to_string(static_cast<int>(disp_py_err)) + ") px",
          cv::Scalar(180, 180, 255));
      char buf[128];
      std::snprintf(buf, sizeof(buf), "Yaw: %+.3f rad", disp_yaw_err);
      txt(buf, cv::Scalar(255, 200, 100));
      std::snprintf(buf, sizeof(buf), "Pitch: %+.3f rad", disp_pitch_err);
      txt(buf, cv::Scalar(255, 200, 100));
      std::snprintf(buf, sizeof(buf), "Vel: %+.2f | %+.2f rad/s", disp_yaw_vel, disp_pitch_vel);
      txt(buf, cv::Scalar(200, 200, 200));
      std::snprintf(buf, sizeof(buf), "Kp: %.1f", disp_kp);
      txt(buf, disp_kp > 4.0f ? cv::Scalar(0, 0, 255) : cv::Scalar(180, 255, 180));
    }

    cv::imshow("Color Track Demo", frame);
    int key = cv::waitKey(1);
    if (key == 27 || key == 'q') break;
  }

  std::cout << "\n=== Done ===" << std::endl;
  cv::destroyAllWindows();
  return 0;
}
