#include "io/gimbal/laser_gimbal.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? | | 打印命令行参数说明}"
  "{p              | /dev/ttyUSB0 | 串口路径}"
  "{b              | 115200 | 波特率}"
  "{@port          | | 串口路径（位置参数，优先级高于 -p） }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto port = cli.has("@port") ? cli.get<std::string>("@port") : cli.get<std::string>("p");
  auto baudrate = cli.get<int>("b");

  tools::logger()->info("LaserGimbal test: port={}, baudrate={}", port, baudrate);

  tools::Exiter exiter;

  io::LaserGimbal gimbal(port, baudrate);

  if (!gimbal.is_open()) {
    tools::logger()->error("Failed to open laser gimbal on {}", port);
    return 1;
  }

  auto t0 = std::chrono::steady_clock::now();
  auto last_print = t0;

  float yaw_cmd = 0.0f;
  float pitch_cmd = 0.0f;

  while (!exiter.exit()) {
    auto state = gimbal.read();

    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration<double>(now - last_print).count();

    if (dt >= 1.0) {
      double yaw_deg = state.yaw * 180.0 / M_PI;
      double pitch_deg = state.pitch * 180.0 / M_PI;
      tools::logger()->info(
        "[状态] mode={}  yaw={:.3f}°({:.3f}rad)  pitch={:.3f}°({:.3f}rad)  "
        "yaw_vel={:.3f}  pitch_vel={:.3f}  bullet={}",
        static_cast<int>(state.mode),
        yaw_deg, state.yaw,
        pitch_deg, state.pitch,
        state.yaw_vel, state.pitch_vel,
        state.bullet_count);
      last_print = now;
    }

    // 每2秒发一个正弦波扫过 ±30°
    auto elapsed = std::chrono::duration<double>(now - t0).count();
    yaw_cmd = 0.5 * std::sin(elapsed * 0.5);
    pitch_cmd = 0.15 * std::sin(elapsed * 0.3 + 1.0);

    // mode=1: 跟踪但不开火
    gimbal.send(1, yaw_cmd, 0, 0, pitch_cmd, 0, 0);

    std::this_thread::sleep_for(10ms);
  }

  // 停止云台
  gimbal.send(0, 0, 0, 0, 0, 0, 0);
  tools::logger()->info("LaserGimbal test finished.");

  return 0;
}
