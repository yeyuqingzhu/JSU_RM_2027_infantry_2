#include "laser_gimbal.hpp"

#include <algorithm>
#include <cmath>

namespace io
{
namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr size_t kPacketSize = 43;
constexpr size_t kStreamBufSize = 512;
constexpr float kMaxAngleDeg = 360.0f;
constexpr float kMaxVelDeg = 720.0f;
}  // namespace

LaserGimbal::LaserGimbal(const std::string & port, int baudrate)
{
  serial_.setPort(port);
  serial_.setBaudrate(baudrate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
  serial_.setTimeout(timeout);
  serial_.open();
  serial_.flushInput();
  serial_.flushOutput();

  read_thread_ = std::thread(&LaserGimbal::read_loop, this);
}

LaserGimbal::~LaserGimbal()
{
  quit_ = true;
  if (read_thread_.joinable()) read_thread_.join();
  if (serial_.isOpen()) serial_.close();
}

LaserGimbalState LaserGimbal::read()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

void LaserGimbal::send(uint8_t mode, float yaw, float yaw_vel, float yaw_acc, float pitch,
                       float pitch_vel, float pitch_acc)
{
  LaserVisionToGimbal pkt;
  pkt.mode = mode;
  pkt.yaw = static_cast<float>(yaw * kRadToDeg);
  pkt.yaw_vel = static_cast<float>(yaw_vel * kRadToDeg);
  pkt.yaw_acc = static_cast<float>(yaw_acc * kRadToDeg);
  pkt.pitch = static_cast<float>(pitch * kRadToDeg);
  pkt.pitch_vel = static_cast<float>(pitch_vel * kRadToDeg);
  pkt.pitch_acc = static_cast<float>(pitch_acc * kRadToDeg);

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
  } catch (const std::exception &) {
  }
}

void LaserGimbal::read_loop()
{
  uint8_t stream_buf[kStreamBufSize] = {0};
  size_t stream_len = 0;

  while (!quit_) {
    size_t nread = 0;
    try {
      size_t avail = serial_.available();
      if (avail > 0) {
        size_t room = kStreamBufSize - stream_len;
        if (room == 0) {
          std::memmove(stream_buf, stream_buf + kStreamBufSize - 64, 64);
          stream_len = 64;
          room = kStreamBufSize - stream_len;
        }
        size_t to_read = std::min(avail, room);
        nread = serial_.read(stream_buf + stream_len, to_read);
        stream_len += nread;
      }
    } catch (const std::exception &) {
    }

    if (nread == 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      continue;
    }

    while (stream_len >= kPacketSize) {
      size_t head_pos = static_cast<size_t>(-1);
      for (size_t i = 1; i < stream_len; ++i) {
        if (stream_buf[i - 1] == 0x5A && stream_buf[i] == 0xA5) {
          head_pos = i - 1;
          break;
        }
      }

      if (head_pos == static_cast<size_t>(-1)) {
        stream_buf[0] = stream_buf[stream_len - 1];
        stream_len = 1;
        break;
      }

      if (stream_len - head_pos < kPacketSize) {
        if (head_pos > 0) {
          std::memmove(stream_buf, stream_buf + head_pos, stream_len - head_pos);
          stream_len -= head_pos;
        }
        break;
      }

      uint8_t buffer[kPacketSize];
      std::memcpy(buffer, stream_buf + head_pos, kPacketSize);
      size_t consumed = head_pos + kPacketSize;
      if (stream_len > consumed) {
        std::memmove(stream_buf, stream_buf + consumed, stream_len - consumed);
        stream_len -= consumed;
      } else {
        stream_len = 0;
      }

      if (buffer[kPacketSize - 1] != 0xFE || buffer[kPacketSize - 2] != 0x7F) {
        continue;
      }

      LaserGimbalToVision frame;
      std::memcpy(&frame, buffer, kPacketSize);

      float yaw_deg = frame.yaw;
      float pitch_deg = frame.pitch;
      float yaw_vel_deg = frame.yaw_vel;
      float pitch_vel_deg = frame.pitch_vel;

      if (std::abs(yaw_deg) > kMaxAngleDeg || std::abs(pitch_deg) > kMaxAngleDeg ||
          std::abs(yaw_vel_deg) > kMaxVelDeg || std::abs(pitch_vel_deg) > kMaxVelDeg) {
        continue;
      }

      LaserGimbalState s;
      s.mode = frame.mode;
      std::memcpy(s.q, frame.q, sizeof(s.q));
      s.yaw = static_cast<float>(yaw_deg * kDegToRad);
      s.yaw_vel = static_cast<float>(yaw_vel_deg * kDegToRad);
      s.pitch = static_cast<float>(pitch_deg * kDegToRad);
      s.pitch_vel = static_cast<float>(pitch_vel_deg * kDegToRad);
      s.bullet_speed = frame.bullet_speed;
      s.bullet_count = frame.bullet_count;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = s;
      }
      has_state_ = true;
    }
  }
}

}  // namespace io
