#ifndef IO__LASER_GIMBAL_HPP
#define IO__LASER_GIMBAL_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "serial/serial.h"

namespace io
{
/*
 * Protocol: binary frame, 0x5A 0xA5 head, 0x7F 0xFE tail.
 * Units in the serial frame: DEGREES for angles, deg/s for velocities.
 * This matches the STM32 firmware in the Lracking laser-tracking gimbal project.
 * The API below converts to/from radians for consistency with the rest of sp_vision_25.
 */

/* STM32 → NUC: 43 bytes */
struct __attribute__((packed)) LaserGimbalToVision
{
  uint8_t head[2] = {0x5A, 0xA5};
  uint8_t mode;
  float q[4];
  float yaw;         // deg
  float yaw_vel;     // deg/s
  float pitch;       // deg
  float pitch_vel;   // deg/s
  float bullet_speed;
  uint16_t bullet_count;
  uint8_t tail[2] = {0x7F, 0xFE};
};
static_assert(sizeof(LaserGimbalToVision) == 43);

/* NUC → STM32: 29 bytes */
struct __attribute__((packed)) LaserVisionToGimbal
{
  uint8_t head[2] = {0x5A, 0xA5};
  uint8_t mode;
  float yaw;         // deg, ABSOLUTE (not delta)
  float yaw_vel;     // deg/s feedforward
  float yaw_acc;     // deg/s² feedforward
  float pitch;       // deg, ABSOLUTE
  float pitch_vel;   // deg/s feedforward
  float pitch_acc;   // deg/s² feedforward
  uint8_t tail[2] = {0x7F, 0xFE};
};
static_assert(sizeof(LaserVisionToGimbal) == 29);

struct LaserGimbalState
{
  uint8_t mode;
  float q[4];
  float yaw;       // rad
  float yaw_vel;   // rad/s
  float pitch;     // rad
  float pitch_vel; // rad/s
  float bullet_speed;
  uint16_t bullet_count;
};

class LaserGimbal
{
public:
  LaserGimbal(const std::string & port, int baudrate = 115200);
  ~LaserGimbal();

  LaserGimbalState read();
  void send(uint8_t mode, float yaw, float yaw_vel, float yaw_acc, float pitch,
            float pitch_vel, float pitch_acc);

  bool is_open() const { return serial_.isOpen(); }

private:
  serial::Serial serial_;
  std::thread read_thread_;
  std::atomic<bool> quit_{false};
  mutable std::mutex mutex_;

  LaserGimbalState state_{};
  std::atomic<bool> has_state_{false};

  void read_loop();
  bool find_and_sync_frame(uint8_t * buffer, size_t & stream_len, uint8_t * stream_buf,
                           size_t stream_cap);
};

}  // namespace io

#endif  // IO__LASER_GIMBAL_HPP
