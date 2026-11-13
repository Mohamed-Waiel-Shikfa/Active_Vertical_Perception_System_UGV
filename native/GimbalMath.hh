// Dependency-free kinematics for the two simulated XF C-200T 3-axis gimbals.
//
// Joint chain from the mount (fixed to the lift carriage) outwards: yaw about +Z, roll about +X,
// pitch about -Y (a positive pitch angle tilts the camera up). The mount frame's +X is the
// camera's neutral, outward view direction. Quaternions are (w, x, y, z), Hamilton convention.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace greenhouse::gimbal
{
constexpr double kPi = 3.14159265358979323846;

struct Quat { double w{1}, x{0}, y{0}, z{0}; };
using Matrix = std::array<std::array<double, 3>, 3>;

inline Quat Multiply(const Quat &a, const Quat &b)
{
  return {a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
          a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
          a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
          a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w};
}
inline Quat Conjugate(const Quat &q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quat AxisAngle(double x, double y, double z, double angle)
{
  const double s = std::sin(angle / 2);
  return {std::cos(angle / 2), x*s, y*s, z*s};
}
inline Quat Rz(double a) { return AxisAngle(0, 0, 1, a); }
inline Quat Rx(double a) { return AxisAngle(1, 0, 0, a); }
inline Quat Ry(double a) { return AxisAngle(0, 1, 0, a); }

inline Matrix ToMatrix(Quat q)
{
  const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  q = {q.w/n, q.x/n, q.y/n, q.z/n};
  return {{{1 - 2*(q.y*q.y + q.z*q.z), 2*(q.x*q.y - q.z*q.w), 2*(q.x*q.z + q.y*q.w)},
           {2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z), 2*(q.y*q.z - q.x*q.w)},
           {2*(q.x*q.z - q.y*q.w), 2*(q.y*q.z + q.x*q.w), 1 - 2*(q.x*q.x + q.y*q.y)}}};
}

inline double Wrap(double angle)
{
  return std::remainder(angle, 2 * kPi);
}

// Camera attitude in the user's convention: roll (+ lowers right side), pitch (+ up), yaw (+ CCW).
struct Attitude { double roll{0}, pitch{0}, yaw{0}; };
// Joint angles of the yaw, roll and pitch motors.
struct Joints { double yaw{0}, roll{0}, pitch{0}; };
// Joint ranges in Joints order (yaw, roll, pitch); the plugin loads the configured values from SDF.
struct Limits { double lower[3]{-kPi/2, -kPi/2, -kPi/2};
                double upper[3]{kPi/2, kPi/2, kPi/2}; };

inline Quat FromAttitude(const Attitude &a)
{
  // Intrinsic yaw, then pitch (about -Y so that positive is up), then roll.
  return Multiply(Multiply(Rz(a.yaw), Ry(-a.pitch)), Rx(a.roll));
}
inline Attitude ToAttitude(const Quat &q)
{
  const auto m = ToMatrix(q);
  return {std::atan2(m[2][1], m[2][2]), std::asin(std::clamp(m[2][0], -1.0, 1.0)),
          std::atan2(m[1][0], m[0][0])};
}
// Orientation produced by the joint chain, relative to the mount.
inline Quat Chain(const Joints &j)
{
  return Multiply(Multiply(Rz(j.yaw), Rx(j.roll)), Ry(-j.pitch));
}
// The yaw link's frame is the mount frame turned by the yaw joint angle.
inline Quat MountFromYawLink(const Quat &yawLinkWorld, double yawJoint)
{
  return Multiply(yawLinkWorld, Rz(-yawJoint));
}
// Heading of the mount's +X axis projected onto the horizontal plane.
inline double Heading(const Quat &mount)
{
  const auto m = ToMatrix(mount);
  return std::atan2(m[1][0], m[0][0]);
}
// Stabilised (horizon-lock): roll and pitch are referenced to gravity and yaw to the mount's
// heading, so the camera follows the robot's turns but not its roll/pitch on uneven ground.
// Unstabilised (FPV): the whole command is relative to the mount, i.e. rigidly to the robot.
inline Quat Desired(const Quat &mount, const Attitude &command, bool stabilise)
{
  if (!stabilise) return Multiply(mount, FromAttitude(command));
  Attitude world = command;
  world.yaw = Heading(mount) + command.yaw;
  return FromAttitude(world);
}
// Every attitude has a second Euler triple (pitch beyond +/-90 deg); report the one nearest the
// command so that e.g. a -120 deg look-down is not displayed as roll 180 / pitch -60 / yaw +180.
inline Attitude Nearest(const Attitude &a, const Attitude &reference)
{
  const Attitude alternate{Wrap(a.roll + kPi), (a.pitch < 0 ? -kPi : kPi) - a.pitch, Wrap(a.yaw + kPi)};
  const auto distance = [&](const Attitude &value) {
    return std::abs(Wrap(value.roll - reference.roll)) + std::abs(value.pitch - reference.pitch) +
           std::abs(Wrap(value.yaw - reference.yaw));
  };
  return distance(alternate) < distance(a) ? alternate : a;
}
// Measured camera attitude in the same reference the command uses.
inline Attitude Measure(const Quat &mount, const Quat &camera, bool stabilise, const Attitude &reference = {})
{
  if (!stabilise) return Nearest(ToAttitude(Multiply(Conjugate(mount), camera)), reference);
  auto attitude = ToAttitude(camera);
  attitude.yaw = Wrap(attitude.yaw - Heading(mount));
  return Nearest(attitude, reference);
}
// Angle between two orientations.
inline double Distance(const Quat &a, const Quat &b)
{
  const auto d = Multiply(Conjugate(a), b);
  return 2 * std::atan2(std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z), std::abs(d.w));
}
// Inverse kinematics of R = Rz(yaw) Rx(roll) Ry(-pitch); unique for |roll| < 90 deg.
inline Joints Solve(const Quat &mount, const Quat &desired)
{
  const auto m = ToMatrix(Multiply(Conjugate(mount), desired));
  return {std::atan2(-m[0][1], m[1][1]), std::asin(std::clamp(m[2][1], -1.0, 1.0)),
          -std::atan2(-m[2][0], m[2][2])};
}
// Clamp to the mechanical range; returns true when any axis had to be limited.
inline bool Clamp(Joints &j, const Limits &limits)
{
  double *values[3] = {&j.yaw, &j.roll, &j.pitch};
  bool saturated = false;
  for (int i = 0; i < 3; ++i)
  {
    const double clamped = std::clamp(*values[i], limits.lower[i], limits.upper[i]);
    saturated |= std::abs(clamped - *values[i]) > 1e-3;
    *values[i] = clamped;
  }
  return saturated;
}
// Motor velocity command: proportional position loop plus target-rate feed-forward,
// bounded by the vendor's 200 deg/s maximum control speed.
inline double Rate(double target, double position, double feedForward, double gain, double maxRate)
{
  return std::clamp(gain * (target - position) + std::clamp(feedForward, -maxRate, maxRate),
                    -maxRate, maxRate);
}
}
