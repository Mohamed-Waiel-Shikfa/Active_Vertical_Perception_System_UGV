// Build-time check of the gimbal kinematics used by greenhouse::Gimbal (no Gazebo needed).
#include "GimbalMath.hh"
#include <cstdio>
#include <random>

using namespace greenhouse::gimbal;

static int failures = 0;
static void Expect(bool condition, const char *message, double a = 0, double b = 0)
{
  if (!condition) { std::fprintf(stderr, "FAIL: %s (%.6f vs %.6f)\n", message, a, b); ++failures; }
}
static double Deg(double value) { return value * kPi / 180; }

int main()
{
  std::mt19937 random(7);
  std::uniform_real_distribution<double> unit(-1, 1);
  const Limits limits;
  // 1. Inverse kinematics reproduces random reachable chains exactly.
  for (int i = 0; i < 20000; ++i)
  {
    const Joints joints{Deg(90)*unit(random), Deg(85)*unit(random), Deg(90)*unit(random)};
    const Quat mount = FromAttitude({Deg(15)*unit(random), Deg(15)*unit(random), kPi*unit(random)});
    const auto solved = Solve(mount, Multiply(mount, Chain(joints)));
    Expect(std::abs(solved.yaw - joints.yaw) < 1e-9 && std::abs(solved.roll - joints.roll) < 1e-9 &&
           std::abs(solved.pitch - joints.pitch) < 1e-9, "IK round trip", solved.pitch, joints.pitch);
  }
  // 2. Horizon lock: on a tilted robot the solved camera attitude equals the command
  //    (roll/pitch w.r.t. gravity, yaw w.r.t. the mount heading).
  for (int i = 0; i < 20000; ++i)
  {
    const Quat mount = FromAttitude({Deg(12)*unit(random), Deg(12)*unit(random), kPi*unit(random)});
    // Within +/-90 deg joints less the 12 deg tilt (plus coupling) the command must stay reachable.
    const Attitude command{Deg(25)*unit(random), Deg(70)*unit(random), Deg(60)*unit(random)};
    const auto desired = Desired(mount, command, true);
    auto joints = Solve(mount, desired);
    Expect(!Clamp(joints, limits), "reachable command saturated");
    Expect(Distance(Multiply(mount, Chain(joints)), desired) < 1e-9, "stabilised orientation");
    const auto measured = Measure(mount, Multiply(mount, Chain(joints)), true, command);
    Expect(std::abs(measured.roll - command.roll) < 1e-9, "stabilised roll", measured.roll, command.roll);
    Expect(std::abs(measured.pitch - command.pitch) < 1e-9, "stabilised pitch", measured.pitch, command.pitch);
    Expect(std::abs(Wrap(measured.yaw - command.yaw)) < 1e-9, "stabilised yaw", measured.yaw, command.yaw);
  }
  // 2b. A near-vertical look-down is reported in the commanded form.
  {
    const Quat mount = FromAttitude({Deg(3), Deg(-2), Deg(-90)});
    const Attitude command{0, Deg(-85), Deg(20)};
    auto joints = Solve(mount, Desired(mount, command, true));
    Expect(!Clamp(joints, limits), "look-down saturated");
    const auto measured = Measure(mount, Multiply(mount, Chain(joints)), true, command);
    Expect(std::abs(measured.pitch - command.pitch) < 1e-9 && std::abs(measured.roll) < 1e-9 &&
           std::abs(Wrap(measured.yaw - command.yaw)) < 1e-9, "look-down measurement", measured.pitch, command.pitch);
  }
  // 3. Level horizon on a 10 deg rolled + 8 deg pitched robot really needs non-zero joint angles.
  {
    const Quat mount = FromAttitude({Deg(10), Deg(8), Deg(90)});
    auto joints = Solve(mount, Desired(mount, {}, true));
    Expect(std::abs(joints.roll) > Deg(5) && std::abs(joints.pitch) > Deg(5), "tilt compensation");
    const auto world = ToAttitude(Multiply(mount, Chain(joints)));
    Expect(std::abs(world.roll) < 1e-9 && std::abs(world.pitch) < 1e-9, "level camera in world");
    Expect(std::abs(Wrap(world.yaw - Deg(90))) < 0.03, "follows outward heading", world.yaw, Deg(90));
  }
  // 4. FPV mode is rigid to the mount: joints equal the command for a single axis.
  {
    const Quat mount = FromAttitude({Deg(10), Deg(-5), Deg(-90)});
    const auto joints = Solve(mount, Desired(mount, {0, Deg(-30), 0}, false));
    Expect(std::abs(joints.pitch + Deg(30)) < 1e-9 && std::abs(joints.roll) < 1e-9, "FPV pitch", joints.pitch);
  }
  // 5. Mount recovered from the yaw link and joint angle.
  {
    const Quat mount = FromAttitude({Deg(4), Deg(-3), Deg(37)});
    const auto recovered = MountFromYawLink(Multiply(mount, Rz(Deg(70))), Deg(70));
    const auto a = ToMatrix(recovered), b = ToMatrix(mount);
    double error = 0;
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) error += std::abs(a[r][c] - b[r][c]);
    Expect(error < 1e-9, "mount recovery", error);
  }
  // 6. Out-of-range requests are clamped to the +/-90 deg operating range.
  {
    Joints joints{Deg(170), Deg(-100), Deg(95)};
    Expect(Clamp(joints, limits) && std::abs(joints.yaw - Deg(90)) < 1e-12 &&
           std::abs(joints.roll + Deg(90)) < 1e-12 && std::abs(joints.pitch - Deg(90)) < 1e-12, "clamp");
  }
  // 7. Closed loop with rate-limited motors keeps the horizon while the robot rocks at 1 Hz.
  {
    Joints q;
    const double dt = 0.002, gain = 40, maxRate = Deg(200);
    Joints previous; bool havePrevious = false;
    double worst = 0;
    for (int step = 0; step < 5000; ++step)
    {
      const double t = step * dt;
      const Quat mount = FromAttitude({Deg(6)*std::sin(2*kPi*t), Deg(4)*std::sin(2*kPi*0.7*t), Deg(90) + 0.3*t});
      auto target = Solve(mount, Desired(mount, {0, Deg(-10), 0}, true));
      Clamp(target, limits);
      Joints ff;
      if (havePrevious)
        ff = {Wrap(target.yaw - previous.yaw)/dt, (target.roll - previous.roll)/dt, (target.pitch - previous.pitch)/dt};
      previous = target; havePrevious = true;
      q.yaw += dt*Rate(target.yaw, q.yaw, ff.yaw, gain, maxRate);
      q.roll += dt*Rate(target.roll, q.roll, ff.roll, gain, maxRate);
      q.pitch += dt*Rate(target.pitch, q.pitch, ff.pitch, gain, maxRate);
      if (t > 1)
      {
        const auto measured = Measure(mount, Multiply(mount, Chain(q)), true);
        worst = std::max({worst, std::abs(measured.roll), std::abs(measured.pitch + Deg(10))});
      }
    }
    Expect(worst < Deg(0.1), "closed-loop horizon error under 0.1 deg", worst * 180 / kPi, 0.1);
    std::printf("closed-loop worst roll/pitch error: %.4f deg\n", worst * 180 / kPi);
  }
  if (failures) { std::fprintf(stderr, "%d gimbal kinematics checks failed\n", failures); return 1; }
  std::printf("PASS: gimbal IK, horizon lock, FPV mode, mount recovery, limits and closed loop.\n");
  return 0;
}
