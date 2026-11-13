// Headless Gazebo (DART) dynamics check of greenhouse::Gimbal on a rocking, lifting base.
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>
#include <cstdio>
#include <mutex>

static std::mutex mutex;
static QJsonObject gimbal, lift;
static double D(double v) { return v * M_PI / 180; }

int main(int argc, char **argv)
{
  gz::sim::ServerConfig config; config.SetSdfFile(argv[1]);
  gz::sim::Server server(config);
  gz::transport::Node node;
  node.Subscribe<gz::msgs::StringMsg>("/greenhouse/gimbal/state", std::function<void(const gz::msgs::StringMsg &)>([](const gz::msgs::StringMsg &m) {
    std::lock_guard<std::mutex> l(mutex); gimbal = QJsonDocument::fromJson(QByteArray::fromStdString(m.data())).object(); }));
  node.Subscribe<gz::msgs::StringMsg>("/greenhouse/lift/state", std::function<void(const gz::msgs::StringMsg &)>([](const gz::msgs::StringMsg &m) {
    std::lock_guard<std::mutex> l(mutex); lift = QJsonDocument::fromJson(QByteArray::fromStdString(m.data())).object(); }));
  auto yawPub = node.Advertise<gz::msgs::Double>("/test/rock_yaw");
  auto rollPub = node.Advertise<gz::msgs::Double>("/test/rock_roll");
  auto pitchPub = node.Advertise<gz::msgs::Double>("/test/rock_pitch");
  auto liftPub = node.Advertise<gz::msgs::Double>("/greenhouse/lift/velocity");
  auto send = [](auto &pub, double v) { gz::msgs::Double m; m.set_data(v); pub.Publish(m); };
  auto step = [&](int n) { server.Run(true, n, false); std::this_thread::sleep_for(std::chrono::milliseconds(3)); };
  auto camera = [&](int id) {
    std::lock_guard<std::mutex> l(mutex);
    for (const auto &c : gimbal["cameras"].toArray()) if (c.toObject()["camera"].toInt() == id) return c.toObject();
    return QJsonObject();
  };
  auto request = [&](const char *json) {
    gz::msgs::StringMsg req, rep; req.set_data(json); bool ok = false;
    node.Request("/greenhouse/gimbal/settings", req, 3000, rep, ok);
    return rep.data();
  };
  step(1500);  // 3 s settle
  for (int id : {1, 2})
  {
    const auto c = camera(id);
    std::printf("settled cam%d ready=%d measured r/p/y=%.3f %.3f %.3f joints y/r/p=%.2f %.2f %.2f err=%.3f deg\n", id,
      c["ready"].toBool(), c["measured"].toObject()["roll"].toDouble(), c["measured"].toObject()["pitch"].toDouble(),
      c["measured"].toObject()["yaw"].toDouble(), c["joints"].toObject()["yaw"].toDouble(),
      c["joints"].toObject()["roll"].toDouble(), c["joints"].toObject()["pitch"].toDouble(), c["error_deg"].toDouble());
  }
  // Rock: roll 8 deg @0.5 Hz, pitch 6 deg @0.35 Hz, turning at 0.3 rad/s; lift raising for part of it.
  double worst[3] = {0, 0, 0}, sumSq = 0; int samples = 0;
  for (int k = 0; k < 600; ++k)   // 600 x 20 ms = 12 s
  {
    const double t = k * 0.02;
    send(rollPub, D(8) * 2 * M_PI * 0.5 * std::cos(2 * M_PI * 0.5 * t));
    send(pitchPub, D(6) * 2 * M_PI * 0.35 * std::cos(2 * M_PI * 0.35 * t));
    send(yawPub, 0.3);
    send(liftPub, (t > 4 && t < 9) ? 1.0 : 0.0);
    step(10);
    if (t < 1.5) continue;
    for (int id : {1, 2})
    {
      const auto c = camera(id); const auto m = c["measured"].toObject();
      worst[0] = std::max(worst[0], std::abs(m["roll"].toDouble()));
      worst[1] = std::max(worst[1], std::abs(m["pitch"].toDouble()));
      worst[2] = std::max(worst[2], c["error_deg"].toDouble());
      if (k % 100 == 0) { const auto j = c["joints"].toObject(); std::printf("  rock t=%.1f cam%d joints y/r/p %.2f %.2f %.2f\n", t, id, j["yaw"].toDouble(), j["roll"].toDouble(), j["pitch"].toDouble()); }
      sumSq += c["error_deg"].toDouble() * c["error_deg"].toDouble(); ++samples;
    }
  }
  {
    std::lock_guard<std::mutex> l(mutex);
    std::printf("lift position after raise: %.3f m\n", lift["position"].toDouble());
  }
  std::printf("rocking+turning+lifting (both cams, stabilised 0/0/0): worst |roll| %.3f, |pitch| %.3f, attitude error %.3f deg, RMS %.3f deg\n",
              worst[0], worst[1], worst[2], std::sqrt(sumSq / samples));
  send(rollPub, 0); send(pitchPub, 0); send(yawPub, 0);
  for (int k = 0; k < 10; ++k) { step(50); const auto c = camera(2); const auto j = c["joints"].toObject();
    std::printf("  stop k=%d cam2 joints %.2f %.2f %.2f err %.3f\n", k, j["yaw"].toDouble(), j["roll"].toDouble(), j["pitch"].toDouble(), c["error_deg"].toDouble()); }
  { const auto c = camera(2); const auto j = c["joints"].toObject(); const auto tg = c["targets"].toObject();
    std::printf("before command cam2 joints %.2f %.2f %.2f targets %.2f %.2f %.2f err %.3f\n", j["yaw"].toDouble(), j["roll"].toDouble(), j["pitch"].toDouble(), tg["yaw"].toDouble(), tg["roll"].toDouble(), tg["pitch"].toDouble(), c["error_deg"].toDouble()); }
  std::printf("settings reply: %s\n", request(argv[2]).c_str());
  double settle = -1;
  for (int k = 0; k < 200; ++k)
  {
    step(10);
    const auto m = camera(2)["measured"].toObject();
    if (std::abs(m["pitch"].toDouble() + 30) < 0.5 && std::abs(m["yaw"].toDouble() - 20) < 0.5 &&
        std::abs(m["roll"].toDouble() - 10) < 0.5) { settle = (k + 1) * 0.02; break; }
  }
  for (int k = 0; k < 3; ++k) { step(100); const auto c = camera(2); const auto j = c["joints"].toObject(); const auto m = c["measured"].toObject();
    const auto tg = c["targets"].toObject(); std::printf("  target %.2f %.2f %.2f\n", tg["yaw"].toDouble(), tg["roll"].toDouble(), tg["pitch"].toDouble()); std::printf("  t+%.1fs joints y/r/p %.2f %.2f %.2f measured r/p/y %.2f %.2f %.2f err %.2f sat %d\n", 4+0.2*(k+1), j["yaw"].toDouble(), j["roll"].toDouble(), j["pitch"].toDouble(),
      m["roll"].toDouble(), m["pitch"].toDouble(), m["yaw"].toDouble(), c["error_deg"].toDouble(), c["saturated"].toBool()); }
  const auto c2 = camera(2);
  std::printf("cam2 step to r10/p-30/y20 reached within 0.5 deg after %.2f s; measured %.3f %.3f %.3f\n", settle,
    c2["measured"].toObject()["roll"].toDouble(), c2["measured"].toObject()["pitch"].toDouble(), c2["measured"].toObject()["yaw"].toDouble());
  std::printf("look-down reply: %s\n", request("{\"camera\":1,\"pitch\":-120}").c_str());
  step(1000);
  const auto c1 = camera(1);
  std::printf("cam1 look-down measured r/p/y %.3f %.3f %.3f sat=%d joints p=%.2f\n", c1["measured"].toObject()["roll"].toDouble(),
    c1["measured"].toObject()["pitch"].toDouble(), c1["measured"].toObject()["yaw"].toDouble(), c1["saturated"].toBool(),
    c1["joints"].toObject()["pitch"].toDouble());
  std::printf("FPV reply: %s\n", request("{\"camera\":1,\"pitch\":0,\"stabilise\":false}").c_str());
  send(rollPub, D(10)); step(50); send(rollPub, 0); step(500);   // tilt base by 10 deg roll
  const auto f = camera(1);
  std::printf("FPV cam1 measured (relative to robot) r/p/y %.3f %.3f %.3f\n", f["measured"].toObject()["roll"].toDouble(),
    f["measured"].toObject()["pitch"].toDouble(), f["measured"].toObject()["yaw"].toDouble());
  std::printf("reject reply: %s\n", request("{\"camera\":2,\"pitch\":-150}").c_str());
  std::printf("reject reply: %s\n", request("{\"camera\":3,\"pitch\":0}").c_str());
  return 0;
}
