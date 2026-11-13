#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityLimitsCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>
#include "GimbalMath.hh"

namespace greenhouse
{
using namespace gz;
using Clock = std::chrono::steady_clock;

static std::string Json(const QJsonObject &object)
{
  return QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
}

class Actuator : public sim::System, public sim::ISystemConfigure,
                 public sim::ISystemPreUpdate, public sim::ISystemPostUpdate
{
  transport::Node node;
  transport::Node::Publisher publisher;
  sim::Entity model{sim::kNullEntity}, joint{sim::kNullEntity};
  std::string jointName, prefix;
  std::mutex mutex;
  double stroke{1.5}, speed{0.1}, maxSpeed{0.2}, ramp{0.5}, forceLimit{30}, movingMass{0.44};
  double lower{0}, upper{1.5}, target{0}, reference{0}, velocityRef{0}, direction{0};
  double q{0}, dq{0}, force{0}, nextStatus{0};
  bool ready{false}, blocked{false};
  Clock::time_point lastInput{Clock::now()};
  Clock::time_point lastStatusWall{};
  QJsonObject status;

  bool Settings(const msgs::StringMsg &request, msgs::StringMsg &response)
  {
    QJsonParseError parse;
    const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(request.data()), &parse);
    std::lock_guard<std::mutex> lock(mutex);
    auto fail = [&](const char *message) {
      response.set_data(Json({{"ok", false}, {"error", message}})); return true;
    };
    if (parse.error != QJsonParseError::NoError || !document.isObject())
      return fail("Settings must be a JSON object");
    if (!ready) return fail("Actuator state is not ready");
    const auto object = document.object();
    for (const auto &key : object.keys())
      if (key != "lower" && key != "upper" && key != "speed" && key != "target" && key != "blocked")
        return fail("Unknown actuator setting");
    for (const auto &key : {"lower", "upper", "speed", "target"})
      if (object.contains(key) && (!object[key].isDouble() || !std::isfinite(object[key].toDouble())))
        return fail("Actuator settings must be finite numbers");
    if (object.contains("blocked") && !object["blocked"].isBool())
      return fail("blocked must be boolean");
    const double lo = object["lower"].toDouble(lower), hi = object["upper"].toDouble(upper);
    const double vel = object["speed"].toDouble(speed), goal = object["target"].toDouble(target);
    if (lo < 0 || hi > stroke || hi - lo < 0.01) return fail("Window must be within the installed hard stroke");
    if (q < lo - 0.0005 || q > hi + 0.0005) return fail("Move carriage inside the new window before applying it");
    if (vel < 0.005 || vel > maxSpeed) return fail("Speed exceeds the vertical payload envelope");
    if (object.contains("target") && (goal < lo || goal > hi)) return fail("Target is outside the operating window");
    lower = lo; upper = hi; speed = vel;
    target = std::clamp(goal, lower, upper);
    direction = 0;
    if (object.contains("blocked"))
    {
      blocked = object["blocked"].toBool();
      target = std::clamp(q, lower, upper); reference = target; velocityRef = 0;
    }
    response.set_data(Json({{"ok", true}, {"lower", lower}, {"upper", upper}, {"speed", speed}}));
    return true;
  }

  void Input(const msgs::Double &message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!std::isfinite(message.data()) || std::abs(message.data()) > 1)
    {
      gzerr << "Invalid actuator direction; stopping manual motion\n";
      direction = 0;
      return;
    }
    if (blocked) return;
    direction = message.data();
    lastInput = Clock::now();
    if (direction != 0) target = direction > 0 ? upper : lower;
    else target = std::clamp(reference, lower, upper);
  }

public:
  void Configure(const sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager &, sim::EventManager &) override
  {
    model = entity;
    jointName = sdf->Get<std::string>("joint_name");
    prefix = sdf->Get<std::string>("prefix");
    stroke = sdf->Get<double>("stroke"); upper = stroke;
    speed = sdf->Get<double>("speed"); maxSpeed = sdf->Get<double>("max_speed");
    ramp = sdf->Get<double>("ramp_time"); forceLimit = sdf->Get<double>("force_limit");
    movingMass = sdf->Get<double>("moving_mass");
    node.Subscribe(prefix + "/lift/velocity", &Actuator::Input, this);
    node.Advertise(prefix + "/lift/settings", &Actuator::Settings, this);
    publisher = node.Advertise<msgs::StringMsg>(prefix + "/lift/state");
  }

  void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override
  {
    if (joint == sim::kNullEntity)
    {
      joint = sim::Model(model).JointByName(ecm, jointName);
      if (joint == sim::kNullEntity) return;
      ecm.CreateComponent(joint, sim::components::JointPosition());
      ecm.CreateComponent(joint, sim::components::JointVelocity());
      ecm.CreateComponent(joint, sim::components::JointVelocityLimitsCmd(
        {{-maxSpeed, maxSpeed}}));
    }
    const auto position = ecm.Component<sim::components::JointPosition>(joint);
    const auto velocity = ecm.Component<sim::components::JointVelocity>(joint);
    if (!position || !velocity || position->Data().empty() || velocity->Data().empty()) return;
    std::lock_guard<std::mutex> lock(mutex);
    q = position->Data()[0]; dq = velocity->Data()[0]; ready = true;
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0 || dt > 0.1) return;
    if (direction != 0 && Clock::now() - lastInput > std::chrono::milliseconds(350))
    {
      direction = 0; target = std::clamp(reference, lower, upper);
    }
    if (blocked) { direction = 0; target = std::clamp(reference, lower, upper); }
    const double acceleration = speed / ramp;
    const double error = target - reference;
    const double desired = std::copysign(std::min(speed, std::sqrt(2 * acceleration * std::abs(error))), error);
    velocityRef += std::clamp(desired - velocityRef, -acceleration * dt, acceleration * dt);
    double next = reference + velocityRef * dt;
    if ((target - reference) * (target - next) <= 0) { next = target; velocityRef = 0; }
    reference = std::clamp(next, lower, upper);
    force = std::clamp(movingMass * 9.81 + 800 * (reference - q) + 60 * (velocityRef - dq),
                       -forceLimit, forceLimit);
    ecm.SetComponentData<sim::components::JointForceCmd>(joint, {force});
  }

  void PostUpdate(const sim::UpdateInfo &info, const sim::EntityComponentManager &) override
  {
    std::unique_lock<std::mutex> lock(mutex);
    const double time = std::chrono::duration<double>(info.simTime).count();
    if (!info.paused && time < nextStatus) return;
    const auto wall=Clock::now();
    if(info.paused && wall-lastStatusWall<std::chrono::milliseconds(50)) return;
    lastStatusWall=wall;
    nextStatus = time + 0.05;
    status = {{"ready", ready}, {"position", q}, {"velocity", dq}, {"force", force},
              {"stroke", stroke}, {"lower", lower}, {"upper", upper}, {"speed", speed},
              {"max_speed", maxSpeed}, {"target", target}, {"force_limit", forceLimit},
              {"blocked", blocked}, {"sim_time", time}};
    msgs::StringMsg message; message.set_data(Json(status));
    lock.unlock();
    publisher.Publish(message);
  }
};

// Two XF C-200T 3-axis gimbals. Each motor is a velocity servo bounded by the URDF effort limit
// (DART SERVO actuator) and by the vendor's 200 deg/s control speed. Targets come from the inverse
// kinematics in GimbalMath.hh using the true simulated link attitude (ideal attitude sensing).
// Commands and joint targets are limited to the <roll|pitch|yaw>_limits ranges from SDF (degrees).
class Gimbal : public sim::System, public sim::ISystemConfigure,
               public sim::ISystemPreUpdate, public sim::ISystemPostUpdate
{
  struct Unit
  {
    int id{0};
    std::string jointNames[3], yawLinkName, cameraLinkName;
    sim::Entity joints[3]{sim::kNullEntity, sim::kNullEntity, sim::kNullEntity};
    sim::Entity yawLink{sim::kNullEntity}, cameraLink{sim::kNullEntity};
    gimbal::Attitude command;
    bool stabilise{true};
    gimbal::Joints position, target, previousTarget;
    bool havePrevious{false}, saturated{false}, ready{false};
    gimbal::Attitude measured;
    double error{0};
  };
  transport::Node node;
  transport::Node::Publisher publisher;
  sim::Entity model{sim::kNullEntity};
  std::string prefix;
  std::mutex mutex;
  std::vector<Unit> units;
  gimbal::Limits limits;
  double maxRate{200 * gimbal::kPi / 180}, gain{40}, nextStatus{0};
  Clock::time_point lastStatusWall{};

  static gimbal::Quat ToQuat(const math::Quaterniond &q) { return {q.W(), q.X(), q.Y(), q.Z()}; }
  static double Degrees(double radians) { return radians * 180 / gimbal::kPi; }

  // {"camera": 1|2, "roll"/"pitch"/"yaw": degrees, "stabilise": bool}; omitted fields are kept.
  std::string Apply(const std::string &text)
  {
    QJsonParseError parse;
    const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(text), &parse);
    auto fail = [](const char *message) { return Json({{"ok", false}, {"error", message}}); };
    if (parse.error != QJsonParseError::NoError || !document.isObject())
      return fail("Gimbal settings must be a JSON object");
    const auto object = document.object();
    for (const auto &key : object.keys())
      if (key != "camera" && key != "roll" && key != "pitch" && key != "yaw" && key != "stabilise")
        return fail("Unknown gimbal setting");
    if (!object["camera"].isDouble()) return fail("camera must be 1 or 2");
    const int id = object["camera"].toInt();
    std::lock_guard<std::mutex> lock(mutex);
    auto unit = std::find_if(units.begin(), units.end(), [&](const Unit &u) { return u.id == id; });
    if (unit == units.end()) return fail("camera must be 1 or 2");
    const char *keys[3] = {"roll", "pitch", "yaw"};
    // Joints order is yaw, roll, pitch; commands are roll, pitch, yaw.
    const double lower[3] = {Degrees(limits.lower[1]), Degrees(limits.lower[2]), Degrees(limits.lower[0])};
    const double upper[3] = {Degrees(limits.upper[1]), Degrees(limits.upper[2]), Degrees(limits.upper[0])};
    double values[3] = {Degrees(unit->command.roll), Degrees(unit->command.pitch), Degrees(unit->command.yaw)};
    for (int i = 0; i < 3; ++i)
    {
      if (!object.contains(keys[i])) continue;
      if (!object[keys[i]].isDouble() || !std::isfinite(object[keys[i]].toDouble()))
        return fail("Gimbal angles must be finite numbers in degrees");
      values[i] = object[keys[i]].toDouble();
      if (values[i] < lower[i] - 1e-9 || values[i] > upper[i] + 1e-9)
        return Json({{"ok", false}, {"error", QString("%1 must be within %2..%3 deg").arg(keys[i])
                                                .arg(lower[i]).arg(upper[i])}});
    }
    if (object.contains("stabilise") && !object["stabilise"].isBool())
      return fail("stabilise must be boolean");
    unit->command = {values[0] * gimbal::kPi / 180, values[1] * gimbal::kPi / 180, values[2] * gimbal::kPi / 180};
    unit->stabilise = object["stabilise"].toBool(unit->stabilise);
    unit->havePrevious = false;
    return Json({{"ok", true}, {"camera", id}, {"roll", values[0]}, {"pitch", values[1]},
                 {"yaw", values[2]}, {"stabilise", unit->stabilise}});
  }
  bool Settings(const msgs::StringMsg &request, msgs::StringMsg &response)
  {
    response.set_data(Apply(request.data())); return true;
  }
  void Command(const msgs::StringMsg &message)
  {
    const auto reply = Apply(message.data());
    if (reply.find("\"ok\":false") != std::string::npos) gzerr << "Gimbal command rejected: " << reply << "\n";
  }

public:
  void Configure(const sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager &, sim::EventManager &) override
  {
    model = entity;
    prefix = sdf->Get<std::string>("prefix");
    if (sdf->HasElement("max_rate")) maxRate = sdf->Get<double>("max_rate");
    const char *axes[3] = {"yaw_limits", "roll_limits", "pitch_limits"};
    for (int i = 0; i < 3; ++i)
      if (sdf->HasElement(axes[i]))
      {
        const auto range = sdf->Get<math::Vector2d>(axes[i]);
        limits.lower[i] = range.X() * gimbal::kPi / 180; limits.upper[i] = range.Y() * gimbal::kPi / 180;
      }
    auto copy = sdf->Clone();
    for (auto element = copy->HasElement("gimbal") ? copy->GetElement("gimbal") : nullptr; element;
         element = element->GetNextElement("gimbal"))
    {
      Unit unit;
      unit.id = element->Get<int>("id");
      unit.jointNames[0] = element->Get<std::string>("yaw_joint");
      unit.jointNames[1] = element->Get<std::string>("roll_joint");
      unit.jointNames[2] = element->Get<std::string>("pitch_joint");
      unit.yawLinkName = element->Get<std::string>("yaw_link");
      unit.cameraLinkName = element->Get<std::string>("camera_link");
      unit.command = {element->Get<double>("roll") * gimbal::kPi / 180,
                      element->Get<double>("pitch") * gimbal::kPi / 180,
                      element->Get<double>("yaw") * gimbal::kPi / 180};
      unit.stabilise = element->Get<bool>("stabilise");
      units.push_back(unit);
    }
    if (units.empty()) gzerr << "greenhouse::Gimbal has no <gimbal> elements\n";
    node.Advertise(prefix + "/gimbal/settings", &Gimbal::Settings, this);
    node.Subscribe(prefix + "/gimbal/command", &Gimbal::Command, this);
    publisher = node.Advertise<msgs::StringMsg>(prefix + "/gimbal/state");
  }

  void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    const double dt = std::chrono::duration<double>(info.dt).count();
    for (auto &unit : units)
    {
      if (!unit.ready)
      {
        bool found = true;
        for (int i = 0; i < 3; ++i)
        {
          unit.joints[i] = sim::Model(model).JointByName(ecm, unit.jointNames[i]);
          if (unit.joints[i] == sim::kNullEntity) { found = false; continue; }
          if (!ecm.Component<sim::components::JointPosition>(unit.joints[i]))
            ecm.CreateComponent(unit.joints[i], sim::components::JointPosition());
          if (!ecm.Component<sim::components::JointVelocity>(unit.joints[i]))
            ecm.CreateComponent(unit.joints[i], sim::components::JointVelocity());
        }
        unit.yawLink = sim::Model(model).LinkByName(ecm, unit.yawLinkName);
        unit.cameraLink = sim::Model(model).LinkByName(ecm, unit.cameraLinkName);
        if (!found || unit.yawLink == sim::kNullEntity || unit.cameraLink == sim::kNullEntity) continue;
      }
      double q[3];
      bool valid = true;
      for (int i = 0; i < 3; ++i)
      {
        const auto component = ecm.Component<sim::components::JointPosition>(unit.joints[i]);
        if (!component || component->Data().empty()) { valid = false; break; }
        q[i] = component->Data()[0];
      }
      if (!valid) continue;
      unit.ready = true;
      unit.position = {q[0], q[1], q[2]};
      const auto mount = gimbal::MountFromYawLink(ToQuat(sim::worldPose(unit.yawLink, ecm).Rot()), q[0]);
      const auto desired = gimbal::Desired(mount, unit.command, unit.stabilise);
      unit.target = gimbal::Solve(mount, desired);
      unit.saturated = gimbal::Clamp(unit.target, limits);
      const auto camera = ToQuat(sim::worldPose(unit.cameraLink, ecm).Rot());
      unit.measured = gimbal::Measure(mount, camera, unit.stabilise, unit.command);
      unit.error = gimbal::Distance(camera, desired);
      if (info.paused || dt <= 0 || dt > 0.1) { unit.havePrevious = false; continue; }
      gimbal::Joints feedForward;
      if (unit.havePrevious)
        feedForward = {gimbal::Wrap(unit.target.yaw - unit.previousTarget.yaw) / dt,
                       (unit.target.roll - unit.previousTarget.roll) / dt,
                       (unit.target.pitch - unit.previousTarget.pitch) / dt};
      unit.previousTarget = unit.target; unit.havePrevious = true;
      const double rates[3] = {
        gimbal::Rate(unit.target.yaw, q[0], feedForward.yaw, gain, maxRate),
        gimbal::Rate(unit.target.roll, q[1], feedForward.roll, gain, maxRate),
        gimbal::Rate(unit.target.pitch, q[2], feedForward.pitch, gain, maxRate)};
      for (int i = 0; i < 3; ++i)
        ecm.SetComponentData<sim::components::JointVelocityCmd>(unit.joints[i], {rates[i]});
    }
  }

  void PostUpdate(const sim::UpdateInfo &info, const sim::EntityComponentManager &) override
  {
    std::unique_lock<std::mutex> lock(mutex);
    const double time = std::chrono::duration<double>(info.simTime).count();
    if (!info.paused && time < nextStatus) return;
    const auto wall = Clock::now();
    if (info.paused && wall - lastStatusWall < std::chrono::milliseconds(100)) return;
    lastStatusWall = wall;
    nextStatus = time + 0.05;
    QJsonArray cameras;
    bool ready = !units.empty();
    for (const auto &unit : units)
    {
      ready &= unit.ready;
      cameras.append(QJsonObject{
        {"camera", unit.id}, {"ready", unit.ready}, {"stabilise", unit.stabilise},
        {"command", QJsonObject{{"roll", Degrees(unit.command.roll)}, {"pitch", Degrees(unit.command.pitch)},
                                {"yaw", Degrees(unit.command.yaw)}}},
        {"measured", QJsonObject{{"roll", Degrees(unit.measured.roll)}, {"pitch", Degrees(unit.measured.pitch)},
                                 {"yaw", Degrees(unit.measured.yaw)}}},
        {"joints", QJsonObject{{"yaw", Degrees(unit.position.yaw)}, {"roll", Degrees(unit.position.roll)},
                               {"pitch", Degrees(unit.position.pitch)}}},
        {"targets", QJsonObject{{"yaw", Degrees(unit.target.yaw)}, {"roll", Degrees(unit.target.roll)},
                                {"pitch", Degrees(unit.target.pitch)}}},
        {"error_deg", Degrees(unit.error)}, {"saturated", unit.saturated}});
    }
    msgs::StringMsg message;
    const QJsonObject range{
      {"roll", QJsonArray{Degrees(limits.lower[1]), Degrees(limits.upper[1])}},
      {"pitch", QJsonArray{Degrees(limits.lower[2]), Degrees(limits.upper[2])}},
      {"yaw", QJsonArray{Degrees(limits.lower[0]), Degrees(limits.upper[0])}}};
    message.set_data(Json({{"ready", ready}, {"sim_time", time}, {"cameras", cameras}, {"limits", range}}));
    lock.unlock();
    publisher.Publish(message);
  }
};

class WorldState : public sim::System, public sim::ISystemConfigure, public sim::ISystemPreUpdate,
                   public sim::ISystemPostUpdate
{
  transport::Node node;
  std::mutex mutex;
  std::string cached{"{}"};
  sim::Entity base{sim::kNullEntity};
  sim::Entity robotEntity{sim::kNullEntity};
  transport::Node::Publisher statePublisher;
  double nextState{0};
  double lastStateTime{-1};
  Clock::time_point lastStateWall{};
  uint64_t stateSequence{0};
  std::vector<std::pair<sim::Entity, sim::Entity>> wheels;
  Clock::time_point lastPublish{};
  bool Read(const msgs::Empty &, msgs::StringMsg &response)
  {
    std::lock_guard<std::mutex> lock(mutex); response.set_data(cached); return true;
  }
public:
  void Configure(const sim::Entity &, const std::shared_ptr<const sdf::Element> &,
                 sim::EntityComponentManager &, sim::EventManager &) override
  {
    node.Advertise("/greenhouse/world/status", &WorldState::Read, this);
    statePublisher = node.Advertise<msgs::StringMsg>("/greenhouse/physics");
  }
  void PreUpdate(const sim::UpdateInfo &, sim::EntityComponentManager &ecm) override
  {
    if (base != sim::kNullEntity) return;
    ecm.Each<sim::components::Model, sim::components::Name>(
      [&](const sim::Entity &entity, const sim::components::Model *, const sim::components::Name *name) {
        const auto text = name->Data();
        if (text == "robot" || (text.size() > 6 && text.substr(text.size()-6) == "/robot"))
        {
          robotEntity = entity;
          base = sim::Model(entity).LinkByName(ecm, "base_link");
          if (base != sim::kNullEntity) sim::Link(base).EnableVelocityChecks(ecm);
          for (const auto &name : {"front_left", "front_right", "rear_left", "rear_right"})
          {
            const auto link = sim::Model(entity).LinkByName(ecm, std::string(name) + "_wheel_link");
            const auto joint = sim::Model(entity).JointByName(ecm, std::string(name) + "_wheel_joint");
            if (link != sim::kNullEntity && joint != sim::kNullEntity)
            {
              sim::Link(link).EnableVelocityChecks(ecm);
              if (!ecm.Component<sim::components::JointVelocity>(joint))
                ecm.CreateComponent(joint, sim::components::JointVelocity());
              wheels.emplace_back(link, joint);
            }
          }
        }
        return true;
      });
  }
  void PostUpdate(const sim::UpdateInfo &info, const sim::EntityComponentManager &ecm) override
  {
    const double time = std::chrono::duration<double>(info.simTime).count();
    const auto wall=Clock::now();
    const bool simulationDue=time+1e-9>=nextState;
    const bool wallDue=wall-lastStateWall>=std::chrono::milliseconds(100);
    if (base != sim::kNullEntity && !info.paused && time>lastStateTime && (simulationDue || wallDue))
    {
      const auto linear = sim::Link(base).WorldLinearVelocity(ecm);
      const auto angular = sim::Link(base).WorldAngularVelocity(ecm);
      if (linear && angular)
      {
        if(simulationDue) nextState=time+.02;
        lastStateTime=time;
        lastStateWall=wall;
        const auto pose = sim::worldPose(base, ecm);
        const auto bodyLinear = pose.Rot().RotateVectorReverse(*linear);
        const auto bodyAngular = pose.Rot().RotateVectorReverse(*angular);
        QJsonArray measurements;
        for (const auto &[link, joint] : wheels)
        {
          const auto spin = ecm.Component<sim::components::JointVelocity>(joint);
          const auto command = ecm.Component<sim::components::JointVelocityCmd>(joint);
          const auto wheelVelocity = sim::Link(link).WorldLinearVelocity(ecm);
          if (!spin || spin->Data().empty() || !wheelVelocity) continue;
          const auto center = pose.Rot().RotateVectorReverse(sim::worldPose(link, ecm).Pos() - pose.Pos());
          const auto centerVelocity = pose.Rot().RotateVectorReverse(*wheelVelocity);
          measurements.append(QJsonObject{
            {"name", QString::fromStdString(ecm.Component<sim::components::Name>(link)->Data())},
            {"omega", spin->Data()[0]},
            {"command_omega",command && !command->Data().empty() ? QJsonValue(command->Data()[0]) : QJsonValue()},
            {"x", center.X()}, {"y", center.Y()}, {"z", center.Z()},
            {"vx", centerVelocity.X()}, {"vy", centerVelocity.Y()}, {"vz", centerVelocity.Z()}});
        }
        QJsonObject state{{"sim_time", time}, {"iteration", static_cast<double>(info.iterations)},
          {"publication_sequence",static_cast<double>(++stateSequence)},
          {"publication_trigger",simulationDue ? "simulation_period" : "fresh_step_wall_period"},
          {"paused", info.paused}, {"source", "Gazebo PostUpdate physical base_link"},
          {"base_entity", static_cast<double>(base)}, {"robot_entity", static_cast<double>(robotEntity)},
          {"x", pose.Pos().X()}, {"y", pose.Pos().Y()}, {"z", pose.Pos().Z()},
          {"roll", pose.Rot().Roll()}, {"pitch", pose.Rot().Pitch()}, {"yaw", pose.Rot().Yaw()},
          {"vx", bodyLinear.X()}, {"vy", bodyLinear.Y()}, {"vz", bodyLinear.Z()},
          {"wx", bodyAngular.X()}, {"wy", bodyAngular.Y()}, {"wz", bodyAngular.Z()},
          {"nav_vx", std::cos(pose.Rot().Yaw())*linear->X()+std::sin(pose.Rot().Yaw())*linear->Y()},
          {"nav_vy", -std::sin(pose.Rot().Yaw())*linear->X()+std::cos(pose.Rot().Yaw())*linear->Y()},
          {"nav_wz", (std::sin(pose.Rot().Roll())*bodyAngular.Y()+std::cos(pose.Rot().Roll())*bodyAngular.Z())/
                     std::cos(pose.Rot().Pitch())},
          {"wheels", measurements}};
        msgs::StringMsg message; message.set_data(Json(state)); statePublisher.Publish(message);
      }
    }
    if (Clock::now() - lastPublish < std::chrono::milliseconds(40)) return;
    lastPublish = Clock::now();
    QJsonArray grounds;
    QJsonObject robot;
    ecm.Each<sim::components::Model, sim::components::Name>(
      [&](const sim::Entity &entity, const sim::components::Model *, const sim::components::Name *name) {
        const auto text = name->Data();
        const auto pose = sim::worldPose(entity, ecm);
        if (text == "robot" || (text.size() > 6 && text.substr(text.size()-6) == "/robot"))
        {
          robot = {{"name", QString::fromStdString(text)}, {"id", static_cast<double>(entity)},
                   {"x", pose.Pos().X()}, {"y", pose.Pos().Y()}, {"z", pose.Pos().Z()},
                   {"roll", pose.Rot().Roll()}, {"pitch", pose.Rot().Pitch()}, {"yaw", pose.Rot().Yaw()}};
          if (base != sim::kNullEntity)
          {
            const auto linear = sim::Link(base).WorldLinearVelocity(ecm);
            const auto angular = sim::Link(base).WorldAngularVelocity(ecm);
            if (linear && angular)
            { robot["speed"] = linear->Length(); robot["angular_speed"] = angular->Length(); }
          }
        }
        if (text.rfind("greenhouse_ground", 0) != 0) return true;
        QJsonObject ground{{"name", QString::fromStdString(text)}, {"id", static_cast<double>(entity)}};
        for (const auto link : ecm.ChildrenByComponents(entity, sim::components::Link()))
        {
          for (const auto shape : ecm.ChildrenByComponents(link, sim::components::Collision()))
          {
            const auto geometry = ecm.Component<sim::components::Geometry>(shape);
            if (geometry && geometry->Data().MeshShape())
              ground["collision_uri"] = QString::fromStdString(geometry->Data().MeshShape()->Uri());
          }
          for (const auto shape : ecm.ChildrenByComponents(link, sim::components::Visual()))
          {
            const auto geometry = ecm.Component<sim::components::Geometry>(shape);
            if (geometry && geometry->Data().MeshShape())
              ground["visual_uri"] = QString::fromStdString(geometry->Data().MeshShape()->Uri());
          }
        }
        grounds.append(ground); return true;
      });
    std::lock_guard<std::mutex> lock(mutex);
    cached = Json({{"paused", info.paused},
                   {"sim_time", std::chrono::duration<double>(info.simTime).count()},
                   {"iteration", static_cast<double>(info.iterations)}, {"grounds", grounds}, {"robot", robot}});
  }
};
}
GZ_ADD_PLUGIN(greenhouse::Actuator, gz::sim::System, greenhouse::Actuator::ISystemConfigure,
              greenhouse::Actuator::ISystemPreUpdate, greenhouse::Actuator::ISystemPostUpdate)
GZ_ADD_PLUGIN(greenhouse::Gimbal, gz::sim::System, greenhouse::Gimbal::ISystemConfigure,
              greenhouse::Gimbal::ISystemPreUpdate, greenhouse::Gimbal::ISystemPostUpdate)
GZ_ADD_PLUGIN(greenhouse::WorldState, gz::sim::System, greenhouse::WorldState::ISystemConfigure,
              greenhouse::WorldState::ISystemPreUpdate,
              greenhouse::WorldState::ISystemPostUpdate)
