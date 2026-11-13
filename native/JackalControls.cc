#include "JackalControls.hh"
#include <QCoreApplication>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QTest>
#include <algorithm>
#include <cmath>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>

using namespace gz;

QImage CameraProvider::requestImage(const QString &id, QSize *size, const QSize &)
{
  const int camera = id.section('/', 0, 0).toInt();
  std::lock_guard<std::mutex> lock(mutex);
  const QImage image = camera >= 1 && camera <= kCameraCount ? images[camera-1] : QImage();
  if (size) *size = image.size();
  return image;
}
void CameraProvider::Set(int camera, const QImage &value)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (camera >= 1 && camera <= kCameraCount) images[camera-1] = value;
}

JackalControls::JackalControls()
{
  connect(&timer, &QTimer::timeout, this, &JackalControls::Tick);
  connect(&terrainProcess, &QProcess::readyReadStandardOutput, this, [this] {
    const QString output = QString::fromUtf8(terrainProcess.readAllStandardOutput()).trimmed();
    if (!output.isEmpty()) { status = output.split('\n').last(); emit changed(); }
  });
  connect(&navigationProcess,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,
          [this](int code,QProcess::ExitStatus) {
    if(code!=0) navigationStatus=QString::fromUtf8(navigationProcess.readAllStandardError()).trimmed();
    emit navigationChanged();
  });
  connect(&terrainProcess, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
    [this](int code, QProcess::ExitStatus) {
      busy = false;
      if (code != 0)
        status = "Terrain not applied: " + QString::fromUtf8(terrainProcess.readAllStandardError()).trimmed().right(300);
      else { ReadParameters(); if(parametersLoaded) status = "Applied revision "+parameterRevision+" (session only)"; }
      emit changed();
    });
  connect(&terrainProcess, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error == QProcess::FailedToStart) { busy = false; status = terrainProcess.errorString(); emit changed(); }
  });
}

JackalControls::~JackalControls()
{
  Stop();
  heartbeatRunning = false;
  if (heartbeat.joinable()) heartbeat.join();
  timer.stop();
  node.Unsubscribe(prefix + "/lift/state");
  for (int camera = 1; camera <= kCameraCount; ++camera)
    node.Unsubscribe(prefix + "/camera" + std::to_string(camera) + "/image");
  node.Unsubscribe(prefix + "/gimbal/state");
  node.Unsubscribe(prefix + "/camera/toggle");
  if (window) window->removeEventFilter(this);
}

void JackalControls::LoadConfig(const tinyxml2::XMLElement *element)
{
  title = "Drive";
  const auto text = [&](const char *key, const char *fallback) {
    const auto entry = element ? element->FirstChildElement(key) : nullptr;
    return std::string(entry && entry->GetText() ? entry->GetText() : fallback);
  };
  prefix = text("prefix", "/greenhouse");
  groundRms = QString::fromStdString(text("ground_rms_mm", "16")).toInt();
  driveTopic = text("drive_topic", "/cmd_vel");
  testOutput = text("test_output", "");
  testPhase = text("test_phase", "full");
  BindQmlContext(Context());
  ReadParameters();
  window = gui::App()->findChild<gui::MainWindow *>()->QuickWindow();
  window->installEventFilter(this);
  connect(window,&QQuickWindow::activeChanged,this,[this] { if(!window->isActive()) ClearInput(); });
  drivePublisher = node.Advertise<msgs::Twist>(driveTopic);
  liftPublisher = node.Advertise<msgs::Double>(prefix + "/lift/velocity");
  overridePublisher = node.Advertise<msgs::Empty>(prefix + "/manual_override");
  gimbalPublisher = node.Advertise<msgs::StringMsg>(prefix + "/gimbal/command");
  node.Subscribe(prefix + "/lift/state", &JackalControls::ReceiveState, this);
  for (int camera = 1; camera <= kCameraCount; ++camera)
    node.Subscribe<msgs::Image>(prefix + "/camera" + std::to_string(camera) + "/image",
      [this, camera](const msgs::Image &image) { ReceiveImage(camera, image); });
  node.Subscribe(prefix + "/gimbal/state", &JackalControls::ReceiveGimbal, this);
  node.Subscribe<msgs::Empty>(prefix + "/camera/toggle", [this](const msgs::Empty &) {
    QMetaObject::invokeMethod(this, [this] { ToggleCamera(); }, Qt::QueuedConnection);
  });
  provider = new CameraProvider();
  gui::App()->Engine()->addImageProvider("gimbal_cameras", provider);
  heartbeatRunning = true;
  heartbeat = std::thread(&JackalControls::Heartbeat, this);
  timer.start(50);
  if (!testOutput.empty()) QTimer::singleShot(1000, this, &JackalControls::RunGuiChecks);
}

void JackalControls::ReceiveState(const msgs::StringMsg &message)
{
  const auto object = QJsonDocument::fromJson(QByteArray::fromStdString(message.data())).object();
  QMetaObject::invokeMethod(this, [this, object] {
    state = object;
    active = object["ready"].toBool() &&
      QFile::exists(qEnvironmentVariable("GREENHOUSE_STATE") + "/controller-ready.json");
    position = object["position"].toDouble(); velocity = object["velocity"].toDouble();
    stroke = object["stroke"].toDouble(); liftSpeed = object["speed"].toDouble();
    maxLiftSpeed = object["max_speed"].toDouble();
    lastState.restart();
    if (status == "Waiting for actuator" && active) status = "Ready. Click the scene, then use the keys.";
    emit changed();
  }, Qt::QueuedConnection);
}

void JackalControls::ReceiveImage(int camera, const msgs::Image &message)
{
  if (message.pixel_format_type() != msgs::PixelFormatType::RGB_INT8 ||
      message.data().size() < static_cast<size_t>(message.height()) * message.step()) return;
  const QImage image(reinterpret_cast<const uchar *>(message.data().data()),
                     message.width(), message.height(), message.step(), QImage::Format_RGB888);
  const QImage copy = image.copy();
  QMetaObject::invokeMethod(this, [this, camera, copy] {
    if (!provider) return;
    provider->Set(camera, copy); lastFrames[camera-1].restart(); ++imageCounts[camera-1]; ++imageCount;
    if (view == camera) { ++frameVersion; emit frameChanged(); }
    if (imageCounts[camera-1] == 1) emit changed();
  }, Qt::QueuedConnection);
}

void JackalControls::ReceiveGimbal(const msgs::StringMsg &message)
{
  const auto object = QJsonDocument::fromJson(QByteArray::fromStdString(message.data())).object();
  QMetaObject::invokeMethod(this, [this, object] {
    gimbals = object["cameras"].toArray().toVariantList();
    gimbalReady = object["ready"].toBool();
    gimbalLimits = object["limits"].toObject().toVariantMap();
    for (const auto &entry : gimbals)
    {
      const auto camera = entry.toMap();
      const int id = camera["camera"].toInt();
      if (id < 1 || id > kCameraCount || gimbalSteering[id-1]) continue;
      const auto command = camera["command"].toMap();
      gimbalTargets[id-1][0] = command["roll"].toDouble();
      gimbalTargets[id-1][1] = command["pitch"].toDouble();
      gimbalTargets[id-1][2] = command["yaw"].toDouble();
    }
    lastGimbal.restart();
    emit gimbalChanged();
  }, Qt::QueuedConnection);
}

void JackalControls::BindOverlay()
{
  if (overlay || !window) return;
  QList<QQuickItem *> items{window->contentItem()};
  for (int i = 0; i < items.size(); ++i)
  {
    auto item = items[i];
    items.append(item->childItems());
    if (item->objectName() == "rw")
    {
      auto context = new QQmlContext(gui::App()->Engine()->rootContext(), this);
      BindQmlContext(context);
      QQmlComponent component(gui::App()->Engine(), QUrl("qrc:/CameraOverlay.qml"));
      auto created = qobject_cast<QQuickItem *>(component.create(context));
      if (!created) { status = component.errorString(); emit changed(); return; }
      created->setParentItem(item->parentItem()); created->setParent(this); created->setZ(100);
      overlay = created; return;
    }
  }
}

bool JackalControls::Editing() const
{
  if (!window) return false;
  for (auto focus = window->activeFocusItem(); focus; focus = focus->parentItem())
  {
    const QString type = focus->metaObject()->className();
    if (type.contains("TextInput") || type.contains("TextEdit") || type.contains("SpinBox") ||
        type.contains("ComboBox")) return true;
  }
  return false;
}

bool JackalControls::eventFilter(QObject *object, QEvent *event)
{
  if (event->type() == QEvent::MouseButtonPress && overlay)
  {
    const auto mouse = static_cast<QMouseEvent *>(event);
    auto scene = overlay->parentItem();
    if (scene && scene->contains(scene->mapFromScene(mouse->windowPos())))
      (view > 0 ? overlay.data() : scene)->forceActiveFocus(Qt::MouseFocusReason);
  }
  if (event->type() == QEvent::WindowDeactivate || event->type() == QEvent::FocusOut)
    ClearInput();
  if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease)
    return QObject::eventFilter(object, event);
  auto key = static_cast<QKeyEvent *>(event);
  const int value = key->key();
  if (value != Qt::Key_Up && value != Qt::Key_Down && value != Qt::Key_Left &&
      value != Qt::Key_Right && value != Qt::Key_A && value != Qt::Key_Q &&
      value != Qt::Key_C && value != Qt::Key_Space && value != Qt::Key_Escape &&
      value != Qt::Key_P && value != Qt::Key_R && value != Qt::Key_Y)
    return QObject::eventFilter(object, event);
  if (Editing()) { ClearInput(); return QObject::eventFilter(object, event); }
  if (event->type()==QEvent::KeyPress && keyboard && !busy &&
      (value==Qt::Key_Up || value==Qt::Key_Down || value==Qt::Key_Left || value==Qt::Key_Right))
    OverrideNavigation();
  if (event->type() == QEvent::KeyPress && !key->isAutoRepeat() &&
      (value == Qt::Key_P || value == Qt::Key_R || value == Qt::Key_Y))
    gimbalKeyModes[value] = {key->modifiers() & Qt::AltModifier ? 2 : 1,
                             key->modifiers() & Qt::ShiftModifier ? -1.0 : 1.0};
  if (value == Qt::Key_C)
  {
    if (event->type() == QEvent::KeyPress && !key->isAutoRepeat() && !keys.contains(value))
      ToggleCamera();
  }
  if (event->type() == QEvent::KeyRelease)
  {
    if (!key->isAutoRepeat()) keys.remove(value);
  }
  else if (value == Qt::Key_Space || value == Qt::Key_Escape) Stop();
  else if (keyboard && !busy) keys.insert(value);
  Tick();
  return true;
}

void JackalControls::Publish(double forward, double yaw, double lift)
{
  std::lock_guard<std::mutex> lock(commandMutex);
  wantedForward = forward; wantedYaw = yaw; wantedLift = lift;
  commandLive = true; commandUpdated = std::chrono::steady_clock::now();
  commandSimTime = state["sim_time"].toDouble();
  lastLift = lift;
}

void JackalControls::Heartbeat()
{
  while (heartbeatRunning)
  {
    {
      std::lock_guard<std::mutex> lock(commandMutex);
      const bool live = commandLive &&
        std::chrono::steady_clock::now() - commandUpdated < std::chrono::milliseconds(1500);
      const double forward = live ? wantedForward : 0, yaw = live ? wantedYaw : 0;
      const double lift = live ? wantedLift : 0;
      if (forward != 0 || yaw != 0 || sentForward != 0 || sentYaw != 0)
      {
        msgs::Twist command; command.mutable_linear()->set_x(forward); command.mutable_angular()->set_z(yaw);
        drivePublisher.Publish(command);
      }
      if (lift != 0 || sentLift != 0)
      {
        msgs::Double command; command.set_data(lift); liftPublisher.Publish(command);
      }
      sentForward = forward; sentYaw = yaw; sentLift = lift;
      if (!testOutput.empty() && (forward != 0 || yaw != 0 || lift != 0))
      {
        commandTrace.append(QJsonObject{{"t", commandSimTime}, {"forward", forward}, {"yaw", yaw},
                                       {"lift", lift}, {"live_ui", live},
          {"sent_at_monotonic_ms", static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count())}});
        if (commandTrace.size() > 3000) commandTrace.removeFirst();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void JackalControls::Tick()
{
  BindOverlay();
  if(!navigationPoll.isValid() || navigationPoll.elapsed()>250)
  {
    navigationPoll.restart();
    QFile file(NavigationStatePath());
    if(file.open(QIODevice::ReadOnly))
    {
      const auto object=QJsonDocument::fromJson(file.readAll()).object();
      navigationReady=object["ready"].toBool();navigating=object["active"].toBool();
      navigationStatus=object["message"].toString();
      if(!object["persistence_error"].isNull() && !object["persistence_error"].isUndefined())
        navigationStatus+="; evidence export error: "+object["persistence_error"].toString();
      if(QDateTime::currentMSecsSinceEpoch()/1000.0-object["wall_time"].toDouble()>2)
      {navigationReady=false;navigating=false;navigationStatus="Navigation feedback unavailable";}
      if(navigating && object.contains("xy_error"))
        navigationStatus+=QString(" - %1 m / %2 deg").arg(object["xy_error"].toDouble(),0,'f',2)
                          .arg(object["yaw_error"].toDouble()*180/3.141592653589793,0,'f',1);
      emit navigationChanged();
    }
    else {navigationReady=false;navigating=false;navigationStatus="Waiting for navigation";emit navigationChanged();}
    // A self_driving script may Apply terrain; refresh the fields so a later GUI Apply cannot revert it.
    const auto stamp=QFileInfo(qEnvironmentVariable("GREENHOUSE_STATE")+"/terrain-state.json").lastModified();
    if(!busy && stamp.isValid() && stamp!=terrainStamp)
    { ReadParameters(); if(parametersLoaded) {status="Terrain changed by a script: revision "+parameterRevision;emit changed();} }
  }
  if (lastState.isValid() && lastState.elapsed() > 3000) active = false;
  if (!window || !window->isActive() || !keyboard || !active || busy || state["blocked"].toBool() || Editing())
  { if (!keys.isEmpty() || lastLift != 0) ClearInput(); gimbalClock.invalidate(); return; }
  const double forward = (keys.contains(Qt::Key_Up) - keys.contains(Qt::Key_Down)) * driveSpeed;
  const double yaw = (keys.contains(Qt::Key_Left) - keys.contains(Qt::Key_Right)) * turnSpeed;
  const double lift = keys.contains(Qt::Key_A) - keys.contains(Qt::Key_Q);
  Publish(forward, yaw, lift);
  SteerGimbals();
}

// Held P / R / Y raise pitch / roll / yaw of camera 1 at gimbalKeySpeed deg/s; Alt selects camera 2 and
// Shift lowers instead (both read when the key goes down). Targets stop at the controller's limits.
void JackalControls::SteerGimbals()
{
  // Real held time, so the rate holds when software rendering slows the GUI loop to a few Hz;
  // capped so a frozen GUI cannot release one large jump.
  const double dt = gimbalClock.isValid() ? std::min(gimbalClock.restart() / 1000.0, 0.5) : 0;
  if (!gimbalClock.isValid()) gimbalClock.start();
  const char *axes[3] = {"roll", "pitch", "yaw"};
  bool steering[kCameraCount]{};
  for (auto mode = gimbalKeyModes.cbegin(); mode != gimbalKeyModes.cend(); ++mode)
  {
    if (!keys.contains(mode.key())) continue;
    const int axis = mode.key() == Qt::Key_R ? 0 : mode.key() == Qt::Key_P ? 1 : 2;
    const auto range = gimbalLimits[axes[axis]].toList();
    if (range.size() != 2) continue;  // no gimbal state received yet
    double &target = gimbalTargets[mode->first - 1][axis];
    target = std::clamp(target + mode->second * gimbalKeySpeed * dt, range[0].toDouble(), range[1].toDouble());
    steering[mode->first - 1] = true;
  }
  for (int camera = 1; camera <= kCameraCount; ++camera)
  {
    gimbalSteering[camera-1] = steering[camera-1];
    if (!steering[camera-1] || !gimbalPublisher.Valid()) continue;
    const auto &target = gimbalTargets[camera-1];
    msgs::StringMsg command;
    command.set_data(QJsonDocument(QJsonObject{{"camera", camera}, {"roll", target[0]}, {"pitch", target[1]},
                                               {"yaw", target[2]}}).toJson(QJsonDocument::Compact).toStdString());
    gimbalPublisher.Publish(command);
  }
}

void JackalControls::Stop()
{
  ClearInput();
  OverrideNavigation();
  msgs::Twist drive;drivePublisher.Publish(drive);
  msgs::Double lift;lift.set_data(0);liftPublisher.Publish(lift);
}

void JackalControls::ClearInput()
{
  keys.clear();
  std::lock_guard<std::mutex> lock(commandMutex);
  const bool baseMoving=sentForward!=0 || sentYaw!=0 || wantedForward!=0 || wantedYaw!=0;
  const bool liftMoving=sentLift!=0 || wantedLift!=0;
  commandLive = false; wantedForward = wantedYaw = wantedLift = 0;
  if (drivePublisher.Valid())
  {
    if(baseMoving) { msgs::Twist drive;drivePublisher.Publish(drive); }
    if(liftMoving) { msgs::Double lift;lift.set_data(0);liftPublisher.Publish(lift); }
  }
  sentForward = sentYaw = 0; sentLift = 0;
  lastLift = 0;
}
void JackalControls::OverrideNavigation()
{
  if(overridePublisher.Valid()) {msgs::Empty request;overridePublisher.Publish(request);}
}
void JackalControls::SetKeyboardEnabled(bool value) { ClearInput(); keyboard = value; emit changed(); }
void JackalControls::SetDriveSpeed(double value)
{ if (std::isfinite(value) && value >= 0 && value <= 2) driveSpeed = value; emit changed(); }
void JackalControls::SetGimbalKeySpeed(double value)
{ if (std::isfinite(value) && value >= 0.1 && value <= 200) gimbalKeySpeed = value; emit changed(); }
void JackalControls::SetTurnSpeed(double value)
{ if (std::isfinite(value) && value >= 0 && value <= 4) turnSpeed = value; emit changed(); }
void JackalControls::ButtonKey(int key, bool pressed)
{
  if (busy || !active) return;
  if(pressed && (key==Qt::Key_Up || key==Qt::Key_Down || key==Qt::Key_Left || key==Qt::Key_Right))
    OverrideNavigation();
  if (pressed) keys.insert(key); else keys.remove(key);
  keyboard = true; Tick(); emit changed();
}

// C cycles 3D view -> camera 1 -> camera 2 -> 3D view; a camera without live frames is skipped.
void JackalControls::ToggleCamera()
{
  for (int step = 1; step <= kCameraCount; ++step)
  {
    const int next = (view + step) % (kCameraCount + 1);
    if (next == 0 || CameraAvailable(next)) { SelectView(next); return; }
  }
  SelectView(0);
  status = "Waiting for gimbal camera images"; emit changed();
}

void JackalControls::SelectView(int index)
{
  BindOverlay();
  if (index < 0 || index > kCameraCount) return;
  if (!overlay) { status = "3D viewport not ready for camera view"; emit changed(); return; }
  if (index > 0 && !CameraAvailable(index))
  { status = QString("Waiting for camera %1 images").arg(index); emit changed(); return; }
  view = index;
  (view > 0 ? overlay.data() : overlay->parentItem())->forceActiveFocus(Qt::OtherFocusReason);
  status = view == 0 ? "Normal 3D view (C: camera 1)"
         : view == 1 ? "Camera 1 - left gimbal (C: camera 2)" : "Camera 2 - right gimbal (C: 3D view)";
  ++frameVersion; emit frameChanged();
  emit changed();
}

bool JackalControls::ApplyGimbal(int camera, double roll, double pitch, double yaw, bool stabilise)
{
  msgs::StringMsg request, response;
  request.set_data(QJsonDocument(QJsonObject{{"camera", camera}, {"roll", roll}, {"pitch", pitch},
      {"yaw", yaw}, {"stabilise", stabilise}}).toJson(QJsonDocument::Compact).toStdString());
  bool result = false;
  const bool received = node.Request(prefix + "/gimbal/settings", request, 2000, response, result);
  const auto object = QJsonDocument::fromJson(QByteArray::fromStdString(response.data())).object();
  if (!received || !result || !object["ok"].toBool())
  { status = object["error"].toString("Gimbal request timed out"); emit changed(); return false; }
  status = QString("Camera %1 gimbal: roll %2, pitch %3, yaw %4 deg (%5)").arg(camera)
             .arg(roll, 0, 'f', 1).arg(pitch, 0, 'f', 1).arg(yaw, 0, 'f', 1)
             .arg(stabilise ? "horizon-stabilised" : "relative to robot");
  emit changed(); return true;
}

bool JackalControls::Settings(const QJsonObject &settings)
{
  msgs::StringMsg request, response;
  request.set_data(QJsonDocument(settings).toJson(QJsonDocument::Compact).toStdString());
  bool result = false;
  const bool received = node.Request(prefix + "/lift/settings", request, 2000, response, result);
  const auto object = QJsonDocument::fromJson(QByteArray::fromStdString(response.data())).object();
  if (!received || !result || !object["ok"].toBool())
  { status = object["error"].toString("Actuator request timed out"); emit changed(); return false; }
  status = "Actuator settings applied (session only)"; emit changed(); return true;
}
void JackalControls::ApplyLift(double lo, double hi, double speed, double target)
{
  Stop();
  Settings({{"lower", lo / 1000}, {"upper", hi / 1000}, {"speed", speed / 1000}, {"target", target / 1000}});
}
void JackalControls::ApplyTerrain(int rmsMm)
{
  QVariantMap values = terrainValues;
  values["ground_bump_rms"] = rmsMm / 1000.0;
  ApplyParameters(values);
}

void JackalControls::ReadParameters()
{
  parametersLoaded=false;
  emit parametersChanged();
  const QString directory = qEnvironmentVariable("GREENHOUSE_STATE");
  QFile file(directory + "/terrain-state.json");
  terrainStamp = QFileInfo(file).lastModified();
  if (!file.open(QIODevice::ReadOnly))
  { status="Cannot read applied parameters: "+file.errorString();emit changed();return; }
  auto state = QJsonDocument::fromJson(file.readAll()).object();
  auto config = state["config"].toObject();
  if (config.isEmpty())
  {
    QFile manifest(state["assets_dir"].toString() + "/manifest.json");
    if (!manifest.open(QIODevice::ReadOnly))
    {status="Cannot read parameter manifest: "+manifest.errorString();emit changed();return;}
    config = QJsonDocument::fromJson(manifest.readAll()).object()["config"].toObject();
  }
  QVariantMap applied;
  for (const auto &key : {"ground_resolution", "ground_height_range", "ground_headland",
       "ground_bump_rms", "ground_bump_scale", "ground_friction", "slip_longitudinal", "slip_lateral"})
  {
    if(!config[key].isDouble() || !std::isfinite(config[key].toDouble()))
    {status="Invalid applied parameter: "+QString(key);emit changed();return;}
    applied[key] = config[key].toDouble();
  }
  terrainValues=applied;
  parameterRevision=state["revision"].toString("startup");
  parametersLoaded=true;
  groundRms = qRound(config["ground_bump_rms"].toDouble() * 1000);
  emit parametersChanged();
}

void JackalControls::BindQmlContext(QQmlContext *context)
{
  context->setContextProperty("jackalControls",this);
}

QString JackalControls::NavigationStatePath()
{
  return qEnvironmentVariable("GREENHOUSE_NAV_CACHE");
}

void JackalControls::ApplyParameters(const QVariantMap &values)
{
  if (busy || !active) return;
  if(!parametersLoaded) {status="Applied parameters are unavailable; cannot Apply";emit changed();return;}
  Stop(); busy = true; status = "Checking stationarity and preparing terrain...";
  emit changed();
  terrainProcess.start("timeout", {"--signal=INT", "--kill-after=5s", "180s", "python3",
     "/opt/greenhouse/tools/apply_terrain.py", "--values",
     QString::fromUtf8(QJsonDocument(QJsonObject::fromVariantMap(values)).toJson(QJsonDocument::Compact))});
}

void JackalControls::Navigate(double x,double y,double yaw)
{
  if(busy || !navigationReady || navigationProcess.state()!=QProcess::NotRunning) return;
  ClearInput();
  navigationStatus="Sending goal in greenhouse frame...";
  emit navigationChanged();
  navigationProcess.start("python3",{"/opt/greenhouse/tools/nav_command.py","go",
      "--x",QString::number(x,'g',12),"--y",QString::number(y,'g',12),"--yaw",QString::number(yaw,'g',12),
      "--speed",QString::number(driveSpeed,'g',12),"--turn",QString::number(turnSpeed,'g',12)});
}
void JackalControls::CancelNavigation()
{
  Stop();
  if(navigationProcess.state()==QProcess::NotRunning)
    navigationProcess.start("python3",{"/opt/greenhouse/tools/nav_command.py","cancel"});
}

GZ_ADD_PLUGIN(JackalControls, gz::gui::Plugin)
