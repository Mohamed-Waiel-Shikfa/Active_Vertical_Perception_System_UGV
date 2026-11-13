#include "JackalControls.hh"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QKeyEvent>
#include <QLibrary>
#include <QTest>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/gui_camera.pb.h>

void JackalControls::RunGuiChecks()
{
  static QElapsedTimer deadline;
  if (!deadline.isValid()) deadline.start();
  const QString output = QString::fromStdString(testOutput);
  QDir().mkpath(output);
  if ((!window || !active || !CameraAvailable() || !overlay) && deadline.elapsed() < 150000)
  { QTimer::singleShot(500, this, &JackalControls::RunGuiChecks); return; }
  QJsonObject result{{"status", "FAIL"}, {"input_path", "QTest events on real production QQuickWindow"},
                     {"stroke", stroke}, {"ready", active}, {"camera_frames", imageCount},
                     {"overlay_found", !overlay.isNull()}, {"window_active", window && window->isActive()}};
  auto require = [](bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
  };
  auto wait = [&](const std::function<bool()> &condition, int milliseconds, const char *message) {
    QElapsedTimer clock; clock.start();
    while (!condition() && clock.elapsed() < milliseconds && deadline.elapsed() < 360000)
      QTest::qWait(40);
    require(condition(), message);
  };
  auto world = [&]() {
    gz::msgs::Empty request; gz::msgs::StringMsg response; bool okay = false;
    bool done = node.Request("/greenhouse/world/status", request, 2500, response, okay);
    require(done && okay, "Native world status unavailable");
    return QJsonDocument::fromJson(QByteArray::fromStdString(response.data())).object();
  };
  auto focusScene = [&]() {
    window->requestActivate();
    if (!window->isActive())
    {
      QLibrary x11("libX11.so.6");
      using Open = void *(*)(const char *);
      using Focus = int (*)(void *, unsigned long, int, unsigned long);
      using Raise = int (*)(void *, unsigned long);
      using Close = int (*)(void *);
      const auto open = reinterpret_cast<Open>(x11.resolve("XOpenDisplay"));
      const auto focus = reinterpret_cast<Focus>(x11.resolve("XSetInputFocus"));
      const auto raise = reinterpret_cast<Raise>(x11.resolve("XRaiseWindow"));
      const auto close = reinterpret_cast<Close>(x11.resolve("XCloseDisplay"));
      if (open && focus && raise && close)
      {
        if (void *display = open(nullptr))
        { raise(display, window->winId()); focus(display, window->winId(), 2, 0); close(display); }
      }
    }
    if (overlay)
    {
      auto scene = overlay->parentItem();
      const auto point = scene->mapToScene(QPointF(scene->width()/2, scene->height()/4)).toPoint();
      QTest::mouseClick(window, Qt::LeftButton, Qt::NoModifier, point);
    }
    QTest::qWait(350);
    require(window->isActive(), "Owned X11 GUI could not be activated");
    require(!Editing(), "Scene click did not release numeric-editor focus");
  };
  auto press = [&](Qt::Key key) { QTest::keyPress(window, key); };
  auto release = [&](Qt::Key key) { QTest::keyRelease(window, key); QTest::qWait(100); };
  auto hold = [&](Qt::Key key, double seconds) {
    focusScene();
    const double start = state["sim_time"].toDouble();
    press(key);
    wait([&] { return state["sim_time"].toDouble() >= start + seconds; },
         20000, "Simulation clock stalled during the actual held key");
    release(key);
    wait([&] { const auto robot = world()["robot"].toObject(); return robot["speed"].toDouble(1) < 0.02 && robot["angular_speed"].toDouble(1) < 0.03; },
         15000, "Base did not stop after key release");
  };
  auto screenshot = [&](const char *name) {
    require(window->grabWindow().save(output + "/" + name), "Could not save actual GUI screenshot");
  };
  try
  {
    require(window && active && CameraAvailable() && overlay, "Production GUI/camera did not become ready");
    if (QFile::exists(output + "/operator-check.json"))
    {
      RunOperatorChecks(output + "/operator-check.json");
      return;
    }
    if (QFile::exists(output + "/layout-only"))
    {
      gz::msgs::GUICamera request;
      auto pose = request.mutable_pose();
      pose->mutable_position()->set_x(-9.2);
      pose->mutable_position()->set_y(-2.5);
      pose->mutable_position()->set_z(2.2);
      const double p = .35, y = .97;
      pose->mutable_orientation()->set_x(-std::sin(p/2)*std::sin(y/2));
      pose->mutable_orientation()->set_y(std::sin(p/2)*std::cos(y/2));
      pose->mutable_orientation()->set_z(std::cos(p/2)*std::sin(y/2));
      pose->mutable_orientation()->set_w(std::cos(p/2)*std::cos(y/2));
      gz::msgs::Boolean reply; bool received = false;
      require(node.Request("/gui/move_to/pose", request, 2000, reply, received)
              && received && reply.data(), "Could not set owned layout inspection view");
      QTest::qWait(3000);
      screenshot("normal-layout.png");
      QFile layout(output + "/layout-result.json");
      if (layout.open(QIODevice::WriteOnly))
        layout.write(QJsonDocument(QJsonObject{{"status", "CAPTURED"}, {"stroke_m", stroke},
                                              {"scope", "layout only; no motion or parameter tests"}}).toJson());
      return;
    }
    QFile started(output + "/gui-progress.json");
    if (started.open(QIODevice::WriteOnly))
      started.write(QJsonDocument(QJsonObject{{"phase", "real keyboard events"}}).toJson());
    focusScene();
    require(!PluginItem()->findChild<QQuickItem *>("keyboardEnable"), "Keyboard checkbox should be removed");
    require(keyboard, "Keyboard input is not enabled by default");
    screenshot("normal-initial.png");
    result["control_card_height"] = CardItem()->height();
    focusScene();
    wait([&] { const auto robot = world()["robot"].toObject(); return robot["speed"].toDouble(1) < 0.01 && robot["angular_speed"].toDouble(1) < 0.02; },
         15000, "Robot had not settled before keyboard measurement");
    const auto initial = world();
    const auto initialRobot = initial["robot"].toObject();
    result["initial_pose"] = initialRobot;
    result["phase"] = QString::fromStdString(testPhase);
    if (testPhase != "remaining")
    {
    hold(Qt::Key_Up, 0.8);
    auto forward = world()["robot"].toObject();
    result["forward_pose"] = forward;
    auto alongHeading = [](const QJsonObject &from, const QJsonObject &to) {
      const double yaw = from["yaw"].toDouble();
      return (to["x"].toDouble()-from["x"].toDouble())*std::cos(yaw)
           + (to["y"].toDouble()-from["y"].toDouble())*std::sin(yaw);
    };
    result["forward_metres_along_heading"] = alongHeading(initialRobot, forward);
    require(alongHeading(initialRobot, forward) > 0.01, "Up did not move forward");
    hold(Qt::Key_Down, 0.8);
    auto reverse = world()["robot"].toObject();
    result["reverse_pose"] = reverse;
    result["reverse_metres_along_heading"] = alongHeading(forward, reverse);
    require(alongHeading(forward, reverse) < -0.01, "Down did not reverse");
    hold(Qt::Key_Left, 0.8);
    auto left = world()["robot"].toObject();
    hold(Qt::Key_Right, 0.8);
    auto right = world()["robot"].toObject();
    require(left["yaw"].toDouble() > reverse["yaw"].toDouble(), "Left yaw sign is wrong");
    require(right["yaw"].toDouble() < left["yaw"].toDouble(), "Right yaw sign is wrong");
    result["arrow_events"] = true;
    result["forward_pose"] = forward; result["reverse_pose"] = reverse;
    result["left_pose"] = left; result["right_pose"] = right;
    press(Qt::Key_Up); press(Qt::Key_Down); press(Qt::Key_A); press(Qt::Key_Q);
    QTest::qWait(300);
    require(lastLift == 0, "Opposing lift keys did not cancel");
    release(Qt::Key_Up); release(Qt::Key_Down); release(Qt::Key_A); release(Qt::Key_Q);
    Stop();
    press(Qt::Key_Up);
    QEvent deactivate(QEvent::WindowDeactivate);
    QCoreApplication::sendEvent(window, &deactivate);
    require(keys.isEmpty() && lastLift == 0, "Window focus loss did not stop inputs");
    release(Qt::Key_Up);
    auto input = PluginItem()->findChild<QQuickItem *>("liftTarget");
    require(input, "Target editor not found");
    const auto textInputs = input->findChildren<QQuickItem *>();
    for (auto item : textInputs)
      if (QString(item->metaObject()->className()).contains("TextInput"))
      { item->forceActiveFocus(); break; }
    require(Editing(), "Numeric editor did not receive real Qt focus");
    press(Qt::Key_A); QTest::qWait(150);
    require(!keys.contains(Qt::Key_A) && lastLift == 0, "Editing a number triggered actuator");
    release(Qt::Key_A); focusScene();
    result["opposing_keys_and_focus_safety"] = true;
    }
    else
    {
      result["prior_verified_cases"] = "arrows, opposing keys, window/numeric focus safety: gui-focus evidence";
    }
    focusScene();
    press(Qt::Key_C);
    wait([&] { return view == 1; }, 2000, "C did not select camera 1");
    QKeyEvent repeat(QEvent::KeyPress, Qt::Key_C, Qt::NoModifier, "c", true);
    QCoreApplication::sendEvent(window, &repeat);
    require(view == 1, "Repeated C incorrectly advanced the view");
    release(Qt::Key_C); QTest::qWait(300);
    screenshot("camera-lower.png");
    result["lower_camera_position"] = position;
    const int framesBefore = imageCounts[0];
    focusScene();
    press(Qt::Key_A);
    wait([&] { return sentLift == 1; }, 2000, "A key was not published; inspect focus/key delivery");
    wait([&] { return position >= stroke - 0.001; }, 150000, "A did not reach the installed upper hard stop");
    release(Qt::Key_A); QTest::qWait(500);
    require(position <= stroke + 0.002 && std::abs(velocity) < 0.01, "Upper hard stop did not hold");
    require(imageCounts[0] > framesBefore + 5, "Camera 1 frames did not continue while raising");
    screenshot("camera-upper.png");
    result["upper_camera_position"] = position;
    result["upper_limit"] = position;
    require(!Settings({{"upper", stroke - 0.05}}), "Unsafe window shrink was accepted");
    require(!Settings({{"target", stroke + 0.1}}), "Target outside hard stroke was accepted");
    focusScene();
    press(Qt::Key_Q);
    wait([&] { return sentLift == -1; }, 2000, "Q key was not published; inspect focus/key delivery");
    wait([&] { return position <= 0.001; }, 150000, "Q did not reach lower hard stop");
    release(Qt::Key_Q); QTest::qWait(400);
    require(position >= -0.002, "Lower hard stop was violated");
    result["lower_limit"] = position;
    require(Settings({{"target", 0.05}}), "Could not position inside test operating window");
    wait([&] { return std::abs(position - 0.05) < 0.002; }, 15000, "Lift target was not reached");
    require(Settings({{"lower", 0.02}, {"upper", 0.12}, {"target", 0.08}, {"speed", 0.05}}),
            "Valid live operating window/speed was rejected");
    wait([&] { return std::abs(position - 0.08) < 0.002; }, 15000, "Live target setting did not move carriage");
    focusScene(); press(Qt::Key_A); wait([&] { return position >= 0.118; }, 15000, "Live upper window not reached"); release(Qt::Key_A);
    focusScene(); press(Qt::Key_Q); wait([&] { return position <= 0.022; }, 15000, "Live lower window not reached"); release(Qt::Key_Q);
    result["software_window"] = true;
    focusScene(); press(Qt::Key_A);
    wait([&] { return sentLift == 1; }, 2000, "Heartbeat did not authorize held A");
    std::this_thread::sleep_for(std::chrono::milliseconds(1800));
    require(sentLift == 0, "Stalled GUI left a nonzero heartbeat running");
    Stop(); release(Qt::Key_A);
    result["stale_gui_cutoff_ms"] = 1500;
    focusScene(); press(Qt::Key_A);
    wait([&] { return sentLift == 1; }, 2000, "A did not publish before focus-stop test");
    QEvent lostFocus(QEvent::WindowDeactivate);
    QCoreApplication::sendEvent(window, &lostFocus);
    require(sentLift == 0 && keys.isEmpty(), "Focus loss did not stop the independent heartbeat");
    release(Qt::Key_A);
    result["heartbeat_focus_stop"] = true;
    focusScene(); press(Qt::Key_A);
    wait([&] { return sentLift == 1; }, 2000, "A did not publish before keyboard-disable test");
    SetKeyboardEnabled(false);
    wait([&] { return !keyboard && sentLift == 0; }, 1500, "Keyboard disable did not stop heartbeat");
    release(Qt::Key_A);
    result["heartbeat_keyboard_disable"] = true;
    SetKeyboardEnabled(true);
    focusScene();
    press(Qt::Key_C); release(Qt::Key_C);
    require(view == 2, "Second C did not select camera 2");
    QTest::qWait(500);
    screenshot("camera2.png");
    auto gimbalMeasured = [&](int id, const char *axis) {
      for (const auto &entry : gimbals)
      {
        const auto object = entry.toMap();
        if (object["camera"].toInt() == id) return object["measured"].toMap()[axis].toDouble();
      }
      return 1e9;
    };
    require(ApplyGimbal(2, 0, -30, 20, true), "Valid gimbal command was rejected");
    wait([&] { return std::abs(gimbalMeasured(2, "pitch") + 30) < 1 && std::abs(gimbalMeasured(2, "yaw") - 20) < 1; },
         10000, "Camera 2 gimbal did not reach pitch -30 / yaw 20 deg");
    require(!ApplyGimbal(2, 0, -91, 0, true), "Pitch outside the +/-90 deg range was accepted");
    result["gimbal2_measured_pitch"] = gimbalMeasured(2, "pitch");
    screenshot("camera2-gimbal.png");
    require(ApplyGimbal(2, 0, 0, 0, true), "Could not restore gimbal 2");
    // Held P raises camera 1 pitch; Alt+Shift+Y lowers camera 2 yaw; nothing else moves.
    auto gimbalCommand = [&](int id, const char *axis) {
      for (const auto &entry : gimbals)
      {
        const auto object = entry.toMap();
        if (object["camera"].toInt() == id) return object["command"].toMap()[axis].toDouble();
      }
      return 1e9;
    };
    wait([&] { return std::abs(gimbalCommand(2, "yaw")) < 1e-6; }, 2000, "Gimbal 2 restore was not reported");
    gimbalKeySpeed = 20;
    focusScene();
    QTest::keyPress(window, Qt::Key_P); QTest::qWait(1000); QTest::keyRelease(window, Qt::Key_P);
    QTest::keyPress(window, Qt::Key_Y, Qt::AltModifier | Qt::ShiftModifier); QTest::qWait(1000);
    QTest::keyRelease(window, Qt::Key_Y, Qt::AltModifier | Qt::ShiftModifier);
    wait([&] { return gimbalCommand(1, "pitch") > 10 && gimbalCommand(2, "yaw") < -10; }, 3000,
         "P / Alt+Shift+Y did not steer the gimbals");
    require(std::abs(gimbalCommand(1, "yaw")) < 1e-6 && std::abs(gimbalCommand(2, "pitch")) < 1e-6,
            "Gimbal keys moved the wrong camera or axis");
    result["gimbal_key_camera1_pitch"] = gimbalCommand(1, "pitch");
    result["gimbal_key_camera2_yaw"] = gimbalCommand(2, "yaw");
    gimbalKeySpeed = 5;
    require(ApplyGimbal(1, 0, 0, 0, true) && ApplyGimbal(2, 0, 0, 0, true), "Could not restore the gimbals");
    press(Qt::Key_C); release(Qt::Key_C);
    require(view == 0, "Third C did not restore the normal 3D view");
    result["camera_toggle_and_coupled_frames"] = true;
    Stop(); QTest::qWait(500);
    auto roughness = PluginItem()->findChild<QQuickItem *>("groundRms");
    auto apply = PluginItem()->findChild<QQuickItem *>("applyTerrain");
    require(roughness && apply, "Runtime ground parameter controls are missing");
    for (auto parent = roughness->parentItem(); parent; parent = parent->parentItem())
    {
      if (parent->property("contentY").isValid())
      {
        const double y = roughness->mapToItem(parent, QPointF(0, 0)).y()
                         + parent->property("contentY").toDouble();
        const double maximum = std::max(0.0, parent->property("contentHeight").toDouble() - parent->height());
        parent->setProperty("contentY", std::clamp(y - parent->height()/2, 0.0, maximum));
        break;
      }
    }
    QTest::qWait(200);
    QTest::mouseClick(window, Qt::LeftButton, Qt::NoModifier,
                     roughness->mapToScene(QPointF(roughness->width()/2, roughness->height()/2)).toPoint());
    QTest::keyClick(window, Qt::Key_A, Qt::ControlModifier);
    QTest::keyClick(window, Qt::Key_0);
    QTest::keyClick(window, Qt::Key_Return);
    QTest::qWait(150);
    require(roughness->property("value").toInt() == 0, "GUI ground value was not edited");
    const auto terrainBefore = world();
    QTest::mouseClick(window, Qt::LeftButton, Qt::NoModifier,
                     apply->mapToScene(QPointF(apply->width()/2, apply->height()/2)).toPoint());
    wait([&] { return busy; }, 1500, "GUI Apply did not start the terrain transaction");
    wait([&] { return !busy; }, 150000, "GUI terrain transaction exceeded its deadline");
    require(status.startsWith("Terrain applied"), "GUI terrain application failed");
    const auto terrainAfter = world();
    require(terrainBefore["robot"].toObject()["id"] == terrainAfter["robot"].toObject()["id"],
            "Terrain update replaced the robot instead of changing support");
    require(terrainAfter["sim_time"].toDouble() >= terrainBefore["sim_time"].toDouble(),
            "Terrain update reset the clock");
    result["terrain_apply_via_gui"] = true;
    result["terrain_after"] = terrainAfter;
    screenshot("normal-controls.png");
    result["status"] = "PASS";
    result["camera_frames"] = imageCount;
    result["camera1_frames"] = imageCounts[0];
    result["camera2_frames"] = imageCounts[1];
    result["world_robot_id_before"] = initialRobot["id"];
    result["world_robot_id_after"] = world()["robot"].toObject()["id"];
  }
  catch (const std::exception &error)
  {
    result["error"] = error.what();
    result["last_status"] = status;
    result["editing"] = Editing();
    if (window) window->grabWindow().save(output + "/gui-failure.png");
  }
  Stop(); keyboard = false; emit changed();
  {
    std::lock_guard<std::mutex> lock(commandMutex);
    result["published_commands"] = commandTrace;
  }
  QFile file(output + "/gui-result.json");
  if (file.open(QIODevice::WriteOnly)) file.write(QJsonDocument(result).toJson());
}
