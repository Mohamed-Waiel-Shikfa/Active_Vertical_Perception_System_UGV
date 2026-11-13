#pragma once
#include <QDateTime>
#include <QElapsedTimer>
#include <QHash>
#include <QImage>
#include <QJsonObject>
#include <QJsonArray>
#include <QPointer>
#include <QProcess>
#include <QQuickImageProvider>
#include <QQuickItem>
#include <QQuickWindow>
#include <QSet>
#include <QTimer>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

constexpr int kCameraCount = 2;

// Latest frame of each gimbal camera; QML requests "image://gimbal_cameras/<camera>/<version>".
class CameraProvider : public QQuickImageProvider
{
  std::mutex mutex;
  QImage images[kCameraCount];
public:
  CameraProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}
  QImage requestImage(const QString &id, QSize *size, const QSize &) override;
  void Set(int camera, const QImage &value);
};

class JackalControls : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(bool keyboardEnabled READ KeyboardEnabled WRITE SetKeyboardEnabled NOTIFY changed)
  Q_PROPERTY(double driveSpeed READ DriveSpeed WRITE SetDriveSpeed NOTIFY changed)
  Q_PROPERTY(double turnSpeed READ TurnSpeed WRITE SetTurnSpeed NOTIFY changed)
  Q_PROPERTY(double position READ Position NOTIFY changed)
  Q_PROPERTY(double velocity READ Velocity NOTIFY changed)
  Q_PROPERTY(double stroke READ Stroke NOTIFY changed)
  Q_PROPERTY(double maxLiftSpeed READ MaxLiftSpeed NOTIFY changed)
  Q_PROPERTY(double liftSpeed READ LiftSpeed NOTIFY changed)
  Q_PROPERTY(int groundRms READ GroundRms NOTIFY changed)
  Q_PROPERTY(QVariantMap terrainValues READ TerrainValues NOTIFY parametersChanged)
  Q_PROPERTY(bool parametersReady READ ParametersReady NOTIFY parametersChanged)
  Q_PROPERTY(QString navigationStatus READ NavigationStatus NOTIFY navigationChanged)
  Q_PROPERTY(bool navigationReady READ NavigationReady NOTIFY navigationChanged)
  Q_PROPERTY(bool navigating READ Navigating NOTIFY navigationChanged)
  Q_PROPERTY(bool ready READ Ready NOTIFY changed)
  Q_PROPERTY(bool cameraView READ CameraView NOTIFY changed)
  Q_PROPERTY(int viewIndex READ ViewIndex NOTIFY changed)
  Q_PROPERTY(bool cameraAvailable READ CameraAvailable NOTIFY changed)
  Q_PROPERTY(bool camera1Available READ Camera1Available NOTIFY changed)
  Q_PROPERTY(bool camera2Available READ Camera2Available NOTIFY changed)
  Q_PROPERTY(QVariantList gimbals READ Gimbals NOTIFY gimbalChanged)
  Q_PROPERTY(bool gimbalReady READ GimbalReady NOTIFY gimbalChanged)
  Q_PROPERTY(QVariantMap gimbalLimits READ GimbalLimits NOTIFY gimbalChanged)
  Q_PROPERTY(double gimbalKeySpeed READ GimbalKeySpeed WRITE SetGimbalKeySpeed NOTIFY changed)
  Q_PROPERTY(int frameVersion READ FrameVersion NOTIFY frameChanged)
  Q_PROPERTY(bool busy READ Busy NOTIFY changed)
  Q_PROPERTY(QString status READ Status NOTIFY changed)

  gz::transport::Node node;
  gz::transport::Node::Publisher drivePublisher, liftPublisher, overridePublisher, gimbalPublisher;
  std::string prefix, driveTopic;
  QPointer<QQuickWindow> window;
  QPointer<QQuickItem> overlay;
  CameraProvider *provider{nullptr};
  QTimer timer;
  QProcess terrainProcess;
  QProcess navigationProcess;
  QElapsedTimer navigationPoll;
  QString navigationStatus{"Navigation starting"};
  bool navigationReady{false}, navigating{false};
  QSet<int> keys;
  bool keyboard{true}, active{false}, busy{false};
  int view{0};  // 0 = normal 3D view, 1 = camera 1 (left gimbal), 2 = camera 2 (right gimbal)
  int imageCounts[kCameraCount]{0, 0};
  QElapsedTimer lastFrames[kCameraCount];
  QVariantList gimbals;
  bool gimbalReady{false};
  QElapsedTimer lastGimbal;
  QVariantMap gimbalLimits;  // {"roll": [lower, upper], ...} degrees, from the controller's state
  // Keyboard steering: P/R/Y key -> (camera, +1/-1) chosen by Alt/Shift when the key went down.
  QHash<int, QPair<int, double>> gimbalKeyModes;
  double gimbalKeySpeed{5};
  double gimbalTargets[kCameraCount][3]{};  // roll, pitch, yaw degrees being steered
  bool gimbalSteering[kCameraCount]{};
  QElapsedTimer gimbalClock;
  double driveSpeed{2.0}, turnSpeed{4.0}, position{0}, velocity{0};
  double stroke{1.5}, maxLiftSpeed{0.2}, liftSpeed{0.1};
  double lastLift{0};
  int frameVersion{0}, imageCount{0};
  int groundRms{16};
  QVariantMap terrainValues;
  bool parametersLoaded{false};
  QString parameterRevision;
  QDateTime terrainStamp;  // terrain-state.json mtime last read; scripts can Apply too
  Q_INVOKABLE void ReadParameters();
  QString status{"Waiting for actuator"};
  QElapsedTimer lastState;
  QJsonObject state;
  QJsonArray commandTrace;
  std::string testOutput;
  std::string testPhase{"full"};
  std::mutex commandMutex;
  std::thread heartbeat;
  std::atomic<bool> heartbeatRunning{false};
  std::atomic<double> sentLift{0};
  double sentForward{0}, sentYaw{0}, wantedForward{0}, wantedYaw{0}, wantedLift{0}, commandSimTime{0};
  bool commandLive{false};
  std::chrono::steady_clock::time_point commandUpdated{};
  void Heartbeat();
  void ReceiveState(const gz::msgs::StringMsg &message);
  void ReceiveImage(int camera, const gz::msgs::Image &message);
  void ReceiveGimbal(const gz::msgs::StringMsg &message);
  void Tick();
  void SteerGimbals();
  void BindOverlay();
  bool Editing() const;
  void Publish(double forward, double yaw, double lift);
  void ClearInput();
  void OverrideNavigation();
  void RunGuiChecks();
  void RunOperatorChecks(const QString &specification);
  bool Settings(const QJsonObject &settings);

public:
  JackalControls();
  ~JackalControls() override;
  void BindQmlContext(QQmlContext *context);
  static QString NavigationStatePath();
  void LoadConfig(const tinyxml2::XMLElement *element) override;
  bool eventFilter(QObject *object, QEvent *event) override;
  bool KeyboardEnabled() const { return keyboard; }
  double DriveSpeed() const { return driveSpeed; }
  double TurnSpeed() const { return turnSpeed; }
  double Position() const { return position; }
  double Velocity() const { return velocity; }
  double Stroke() const { return stroke; }
  double MaxLiftSpeed() const { return maxLiftSpeed; }
  double LiftSpeed() const { return liftSpeed; }
  int GroundRms() const { return groundRms; }
  QVariantMap TerrainValues() const { return terrainValues; }
  bool ParametersReady() const { return parametersLoaded; }
  QString NavigationStatus() const { return navigationStatus; }
  bool NavigationReady() const { return navigationReady; }
  bool Navigating() const { return navigating; }
  bool Ready() const { return active; }
  bool CameraView() const { return view > 0; }
  int ViewIndex() const { return view; }
  bool CameraAvailable(int camera) const
  { return camera >= 1 && camera <= kCameraCount && imageCounts[camera-1] > 0 &&
           lastFrames[camera-1].isValid() && lastFrames[camera-1].elapsed() < 3000; }
  bool CameraAvailable() const { return view > 0 ? CameraAvailable(view) : AllCamerasAvailable(); }
  bool AllCamerasAvailable() const { return CameraAvailable(1) && CameraAvailable(2); }
  bool Camera1Available() const { return CameraAvailable(1); }
  bool Camera2Available() const { return CameraAvailable(2); }
  QVariantList Gimbals() const { return gimbals; }
  bool GimbalReady() const { return gimbalReady && lastGimbal.isValid() && lastGimbal.elapsed() < 3000; }
  QVariantMap GimbalLimits() const { return gimbalLimits; }
  double GimbalKeySpeed() const { return gimbalKeySpeed; }
  void SetGimbalKeySpeed(double value);
  int FrameVersion() const { return frameVersion; }
  bool Busy() const { return busy; }
  QString Status() const { return status; }
  void SetKeyboardEnabled(bool value);
  void SetDriveSpeed(double value);
  void SetTurnSpeed(double value);
  Q_INVOKABLE void ToggleCamera();
  Q_INVOKABLE void SelectView(int index);
  Q_INVOKABLE bool ApplyGimbal(int camera, double rollDeg, double pitchDeg, double yawDeg, bool stabilise);
  Q_INVOKABLE void Stop();
  Q_INVOKABLE void ButtonKey(int key, bool pressed);
  Q_INVOKABLE void ApplyLift(double lowerMm, double upperMm, double speedMm, double targetMm);
  Q_INVOKABLE void ApplyTerrain(int rmsMm);
  Q_INVOKABLE void ApplyParameters(const QVariantMap &values);
  Q_INVOKABLE void Navigate(double x, double y, double yawDegrees);
  Q_INVOKABLE void CancelNavigation();
signals:
  void changed();
  void frameChanged();
  void parametersChanged();
  void navigationChanged();
  void gimbalChanged();
};
