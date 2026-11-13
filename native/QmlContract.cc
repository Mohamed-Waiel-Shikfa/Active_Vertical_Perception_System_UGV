#include "JackalControls.hh"
#include <QFile>
#include <QGuiApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTest>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQmlExpression>
#include <iostream>
#include <memory>

int main(int argc,char **argv)
{
  QGuiApplication app(argc,argv);
  QTemporaryDir state;
  qputenv("GREENHOUSE_STATE",state.path().toUtf8());
  QJsonObject values{{"ground_resolution",.15},{"ground_height_range",.03},{"ground_headland",3.0},
                     {"ground_bump_rms",.003},{"ground_bump_scale",.18},{"ground_friction",.9},
                     {"slip_longitudinal",.1},{"slip_lateral",1.0}};
  auto writeParameters=[&]{
    QFile file(state.path()+"/terrain-state.json");
    if(!file.open(QIODevice::WriteOnly)) return false;
    file.write(QJsonDocument(QJsonObject{{"config",values},{"revision","contract"}}).toJson());
    return true;
  };
  if(!writeParameters()) return 1;
  JackalControls controls;
  QQmlEngine engine;
  QFile file(":/JackalControls/JackalControls.qml");
  if(!file.open(QIODevice::ReadOnly)) return 1;
  QQmlComponent component(&engine);
  component.setData(file.readAll(),QUrl("file::/JackalControls/JackalControls.qml"));
  std::unique_ptr<QObject> panel(component.create());
  if(!panel) {std::cerr<<component.errorString().toStdString();return 1;}
  if(panel->findChild<QObject *>("applyTerrain"))
  {std::cerr<<"Panel instantiated before native context";return 1;}
  controls.BindQmlContext(engine.rootContext());
  QMetaObject::invokeMethod(&controls,"ReadParameters",Qt::DirectConnection);
  app.processEvents();
  if(!controls.ParametersReady()) return 1;
  for(const auto &name:{"groundResolutionInput","groundHeightInput","groundHeadlandInput",
      "groundRmsInput","groundScaleInput","groundFrictionInput","slipLongitudinalInput",
      "slipLateralInput","goalXInput","goalYInput","goalYawInput","navigateGo","navigateCancel","applyLift",
      "gimbal1RollInput","gimbal1PitchInput","gimbal1YawInput","gimbal1Stabilise","applyGimbal1",
      "gimbal2RollInput","gimbal2PitchInput","gimbal2YawInput","gimbal2Stabilise","applyGimbal2",
      "gimbalKeySpeedInput"})
    if(!panel->findChild<QObject *>(name)) {std::cerr<<"Missing native control "<<name;return 1;}
  auto value=[&](const char *name){return panel->findChild<QObject *>(name)->property("value").toInt();};
  if(value("groundResolutionInput")!=150 || value("groundHeightInput")!=30
      || value("groundHeadlandInput")!=300 || value("groundRmsInput")!=30
      || value("groundScaleInput")!=180 || value("groundFrictionInput")!=90
      || value("slipLongitudinalInput")!=10 || value("slipLateralInput")!=100)
  {
    std::cerr<<"Initial editor integers: "<<value("groundResolutionInput")<<","
      <<value("groundHeightInput")<<","<<value("groundHeadlandInput")<<","
      <<value("groundRmsInput")<<","<<value("groundScaleInput")<<","
      <<value("groundFrictionInput")<<","<<value("slipLongitudinalInput")<<","
      <<value("slipLateralInput")<<"; authoritative="
      <<QJsonDocument(QJsonObject::fromVariantMap(controls.TerrainValues())).toJson().toStdString();
    return 1;
  }
  values["ground_height_range"]=.025;
  if(!writeParameters()) return 1;
  QMetaObject::invokeMethod(&controls,"ReadParameters",Qt::DirectConnection);
  app.processEvents();
  if(value("groundHeightInput")!=25)
  {std::cerr<<"Subsequent native readback did not update editor";return 1;}
  QQuickWindow window;
  window.setWidth(340);window.setHeight(942);
  auto item=qobject_cast<QQuickItem *>(panel.get());
  item->setParentItem(window.contentItem());item->setWidth(340);item->setHeight(942);
  window.show();window.requestActivate();QTest::qWait(150);
  if(panel->findChild<QQuickItem *>("parameterHorizontalBar")->isVisible())
  {std::cerr<<"Horizontal parameter scrollbar covers bottom controls";return 1;}
  auto friction=panel->findChild<QQuickItem *>("groundFrictionInput");
  auto input=qvariant_cast<QQuickItem *>(friction->property("contentItem"));
  if(!input) return 1;
  QTest::mouseClick(&window,Qt::LeftButton,Qt::NoModifier,
      input->mapToScene(QPointF(input->width()/2,input->height()/2)).toPoint());
  if(!input->hasActiveFocus()) {std::cerr<<"Real numeric text click did not focus editor";return 1;}
  QTest::keyClick(&window,Qt::Key_A,Qt::ControlModifier);
  for(const auto &ch:QString("0.70"))
    QTest::keyClick(&window,static_cast<Qt::Key>(ch.toLatin1()));
  QTest::keyClick(&window,Qt::Key_Return);QTest::keyClick(&window,Qt::Key_Tab);
  app.processEvents();
  if(value("groundFrictionInput")!=70)
  {std::cerr<<"Real numeric entry did not commit";return 1;}
  QMetaObject::invokeMethod(&controls,"ReadParameters",Qt::DirectConnection);
  app.processEvents();
  if(value("groundFrictionInput")!=90) return 1;
  for(const auto &method:{"ApplyParameters","ApplyLift","Navigate","CancelNavigation","Stop","ButtonKey","ToggleCamera",
                           "SelectView","ApplyGimbal"})
  {
    QQmlExpression expression(qmlContext(panel.get()),panel.get(),
                             QString("typeof jackalControls.%1").arg(method));
    if(expression.evaluate().toString()!="function")
    {std::cerr<<"QML context method is shadowed: "<<method;return 1;}
  }
  QMetaObject::invokeMethod(&controls,"changed",Qt::DirectConnection);
  QMetaObject::invokeMethod(&controls,"parametersChanged",Qt::DirectConnection);
  QQmlEngine cameraEngine;
  QQmlContext cameraContext(cameraEngine.rootContext());
  controls.BindQmlContext(&cameraContext);
  QFile cameraFile(":/CameraOverlay.qml");
  if(!cameraFile.open(QIODevice::ReadOnly)) return 1;
  QQmlComponent cameraComponent(&cameraEngine);
  cameraComponent.setData(cameraFile.readAll(),QUrl("file::/CameraOverlay.qml"));
  std::unique_ptr<QObject> camera(cameraComponent.create(&cameraContext));
  if(!camera) {std::cerr<<cameraComponent.errorString().toStdString();return 1;}
  item->setParentItem(nullptr);
  std::cout<<"PASS: late native context, all authoritative initial values, subsequent readback, commands, gimbal controls and independent camera context.\n";
}
