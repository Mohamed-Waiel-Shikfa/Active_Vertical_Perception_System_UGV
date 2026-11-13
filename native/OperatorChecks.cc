#include "JackalControls.hh"
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QFile>
#include <QJsonDocument>
#include <QDir>
#include <QRegularExpression>
#include <QLibrary>
#include <QTest>
#include <functional>
#include <algorithm>
#include <cmath>
#include <stdexcept>

void JackalControls::RunOperatorChecks(const QString &path)
{
  QFile specFile(path);
  if (!specFile.open(QIODevice::ReadOnly)) return;
  const auto spec = QJsonDocument::fromJson(specFile.readAll()).object();
  const QString directory = QString::fromStdString(testOutput);
  const QString navigationState=NavigationStatePath();
  QJsonObject report{{"status", "FAIL"}, {"path", "real Qt field editing and button events"}};
  QJsonArray results;
  auto require = [](bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
  };
  auto readJson = [](const QString &name) {
    QElapsedTimer deadline;deadline.start();
    QString error;
    do
    {
      QFile file(name);
      if(file.open(QIODevice::ReadOnly))
      {
        QJsonParseError parse;
        const auto document=QJsonDocument::fromJson(file.readAll(),&parse);
        if(parse.error==QJsonParseError::NoError && document.isObject()) return document.object();
        error=parse.errorString();
      }
      else error=file.errorString();
      QTest::qWait(20);
    } while(deadline.elapsed()<750);
    throw std::runtime_error(("Evidence JSON not readable: "+name+": "+error).toStdString());
  };
  auto world = [&]() {
    gz::msgs::Empty request;gz::msgs::StringMsg response;bool okay=false;
    require(node.Request("/greenhouse/world/status",request,2500,response,okay) && okay,
            "Native physics status unavailable");
    return QJsonDocument::fromJson(QByteArray::fromStdString(response.data())).object();
  };
  auto meshHash = [&](const QJsonObject &terrain) {
    QFile file(terrain["assets_dir"].toString()+"/models/greenhouse_ground/meshes/terrain.dae");
    require(file.open(QIODevice::ReadOnly),"Physical terrain mesh not readable");
    QCryptographicHash hash(QCryptographicHash::Sha256);
    require(hash.addData(&file),"Physical mesh hash failed");
    return QString(hash.result().toHex());
  };
  auto find = [&](const QString &name) -> QQuickItem * {
    QList<QQuickItem *> pending{window->contentItem()};
    for (int i=0;i<pending.size();++i)
    {
      auto item=pending[i]; pending.append(item->childItems());
      if (item->objectName()==name) return item;
    }
    return nullptr;
  };
  auto wait = [&](const std::function<bool()> &ready, int timeout, const char *message) {
    QElapsedTimer timer; timer.start();
    while(!ready() && timer.elapsed()<timeout) QTest::qWait(50);
    require(ready(), message);
  };
  auto reveal = [&](QQuickItem *item) {
    for(auto parent=item->parentItem();parent;parent=parent->parentItem())
    {
      if(parent->property("contentY").isValid())
      {
        const double y=item->mapToItem(parent,QPointF(0,0)).y()+parent->property("contentY").toDouble();
        const double max=std::max(0.0,parent->property("contentHeight").toDouble()-parent->height());
        parent->setProperty("contentY",std::clamp(y-parent->height()/2,0.0,max));
        break;
      }
    }
    QTest::qWait(150);
  };
  auto click = [&](QQuickItem *item) {
    require(item,"Control not found"); reveal(item);
    QTest::mouseClick(window,Qt::LeftButton,Qt::NoModifier,
                     item->mapToScene(QPointF(item->width()/2,item->height()/2)).toPoint());
  };
  struct Field {const char *name;const char *key;double factor;};
  const Field parameterFields[]={
    {"groundResolution","ground_resolution",1},{"groundHeight","ground_height_range",1},
    {"groundHeadland","ground_headland",1},{"groundRms","ground_bump_rms",.001},
    {"groundScale","ground_bump_scale",1},{"groundFriction","ground_friction",1},
    {"slipLongitudinal","slip_longitudinal",1},{"slipLateral","slip_lateral",1}};
  auto proposal = [&]{
    QJsonObject values;
    for(const auto &field:parameterFields)
    {
      auto item=find(field.name);require(item,"Parameter field missing");
      values[field.key]=item->property("number").toDouble()*field.factor;
    }
    return values;
  };
  try
  {
    window->requestActivate();
    QLibrary x11("libX11.so.6");
    const auto open=reinterpret_cast<void *(*)(const char *)>(x11.resolve("XOpenDisplay"));
    const auto focus=reinterpret_cast<int (*)(void *,unsigned long,int,unsigned long)>(x11.resolve("XSetInputFocus"));
    const auto close=reinterpret_cast<int (*)(void *)>(x11.resolve("XCloseDisplay"));
    if(open&&focus&&close) if(void *display=open(nullptr)) {focus(display,window->winId(),2,0);close(display);}
    QTest::qWait(250);
    wait([&]{return navigationReady;},150000,"Native-feedback navigation did not become ready");
    require(parametersLoaded,"Authoritative parameter readback is not ready");
    for(const auto &field:parameterFields)
      require(std::abs(proposal()[field.key].toDouble()-terrainValues[field.key].toDouble())<1e-8,
              "Initial UI proposal differs from authoritative configuration");
    for (const auto &entry : spec["operations"].toArray())
    {
      const auto operation=entry.toObject();
      const auto fields=operation["fields"].toObject();
      for(auto it=fields.begin();it!=fields.end();++it)
      {
        auto editor=find(it.key()); require(editor,"Numeric editor missing");
        auto input=qvariant_cast<QQuickItem *>(editor->property("contentItem"));
        require(input,"Editable native input missing");
        click(input);
        wait([&]{return input->hasActiveFocus();},1500,"Actual numeric text input did not receive focus; no typing sent");
        QTest::keyClick(window,Qt::Key_A,Qt::ControlModifier);
        for (const QChar ch : it.value().toString())
          QTest::keyClick(window,static_cast<Qt::Key>(ch.toLatin1()));
        QTest::keyClick(window,Qt::Key_Return);
        QTest::keyClick(window,Qt::Key_Tab); QTest::qWait(150);
      }
      const auto beforeValues=terrainValues;
      const auto beforeTerrain=readJson(directory+"/terrain-state.json");
      const auto beforeWorld=world();
      const auto oldHash=meshHash(beforeTerrain);
      const QString name=operation["name"].toString();
      QJsonObject evidence{{"name",name},{"before_world",beforeWorld}};
      if(operation["button"].toString("applyTerrain")=="applyTerrain")
      {
        auto intended=beforeValues;
        for(const auto &field:parameterFields)
          if(fields.contains(QString(field.name)+"Input"))
            intended[field.key]=fields[QString(field.name)+"Input"].toString().toDouble()*field.factor;
        const auto actual=proposal();
        for(const auto &field:parameterFields)
          if(std::abs(actual[field.key].toDouble()-intended[field.key].toDouble())>=1e-8)
            throw std::runtime_error(QString("UI field %1 is %2, expected %3; no Apply sent")
                .arg(field.key).arg(actual[field.key].toDouble()).arg(intended[field.key].toDouble()).toStdString());
        evidence["ui_proposal"]=actual;
      }
      click(find(operation["button"].toString("applyTerrain")));
      if(operation["button"].toString("applyTerrain")=="applyTerrain")
      {
        wait([&]{return busy;},2000,"Apply button did not start a transaction");
        wait([&]{return !busy;},210000,"Runtime parameter application timed out");
        const bool expectedFailure=operation["expect_error"].toBool();
        if(expectedFailure) require(status.startsWith("Terrain not applied"),"Invalid values were not rejected");
        else require(status.startsWith("Applied"),"Parameter transaction did not succeed");
        const auto expected=operation["readback"].toObject();
        for(auto it=expected.begin();it!=expected.end();++it)
          require(std::abs(terrainValues[it.key()].toDouble()-it.value().toDouble())<1e-8,"Applied value readback mismatch");
        if(expectedFailure) require(terrainValues==beforeValues,"Rejected transaction changed applied values");
        const auto applied=readJson(directory+"/terrain-state.json");
        const auto afterWorld=world();
        const auto newHash=meshHash(applied);
        if(expectedFailure) require(applied==beforeTerrain,"Rejected transaction changed the physical revision");
        else
        {
          require(applied["before_sim_time"]==applied["after_sim_time"],"Physics stepped during support replacement");
          require(applied["robot_id_before"]==applied["robot_id_after"],"Robot entity was replaced");
          require(applied["collision_uri"]==applied["visual_uri"],"Rendered/collision surface mismatch");
          require(afterWorld["grounds"].toArray().size()==1,"Missing or duplicate ground support");
          const auto nav=readJson(navigationState);
          require(nav["revision"]==applied["revision"],"Navigation and terrain revisions differ");
          if(operation["mesh_changed"].toBool()) require(oldHash!=newHash,"Requested field did not change actual mesh");
        }
        evidence["after_world"]=afterWorld;
        evidence["terrain"]=applied;
        evidence["mesh_sha256_before"]=oldHash;evidence["mesh_sha256_after"]=newHash;
        evidence["surface"]=readJson(applied["assets_dir"].toString()+"/manifest.json")["terrain"];
      }
      if(operation["button"].toString()=="applyLift")
      {
        const double desired=operation["target_m"].toDouble();
        double maximumForce=0;
        wait([&]{maximumForce=std::max(maximumForce,std::abs(state["force"].toDouble()));
                  return std::abs(position-desired)<.002 && std::abs(velocity)<.003;},
             90000,"Startup gimbal-camera payload did not reach the short lift target");
        evidence["lift"]=state;evidence["maximum_force_N"]=maximumForce;
        require(maximumForce<30.00001,"Finite actuator force cap exceeded");
      }
      if(operation["button"].toString()=="navigateGo")
      {
        wait([&]{return navigating;},12000,"Real GUI Go did not start a ROS navigation action");
        if(operation["wait_result"].toBool(true))
        {
          wait([&]{
            const auto phase=readJson(navigationState)["phase"].toString();
            return phase=="Succeeded" || phase=="Error" || phase=="Canceled";
          },600000,"Actual GUI goal exceeded bounded navigation deadline");
          const auto nav=readJson(navigationState);
          evidence["navigation"]=nav;
          require(nav["phase"].toString()=="Succeeded","Real GUI goal failed its full pose tolerance");
        }
      }
      evidence["status"]=status;evidence["applied"]=QJsonObject::fromVariantMap(terrainValues);
      results.append(evidence);
      QDir().mkpath(directory+"/operator-cases");
      QString caseName=name;caseName.replace(QRegularExpression("[^A-Za-z0-9_-]"),"-");
      QFile completed(directory+"/operator-cases/"+caseName+".json");
      if(!completed.exists() && completed.open(QIODevice::WriteOnly))
        completed.write(QJsonDocument(evidence).toJson());
      QFile partial(directory+"/operator-result.json");
      if(partial.open(QIODevice::WriteOnly)) partial.write(QJsonDocument(QJsonObject{{"status","RUNNING"},{"operations",results}}).toJson());
      if(operation.contains("screenshot"))
        require(window->grabWindow().save(directory+"/"+operation["screenshot"].toString()),"Screenshot save failed");
    }
    report["status"]="PASS";
  }
  catch(const std::exception &error)
  {
    report["error"]=error.what();
    report["native_status"]=status;
    report["applied_values"]=QJsonObject::fromVariantMap(terrainValues);
    report["operations"]=results;
    QFile failed(directory+"/operator-result.json");
    if(failed.open(QIODevice::WriteOnly)) failed.write(QJsonDocument(report).toJson());
    window->grabWindow().save(directory+"/operator-failure.png");
  }
  Stop(); report["operations"]=results;
  QFile file(directory+"/operator-result.json");
  if(file.open(QIODevice::WriteOnly)) file.write(QJsonDocument(report).toJson());
}
