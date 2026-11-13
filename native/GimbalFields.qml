import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "qrc:/JackalControls" as Fields

// Roll / pitch / yaw command for one C-200T gimbal. Angles are degrees in the gimbal's own frame:
// 0/0/0 looks level at the horizon, straight out of the robot's side the gimbal is mounted on.
// The fields follow the live command (Apply, P/R/Y keys, ROS) whenever it changes.
ColumnLayout {
  id: gimbal
  property int camera: 1
  property string title
  property string shown: ""
  property var current: {
    var list = jackalControls.gimbals
    for (var i = 0; i < list.length; ++i) if (list[i].camera === gimbal.camera) return list[i]
    return null
  }
  Layout.fillWidth: true
  spacing: 4
  function range(axis, index) {
    var limits = jackalControls.gimbalLimits[axis]
    return limits === undefined ? (index === 0 ? -90 : 90) : limits[index]
  }
  function load() {
    if (gimbal.current === null) return
    var c = gimbal.current.command
    var key = [c.roll, c.pitch, c.yaw, gimbal.current.stabilise].join("/")
    if (key === gimbal.shown) return
    gimbal.shown = key
    roll.number = c.roll
    pitch.number = c.pitch
    yaw.number = c.yaw
    stabilise.checked = gimbal.current.stabilise
  }
  onCurrentChanged: load()
  Label { text: gimbal.title; font.bold: true; Layout.topMargin: 2 }
  Fields.RuntimeField { id: roll; objectName: "gimbal" + gimbal.camera + "Roll"; label: "Roll (deg)"; minimum: gimbal.range("roll", 0); maximum: gimbal.range("roll", 1); decimals: 1; help: "Positive lowers the image's right side. Keys: R (+), Shift+R (-); Alt for camera 2." }
  Fields.RuntimeField { id: pitch; objectName: "gimbal" + gimbal.camera + "Pitch"; label: "Pitch (deg)"; minimum: gimbal.range("pitch", 0); maximum: gimbal.range("pitch", 1); decimals: 1; help: "Positive tilts up, 0 is the horizon. Keys: P (+), Shift+P (-); Alt for camera 2." }
  Fields.RuntimeField { id: yaw; objectName: "gimbal" + gimbal.camera + "Yaw"; label: "Yaw (deg)"; minimum: gimbal.range("yaw", 0); maximum: gimbal.range("yaw", 1); decimals: 1; help: "From the outward mounting direction; positive turns counterclockwise. Keys: Y (+), Shift+Y (-); Alt for camera 2." }
  CheckBox {
    id: stabilise
    objectName: "gimbal" + gimbal.camera + "Stabilise"
    implicitHeight: 28
    checked: true
    text: "Stabilise (horizon lock)"
    ToolTip.text: "On: roll/pitch held relative to gravity, yaw follows the robot heading. Off: angles are relative to the robot (FPV)."
    ToolTip.visible: hovered
  }
  Label {
    objectName: "gimbal" + gimbal.camera + "Measured"
    Layout.fillWidth: true
    wrapMode: Text.WordWrap
    text: gimbal.current === null ? "Waiting for gimbal state"
          : "Actual: roll " + gimbal.current.measured.roll.toFixed(1) + ", pitch "
            + gimbal.current.measured.pitch.toFixed(1) + ", yaw " + gimbal.current.measured.yaw.toFixed(1)
            + " deg" + (gimbal.current.saturated ? " (at mechanical limit)" : "")
  }
  Button {
    objectName: "applyGimbal" + gimbal.camera
    text: "Apply camera " + gimbal.camera + " gimbal"
    implicitHeight: 32
    Layout.fillWidth: true
    enabled: jackalControls.gimbalReady
    onClicked: jackalControls.ApplyGimbal(gimbal.camera, roll.number, pitch.number, yaw.number, stabilise.checked)
  }
}
