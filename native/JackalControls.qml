import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "qrc:/JackalControls" as Fields

Loader {
  Layout.minimumWidth: 320
  Layout.minimumHeight: 610
  anchors.fill: parent
  active: typeof jackalControls !== "undefined"
  sourceComponent: Component {
ColumnLayout {
  id: panel
  objectName: "jackalControlsPanel"
  Layout.minimumWidth: 320
  Layout.minimumHeight: 610
  anchors.fill: parent
  anchors.margins: 8
  spacing: 4
  property bool initializedSettings: false
  function initializeSettings() {
    if (jackalControls.ready && !panel.initializedSettings) {
      liftSpeed.value = Math.round(jackalControls.liftSpeed * 1000)
      panel.readTerrain()
      panel.initializedSettings = true
    }
  }
  function readTerrain() {
    if(!jackalControls.parametersReady) return
    var v = jackalControls.terrainValues
    if (v.ground_resolution === undefined) return
    sampleSpacing.number = v.ground_resolution
    heightRange.number = v.ground_height_range
    padding.number = v.ground_headland
    roughness.number = v.ground_bump_rms * 1000
    bumpScale.number = v.ground_bump_scale
    friction.number = v.ground_friction
    longitudinal.number = v.slip_longitudinal
    lateral.number = v.slip_lateral
  }
  Connections {
    target: typeof jackalControls === "undefined" ? null : jackalControls
    function onChanged() { panel.initializeSettings() }
    function onParametersChanged() { panel.readTerrain() }
  }
  Component.onCompleted: panel.readTerrain()
  Label { text: "Teleop"; font.bold: true }
  Label { text: "Arrows: drive   A / Q: lift   C: 3D / cam 1 / cam 2\nP / R / Y: cam 1 pitch / roll / yaw up; +Alt: cam 2; +Shift: down"; Layout.fillWidth: true; wrapMode: Text.WordWrap }
  Label { text: "Parameters"; font.bold: true; Layout.topMargin: 4 }
  Button { objectName: "applyTerrain"; text: jackalControls.busy ? "Applying..." : "Apply terrain / contact"; implicitHeight: 32; Layout.fillWidth: true; enabled: jackalControls.ready && jackalControls.parametersReady && !jackalControls.busy; onClicked: jackalControls.ApplyParameters({
      "ground_resolution": sampleSpacing.number, "ground_height_range": heightRange.number,
      "ground_headland": padding.number, "ground_bump_rms": roughness.number / 1000,
      "ground_bump_scale": bumpScale.number, "ground_friction": friction.number,
      "slip_longitudinal": longitudinal.number, "slip_lateral": lateral.number
    }) }
  Label {
    objectName: "operationStatus"
    text: jackalControls.parametersReady ? jackalControls.status : "Loading applied parameters: " + jackalControls.status
    wrapMode: Text.WordWrap
    maximumLineCount: 3
    elide: Text.ElideRight
    Layout.fillWidth: true
    Layout.preferredHeight: 48
    ToolTip.text: text
    ToolTip.visible: statusMouse.containsMouse
    MouseArea { id: statusMouse; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
  }
  ScrollView {
    id: settingsScroll
    rightPadding: 16
    contentWidth: availableWidth
    ScrollBar.horizontal: ScrollBar { objectName: "parameterHorizontalBar"; policy: ScrollBar.AlwaysOff }
    Layout.fillWidth: true
    Layout.fillHeight: true
    Layout.minimumHeight: 150
    clip: true
    ColumnLayout {
      width: settingsScroll.availableWidth
      enabled: jackalControls.parametersReady && !jackalControls.busy
      spacing: 4
      Label { text: "Terrain"; font.bold: true }
      Fields.RuntimeField { id: sampleSpacing; objectName: "groundResolution"; label: "Sample spacing (m)"; minimum: 0.04; maximum: 0.25; decimals: 3; help: "Upstream sampling; enabled bumps subdivide the final collision mesh." }
      Fields.RuntimeField { id: heightRange; objectName: "groundHeight"; label: "Base height range (m)"; minimum: 0.005; maximum: 0.08; decimals: 3; help: "Upstream elevation range, before wheel-scale relief." }
      Fields.RuntimeField { id: padding; objectName: "groundHeadland"; label: "Ground padding (m)"; minimum: 3; maximum: 8; help: "Mesh extent padding, not crop turning space. Must support the entire house and robot." }
      Fields.RuntimeField { id: roughness; objectName: "groundRms"; label: "Bump band RMS (mm)"; minimum: 0; maximum: 20; decimals: 1 }
      Fields.RuntimeField { id: bumpScale; objectName: "groundScale"; label: "Bump scale (m)"; minimum: 0.16; maximum: 0.5; decimals: 3 }
      Label { text: "Contact"; font.bold: true; Layout.topMargin: 4 }
      Fields.RuntimeField { id: friction; objectName: "groundFriction"; label: "Ground friction"; minimum: 0.1; maximum: 1.2; help: "Isotropic DART Coulomb friction. The stock wheel coefficient 0.5 can limit effective traction." }
      Fields.RuntimeField { id: longitudinal; objectName: "slipLongitudinal"; label: "Slip: longitudinal"; minimum: 0; maximum: 2; help: "WheelSlip compliance, not percent slip; 40 N is an estimated normalization." }
      Fields.RuntimeField { id: lateral; objectName: "slipLateral"; label: "Slip: lateral"; minimum: 0; maximum: 2; help: "Skid steering needs lateral slip. Zero is not the goal." }
      Label { text: "Robot"; font.bold: true; Layout.topMargin: 4 }
      Label { text: "Lift payload: 2 x (C-200T + camera), startup only"; wrapMode: Text.WordWrap; Layout.fillWidth: true }
      GridLayout {
        columns: 2
        Layout.fillWidth: true
        Label { text: "Drive (m/s)" }
        SpinBox { objectName: "driveSpeed"; implicitHeight: 34; Layout.fillWidth: true; Layout.minimumWidth: 110; Layout.preferredWidth: 130; from: 0; to: 200; value: 200; editable: true; onValueModified: jackalControls.driveSpeed = value / 100; textFromValue: function(v) { return (v / 100).toFixed(2) }; valueFromText: function(t) { return Number(t)*100 } }
        Label { text: "Turn (rad/s)" }
        SpinBox { implicitHeight: 34; Layout.fillWidth: true; Layout.minimumWidth: 110; Layout.preferredWidth: 130; from: 0; to: 400; value: 400; editable: true; onValueModified: jackalControls.turnSpeed = value / 100; textFromValue: function(v) { return (v / 100).toFixed(2) }; valueFromText: function(t) { return Number(t)*100 } }
        Label { text: "Lift speed (mm/s)" }
        SpinBox { id: liftSpeed; objectName: "liftSpeed"; implicitHeight: 34; Layout.fillWidth: true; Layout.minimumWidth: 110; Layout.preferredWidth: 130; from: 5; to: Math.round(jackalControls.maxLiftSpeed*1000); value: 100; editable: true }
        Label { text: "Lift target (mm)" }
        SpinBox { id: target; objectName: "liftTarget"; implicitHeight: 34; Layout.fillWidth: true; Layout.minimumWidth: 110; Layout.preferredWidth: 130; from: 0; to: Math.floor(jackalControls.stroke*1000); value: 0; editable: true }
      }
      Button { objectName: "applyLift"; text: "Apply lift"; implicitHeight: 32; Layout.fillWidth: true; enabled: jackalControls.ready && !jackalControls.busy; onClicked: jackalControls.ApplyLift(0, Math.floor(jackalControls.stroke*1000), liftSpeed.value, target.value) }
      Label { text: "Gimbal cameras (XF C-200T)"; font.bold: true; Layout.topMargin: 4 }
      Fields.RuntimeField { objectName: "gimbalKeySpeed"; label: "Key rotation speed (deg/s)"; minimum: 0.1; maximum: 200; decimals: 1; number: 5
        help: "How fast a held P / R / Y key turns the gimbal, for every axis of both cameras."
        onNumberChanged: jackalControls.gimbalKeySpeed = number }
      Fields.GimbalFields { camera: 1; title: "Camera 1 - left"; Layout.fillWidth: true }
      Fields.GimbalFields { camera: 2; title: "Camera 2 - right"; Layout.fillWidth: true }
    }
  }
  Label { text: "Navigation"; font.bold: true; Layout.topMargin: 6 }
  Fields.RuntimeField { id: goalX; objectName: "goalX"; label: "X (m)"; minimum: -25; maximum: 25; number: -6.5 }
  Fields.RuntimeField { id: goalY; objectName: "goalY"; label: "Y (m)"; minimum: -25; maximum: 25; number: 0.0 }
  Fields.RuntimeField { id: goalYaw; objectName: "goalYaw"; label: "Rotation Z (deg)"; minimum: -180; maximum: 180; decimals: 1; number: 0 }
  RowLayout {
    Layout.fillWidth: true
    Button { objectName: "navigateGo"; text: "Go"; Layout.fillWidth: true; implicitHeight: 32;
      enabled: jackalControls.navigationReady && !jackalControls.busy && !jackalControls.navigating
      onClicked: jackalControls.Navigate(goalX.number, goalY.number, goalYaw.number) }
    Button { objectName: "navigateCancel"; text: "Cancel"; Layout.fillWidth: true; implicitHeight: 32;
      onClicked: jackalControls.CancelNavigation() }
  }
  Label { objectName: "navigationStatus"; text: jackalControls.navigationStatus; wrapMode: Text.WordWrap;
    Layout.fillWidth: true; Layout.preferredHeight: 48; maximumLineCount: 3; elide: Text.ElideRight }
}
  }
}
