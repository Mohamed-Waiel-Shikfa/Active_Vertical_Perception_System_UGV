import QtQuick 2.12
import QtQuick.Controls 2.12

Rectangle {
  objectName: "gimbalCameraOverlay"
  anchors.fill: parent
  color: "black"
  visible: jackalControls.cameraView
  Image {
    anchors.fill: parent
    source: jackalControls.viewIndex > 0 && jackalControls.frameVersion > 0
            ? "image://gimbal_cameras/" + jackalControls.viewIndex + "/" + jackalControls.frameVersion : ""
    cache: false
    fillMode: Image.PreserveAspectFit
  }
  Label {
    anchors.left: parent.left
    anchors.top: parent.top
    anchors.margins: 12
    color: "white"
    padding: 6
    background: Rectangle { color: "#b0000000" }
    text: !jackalControls.cameraAvailable ? "Camera " + jackalControls.viewIndex + " frames unavailable"
          : jackalControls.viewIndex === 1 ? "Camera 1 - left C-200T gimbal - C: camera 2"
          : "Camera 2 - right C-200T gimbal - C: 3D view"
  }
}
