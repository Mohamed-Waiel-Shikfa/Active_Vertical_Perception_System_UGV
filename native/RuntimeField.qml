import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

RowLayout {
  id: row
  property string label
  property string help
  property real number: 0
  property real minimum: 0
  property real maximum: 1
  property int decimals: 2
  property real multiplier: Math.pow(10, decimals)
  property alias editor: input
  Layout.fillWidth: true
  Label { text: row.label; Layout.fillWidth: true; wrapMode: Text.WordWrap
    ToolTip.text: row.help; ToolTip.visible: labelMouse.containsMouse
    MouseArea { id: labelMouse; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
  }
  SpinBox {
    id: input
    objectName: row.objectName + "Input"
    Layout.preferredWidth: 125
    implicitHeight: 32
    editable: true
    validator: DoubleValidator {
      bottom: row.minimum
      top: row.maximum
      decimals: row.decimals
      notation: DoubleValidator.StandardNotation
    }
    from: Math.round(row.minimum * row.multiplier)
    to: Math.round(row.maximum * row.multiplier)
    value: Math.round(row.number * row.multiplier)
    textFromValue: function(v) { return (v / row.multiplier).toFixed(row.decimals) }
    valueFromText: function(t) { return Math.round(Number(t) * row.multiplier) }
    onValueModified: row.number = value / row.multiplier
  }
}
