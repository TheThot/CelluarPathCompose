import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import Decomposer 1.0
import "."

Window {
    id: root
    visible: true
    width: 1200
    height: 900
    title: "Polygon Decomposition Visualizer"
    color: "#f5f5f5"

    Decomposer {
        id: decomposer

        property var pathGen: transects

        sweepAngle:         myToolbar.sweepAngle
        showDecomposition:  myToolbar.showDecomposition
        showOrientedRect:   myToolbar.showOrientedRect
        showPathCoverage:   myToolbar.showPathCoverage
        transectWidth:      myToolbar.widthSpace

    }

    Toolbar {
        id: myToolbar
        _decomposer: decomposer
    }

    Rectangle {
        id: visualizationArea
        anchors {
            left: myToolbar.right
            right: parent.right
            top: parent.top
            bottom: parent.bottom
            margins: 10
        }
        color: "white"
        border.color: "#bdc3c7"
        border.width: 2
        clip: true

        MyCanvas {
            id: canvas
            anchors.fill: parent
            decomposer: decomposer
            pathgenerator: decomposer.pathGen
            showPolyWithHoles: !myToolbar.buttonsVisibleProp
        }

        Rectangle {
            anchors {
                left: parent.left
                bottom: parent.bottom
                margins: 10
            }
            width: 200
            height: 80
            color: "white"
            border.color: "#bdc3c7"
            border.width: 1
            radius: 5

            Column {
                anchors.centerIn: parent
                spacing: 5
                Text {
                    text: "Decomposition Info"
                    font.bold: true
                    color: "#2c3e50"
                }
                Text {
                    text: "Cells: " + (decomposer.decompositionCells ? decomposer.decompositionCells.length : 0)
                    color: "#34495e"
                }
                Text {
                    text: "Angle: " + myToolbar.sweepAngle.toFixed(1) + "°"
                    color: "#34495e"
                }
            }
        }
    }

    Connections {
        target: decomposer
        function onDecompositionCellsChanged() { canvas.requestPaint() }
        function onOrientedRectChanged() { canvas.requestPaint() }
        function onOriginalPolygonChanged() { canvas.requestPaint() }
        function onSweepAngleChanged() { canvas.requestPaint() }
        function onShowDecompositionChanged() { canvas.requestPaint() }
        function onShowOrientedRectChanged() { canvas.requestPaint() }
        function ontrWdthChanged() { canvas.requestPaint() }
    }
}