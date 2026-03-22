import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0

// Панель инструментов
Rectangle {
    id: _root
    width: 280
    height: parent.height
    color: "#2c3e50"

    property var _decomposer
    property bool buttonsVisibleProp : true

    property alias sweepAngle: angleSlider.value
    property alias showDecomposition: showDecompositionCheck.checked
    property alias showOrientedRect: showOrientedRectCheck.checked
    property alias showPathCoverage: showPathCoverageCheck.checked
    property alias widthSpace: trWSlider.value

    function getRandom(previousValue) {
        return Math.floor(previousValue + Math.random() * 90) % 360;
    }

    function testArrayOut(){
        var test = decomposer.test_2Darray;
        console.log("Test size is ", test.length);
        console.log("Iterate throught holes");
        for (var onePoly of test) {
            console.log("Curr poly size is ", onePoly.length);
            console.log("Show one poly path");
            for(var row of onePoly){
                console.log("Poly coord is ", row);
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // Заголовок
        Text {
            text: "DECOMPOSITION VISUALIZER"
            color: "white"
            font.bold: true
            font.pixelSize: 16
            Layout.alignment: Qt.AlignHCenter
        }

        // Разделитель
        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#7f8c8d"
        }

        Column {
            spacing: 0
            // padding: 0
            ButtonGroup {
                id: polySetup
                onClicked: {
                    decomposer.resetToPolygonWithHoleState();
                    buttonsVisibleProp = rdbtn1.checked;
                    // testArrayOut();
                }   // обработка выбора переключателя
            }
            RadioButton {
                id: rdbtn1
                ButtonGroup.group: polySetup
                width: 20
                contentItem: Text {
                    text: "Simpl poly"
                    font.pixelSize: 14
                    color: "white"
                    anchors.left: parent.left
                    anchors.leftMargin: 30  // Отступ слева
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
            RadioButton {
                id: rdbtn2
                ButtonGroup.group: polySetup
                width: 20
                contentItem: Text {
                    text: "Poly with holes"
                    font.pixelSize: 14
                    color: "white"
                    anchors.left: parent.left
                    anchors.leftMargin: 30  // Отступ слева
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
            Component.onCompleted: {
                rdbtn1.checked = true
            }
        }

        // Кнопки для смены полигона
        Text {
            text: "Polygon Shapes:"
            color: "white"
            font.pixelSize: 14
            font.bold: true
        }

        Column {
            spacing: 5
            Layout.fillWidth: true

            Repeater{
                model: 3
                delegate: Button {
                    required property int index
                    property int showIdx:index+1
                    text: "Complex shape (" + showIdx + ")"
                    width: parent.width
                    onClicked: decomposer.resetPolygon(index)
                }
            }
        }

        // Разделитель
        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#7f8c8d"
        }

        // Управление углом
        Text {
            text: "Sweep Line Angle: " + angleSlider.value.toFixed(1) + "°"
            color: "white"
            font.pixelSize: 14
        }

        Slider {
            id: angleSlider
            Layout.fillWidth: true
            from: 0
            to: 180
            value: 0
            stepSize: 1
        }

        // Управление углом
        Text {
            text: "Path Space Width : " + trWSlider.value.toFixed(1)
            color: "white"
            font.pixelSize: 14
        }

        Slider {
            id: trWSlider
            Layout.fillWidth: true
            from: 5
            to: 60
            value: 30
            stepSize: 1
        }

        // Управление видимостью
        Column {
            Layout.fillWidth: true
            spacing: 8

            CheckBox {
                id: showDecompositionCheck
                checked: true
                text: "Show Decomposition"
                contentItem: Text {
                    text: showDecompositionCheck.text
                    color: "white"
                    font.pixelSize: 14
                    x: showDecompositionCheck.indicator.width + 10
                }
            }

            CheckBox {
                id: showOrientedRectCheck
                checked: false
                text: "Show Oriented Rect"
                visible: true
                contentItem: Text {
                    text: showOrientedRectCheck.text
                    color: "white"
                    font.pixelSize: 14
                    x: showOrientedRectCheck.indicator.width + 10
                }
            }

            CheckBox {
                id: showPathCoverageCheck
                checked: false
                text: "Show Path Coverage"
                contentItem: Text {
                    text: showPathCoverageCheck.text
                    color: "white"
                    font.pixelSize: 14
                    x: showPathCoverageCheck.indicator.width + 10
                }
            }
        }

        // Кнопки для смены полигона
        Text {
            text: "Polygon Shapes:"
            color: "white"
            font.pixelSize: 14
            font.bold: true
        }

        // Информация
        Text {
            text: "Info:"
            color: "white"
            font.pixelSize: 14
            font.bold: true
        }

        Text {
            text: "Cells: " + (decomposer.decompositionCells ? decomposer.decompositionCells.length : 0)
            color: "white"
            font.pixelSize: 12
        }

        Text {
            text: "Sweep Line: " + angleSlider.value.toFixed(1) + "°"
            color: "white"
            font.pixelSize: 12
        }

        // Разделитель
        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#7f8c8d"
        }

        Item {
            Layout.fillHeight: true
        }
    }
}