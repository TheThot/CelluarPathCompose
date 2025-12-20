import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

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
            padding: 0
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
                    horizontalAlignment: Text.AlignLeft
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: 30
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
                    horizontalAlignment: Text.AlignLeft
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: 30
                }
            }
            Component.onCompleted: {
                rdbtn1.checked = true
            }
        }
        /*RadioButton {
            contentItem: Loader {
                sourceComponent: control.checked ? checkedComponent : uncheckedComponent

                Component {
                    id: checkedComponent
                    Row {
                        spacing: 5
                        Image { source: "checked.svg" }
                        Text {
                            text: control.text
                            color: "green"
                            font.bold: true
                        }
                    }
                }

                Component {
                    id: uncheckedComponent
                    Text {
                        text: control.text
                        color: "gray"
                    }
                }
            }
        }*/

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
                    leftPadding: showDecompositionCheck.indicator.width + 10
                }
            }

            CheckBox {
                id: showOrientedRectCheck
                checked: true
                text: "Show Oriented Rect"
                contentItem: Text {
                    text: showOrientedRectCheck.text
                    color: "white"
                    font.pixelSize: 14
                    leftPadding: showOrientedRectCheck.indicator.width + 10
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
                    leftPadding: showPathCoverageCheck.indicator.width + 10
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

        Column {
            spacing: 5
            Layout.fillWidth: true

            Button {
                text: "Hexagon"
                width: parent.width
                visible: buttonsVisibleProp
                onClicked: decomposer.resetPolygon()
            }

            Button {
                text: "Star"
                width: parent.width
                visible: buttonsVisibleProp
                onClicked: {
                    var starPoints = []
                    var points = 5
                    var outerRadius = 150
                    var innerRadius = 70
                    for (var i = 0; i < points * 2; i++) {
                        var radius = (i % 2 === 0) ? outerRadius : innerRadius
                        var angle = Math.PI * i / points
                        starPoints.push({
                            "x": 300 + radius * Math.cos(angle),
                            "y": 250 + radius * Math.sin(angle)
                        })
                    }
                    decomposer.setOriginalPolygon(starPoints)
                }
            }

            Button {
                text: "Complex Shape"
                width: parent.width
                visible: buttonsVisibleProp
                onClicked: {
                    var x = getRandom(100);
                    var y = getRandom(x);
                    var complexPoints = []
                    for (var i = 0; i < 5; i++) {
                        complexPoints.push({"x": x, "y": y});
                        x = getRandom(y);
                        y = getRandom(x);
                    }
                    decomposer.setOriginalPolygon(complexPoints)
                }
            }
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

        // Легенда
        Text {
            text: "Legend:"
            color: "white"
            font.pixelSize: 14
            font.bold: true
        }

        Row {
            spacing: 10
            Rectangle {
                width: 15
                height: 15
                color: "red"
                radius: 3
            }
            Text {
                text: "Original Polygon"
                color: "white"
                font.pixelSize: 12
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Row {
            spacing: 10
            Rectangle {
                width: 15
                height: 15
                color: "#33B5E5"
                radius: 3
            }
            Text {
                text: "Decomposition Cells"
                color: "white"
                font.pixelSize: 12
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Row {
            spacing: 10
            Rectangle {
                width: 15
                height: 15
                color: "#99CC00"
                border.color: "black"
                border.width: 1
                radius: 3
            }
            Text {
                text: "Oriented Bounding Rect"
                color: "white"
                font.pixelSize: 12
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Item {
            Layout.fillHeight: true
        }
    }
}