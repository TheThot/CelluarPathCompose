import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import Decomposer 1.0

Window {
    id: root
    visible: true
    width: 1200
    height: 900
    title: "Polygon Decomposition Visualizer"
    color: "#f5f5f5"

    function getRandom(previousValue) {
        return Math.floor(previousValue + Math.random() * 90) % 360;
    }

    Decomposer {
        id: decomposer
        sweepAngle: angleSlider.value
        showDecomposition: showDecompositionCheck.checked
        showOrientedRect: showOrientedRectCheck.checked
    }

    // Панель инструментов
    Rectangle {
        id: controlPanel
        width: 250
        height: parent.height
        color: "#2c3e50"

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
                onValueChanged: decomposer.updateDecomposition()
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
                    onClicked: decomposer.resetPolygon()
                }

                Button {
                    text: "Star"
                    width: parent.width
                    onClicked: {
                        var starPoints = []
                        var points = 5
                        var outerRadius = 150
                        var innerRadius = 70
                        for (var i = 0; i < points * 2; i++) {
                            var radius = (i % 2 == 0) ? outerRadius : innerRadius
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

            // Кнопка обновления
            Button {
                text: "Update Decomposition"
                width: parent.width
                highlighted: true
                onClicked: decomposer.updateDecomposition()
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

    // Область визуализации
    Rectangle {
        id: visualizationArea
        anchors {
            left: controlPanel.right
            right: parent.right
            top: parent.top
            bottom: parent.bottom
            margins: 10
        }
        color: "white"
        border.color: "#bdc3c7"
        border.width: 2
        clip: true

        // Канвас для рисования
        Canvas {
            id: canvas
            anchors.fill: parent
            renderTarget: Canvas.FramebufferObject
            antialiasing: true

            property real scaleFactor: Math.min(width / 600, height / 500)
            property real offsetX: (width - 600 * scaleFactor) / 2
            property real offsetY: (height - 500 * scaleFactor) / 2

            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                ctx.clearRect(0, 0, width, height)

                // Сохраняем контекст
                ctx.save()

                // Применяем масштабирование и смещение
                ctx.translate(offsetX, offsetY)
                ctx.scale(scaleFactor, scaleFactor)

                // Рисуем sweep line
                drawSweepLine(ctx)

                // Рисуем декомпозицию если включено
                if (decomposer.showDecomposition) {
                    drawDecomposition(ctx)
                }

                // Рисуем ориентированный прямоугольник если включено
                if (decomposer.showOrientedRect) {
                    drawOrientedRect(ctx)
                }

                // Рисуем исходный полигон
                drawOriginalPolygon(ctx)

                // Восстанавливаем контекст
                ctx.restore()
            }

            function drawOriginalPolygon(ctx) {
                var polygon = decomposer.originalPolygon
                if (!polygon || polygon.length < 3) return

                ctx.beginPath()
                ctx.moveTo(polygon[0].x, polygon[0].y)

                for (var i = 1; i < polygon.length; i++) {
                    ctx.lineTo(polygon[i].x, polygon[i].y)
                }
                ctx.closePath()

                ctx.fillStyle = "rgba(255, 0, 0, 0.2)"
                ctx.strokeStyle = "red"
                ctx.lineWidth = 2
                ctx.fill()
                ctx.stroke()

                // Рисуем вершины
                for (var j = 0; j < polygon.length; j++) {
                    ctx.beginPath()
                    ctx.arc(polygon[j].x, polygon[j].y, 4, 0, Math.PI * 2)
                    ctx.fillStyle = "red"
                    ctx.fill()
                    ctx.strokeStyle = "darkred"
                    ctx.lineWidth = 1
                    ctx.stroke()
                }
            }

            function drawDecomposition(ctx) {
                var cells = decomposer.decompositionCells
                if (!cells || cells.length === 0) return

                // Цвета для ячеек
                var colors = [
                    "rgba(51, 181, 229, 0.3)",
                    "rgba(153, 204, 0, 0.3)",
                    "rgba(255, 187, 51, 0.3)",
                    "rgba(255, 68, 68, 0.3)",
                    "rgba(170, 102, 204, 0.3)",
                    "rgba(102, 204, 204, 0.3)"
                ]

                for (var i = 0; i < cells.length; i++) {
                    var cell = cells[i]
                    if (!cell.points || cell.points.length < 3) continue

                    ctx.beginPath()
                    ctx.moveTo(cell.points[0].x, cell.points[0].y)

                    for (var j = 1; j < cell.points.length; j++) {
                        ctx.lineTo(cell.points[j].x, cell.points[j].y)
                    }
                    ctx.closePath()

                    ctx.fillStyle = colors[i % colors.length]
                    ctx.strokeStyle = "#33B5E5"
                    ctx.lineWidth = 1
                    ctx.fill()
                    ctx.stroke()

                    // Подпись площади
                    if (cell.area > 10) {
                        ctx.fillStyle = "#33B5E5"
                        ctx.font = "10px Arial"
                        ctx.textAlign = "center"
                        ctx.textBaseline = "middle"

                        // Находим центр ячейки
                        var centerX = 0, centerY = 0
                        for (var k = 0; k < cell.points.length; k++) {
                            centerX += cell.points[k].x
                            centerY += cell.points[k].y
                        }
                        centerX /= cell.points.length
                        centerY /= cell.points.length

                        ctx.fillText(cell.area.toFixed(1), centerX, centerY)
                    }
                }
            }

            function drawOrientedRect(ctx) {
                var rect = decomposer.orientedRect
                if (!rect || rect.length !== 4) return

                ctx.beginPath()
                ctx.moveTo(rect[0].x, rect[0].y)

                for (var i = 1; i < rect.length; i++) {
                    ctx.lineTo(rect[i].x, rect[i].y)
                }
                ctx.closePath()

                ctx.fillStyle = "rgba(153, 204, 0, 0.1)"
                ctx.strokeStyle = "#99CC00"
                ctx.lineWidth = 2
                ctx.setLineDash([5, 5])
                ctx.fill()
                ctx.stroke()
                ctx.setLineDash([])
            }

            function drawSweepLine(ctx) {
                var angle = decomposer.sweepAngle
                if (angle === 0) return // Горизонтальная линия не рисуется явно

                ctx.save()

                // Поворачиваем контекст
                var centerX = 300
                var centerY = 250
                ctx.translate(centerX, centerY)
                ctx.rotate(-angle * Math.PI / 180)

                // Рисуем sweep line через весь экран
                ctx.beginPath()
                ctx.moveTo(-1000, 0)
                ctx.lineTo(1000, 0)
                ctx.strokeStyle = "#7f8c8d"
                ctx.lineWidth = 1
                ctx.setLineDash([3, 3])
                ctx.stroke()
                ctx.setLineDash([])

                // Стрелка направления
                ctx.beginPath()
                ctx.moveTo(50, 0)
                ctx.lineTo(30, -5)
                ctx.lineTo(30, 5)
                ctx.closePath()
                ctx.fillStyle = "#7f8c8d"
                ctx.fill()

                ctx.restore()

                // Текст с углом
                ctx.fillStyle = "#7f8c8d"
                ctx.font = "bold 12px Arial"
                ctx.textAlign = "center"
                ctx.fillText("Sweep Line: " + angle.toFixed(1) + "°",
                    centerX + Math.cos(angle * Math.PI / 180) * 70,
                    centerY - Math.sin(angle * Math.PI / 180) * 70)
            }
        }

        // Кнопка перерисовки
        Button {
            anchors {
                top: parent.top
                right: parent.right
                margins: 10
            }
            text: "Refresh"
            onClicked: canvas.requestPaint()
        }

        // Информационная панель
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
                    text: "Angle: " + angleSlider.value.toFixed(1) + "°"
                    color: "#34495e"
                }
            }
        }
    }

    // Обновление канваса при изменении свойств
    Connections {
        target: decomposer
        function onDecompositionCellsChanged() { canvas.requestPaint() }
        function onOrientedRectChanged() { canvas.requestPaint() }
        function onOriginalPolygonChanged() { canvas.requestPaint() }
        function onSweepAngleChanged() { canvas.requestPaint() }
        function onShowDecompositionChanged() { canvas.requestPaint() }
        function onShowOrientedRectChanged() { canvas.requestPaint() }
    }

    // Таймер для плавной анимации угла
    Timer {
        id: animationTimer
        interval: 50
        repeat: true
        running: animateCheck.checked

        property real animationAngle: 0
        property real direction: 1

        onTriggered: {
            animationAngle += direction * 2
            if (animationAngle >= 180 || animationAngle <= 0) {
                direction *= -1
            }
            angleSlider.value = animationAngle
        }
    }

    // Флажок анимации
    CheckBox {
        id: animateCheck
        anchors {
            bottom: visualizationArea.bottom
            right: visualizationArea.right
            margins: 10
        }
        text: "Animate Angle"

        contentItem: Text {
            text: animateCheck.text
            color: "#2c3e50"
            font.pixelSize: 12
            leftPadding: animateCheck.indicator.width + 5
        }
    }
}
