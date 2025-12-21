import QtQuick 2.15

Canvas {
    id: _root
    property var decomposer
    property var pathgenerator
    property bool showPolyWithHoles: false
    renderTarget: Canvas.FramebufferObject
    antialiasing: true

    property real scaleFactor: Math.min(width / 600, height / 500)
    property real offsetX: (width - 600 * scaleFactor) / 2
    property real offsetY: (height - 500 * scaleFactor) / 2

    onPaint: {
        var ctx = getContext("2d")
        ctx.reset()
        ctx.clearRect(0, 0, width, height)
        ctx.setLineDash([])  // Глобальный сброс пунктира!

        ctx.save()
        ctx.translate(offsetX, offsetY)
        ctx.scale(scaleFactor, scaleFactor)

        if(!decomposer.showPathCoverage){
            drawSweepLine(ctx)
        }

        if (decomposer.showPathCoverage) {
            drawTransects(ctx)
            drawSFpoints(ctx)
        }

        if (decomposer.showDecomposition) {
            showPolyWithHoles ? drawBPDDecomposition(ctx) : drawDecomposition(ctx)
        }

        if (decomposer.showOrientedRect) {
            showPolyWithHoles ? drawOrientedHoleRects(ctx) : drawOrientedRect(ctx)
        }

        showPolyWithHoles ? drawOriginalPolygonWithHoles(ctx) : drawOriginalPolygon(ctx)

        ctx.restore()
    }

    function drawTransects(ctx){
        var lines = pathgenerator.pathTraj

        drawLines(ctx, lines, "#101D6B")
    }

    function drawSFpoints(ctx){
        var pointS = pathgenerator.startP
        var pointF = pathgenerator.endP

        ctx.beginPath()
        ctx.arc(pointS.x, pointS.y, 10, 0, Math.PI * 2)
        ctx.fillStyle = "rgba(0, 0, 0, 0)"
        ctx.fill()
        ctx.strokeStyle = "red"
        ctx.lineWidth = 2
        ctx.stroke()

        ctx.beginPath()
        ctx.arc(pointF.x, pointF.y, 10, 0, Math.PI * 2)
        ctx.fillStyle = "rgba(0, 0, 0, 0)"
        ctx.fill()
        ctx.strokeStyle = "red"
        ctx.lineWidth = 2
        ctx.stroke()

        ctx.fillStyle = "red"
        ctx.font = "16px Arial"
        ctx.textAlign = "center"
        ctx.textBaseline = "middle"
        ctx.fillText("S", pointS.x, pointS.y)
        ctx.fillText("F", pointF.x, pointF.y)
    }

    function drawOriginalPolygon(ctx) {
        var polygon = decomposer.originalPolygon
        if (!polygon || polygon.length < 3) return

        drawPolygon(ctx, polygon, "red", "rgba(0, 0, 0, 0.2)", 2)
        drawVertices(ctx, polygon, "red", "darkred")
    }

    function drawOriginalPolygonWithHoles(ctx) {
        var polygon = decomposer.originalPolygon
        var holesArray = decomposer.holes_2Darray

        if (!polygon || polygon.length < 3) return

        drawPolygon(ctx, polygon, "rgba(0, 255, 0, 0.4)", "rgba(0, 0, 0, 0)", 2)

        holesArray.forEach(function(onePoly) {
            drawPolygon(ctx, onePoly, "red", "rgba(255, 0, 0, 0.4)", 2)
        })
    }

    function drawBPDDecomposition(ctx) {
        var cells = decomposer.bpdDecompositionCells
        if (!cells || cells.length === 0) return

        var colors = [
            "rgba(51, 181, 229, 0.3)",
            "rgba(153, 204, 0, 0.3)",
            "rgba(255, 187, 51, 0.3)",
            "rgba(255, 68, 68, 0.3)",
            "rgba(170, 102, 204, 0.3)",
            "rgba(102, 204, 204, 0.3)",
            "rgba(10, 255, 10, 0.3)"
        ]

        cells.forEach(function(cell, i) {
            if (!cell.points || cell.points.length < 3) return

            drawPolygon(ctx, cell.points, "#33B5E5", colors[i % colors.length], 1)
            drawAreaLabel(ctx, cell)
        })
    }

    function drawDecomposition(ctx) {
        var cells = decomposer.decompositionCells
        if (!cells || cells.length === 0) return

        var colors = [
            "rgba(51, 181, 229, 0.3)",
            "rgba(153, 204, 0, 0.3)",
            "rgba(255, 187, 51, 0.3)",
            "rgba(255, 68, 68, 0.3)",
            "rgba(170, 102, 204, 0.3)",
            "rgba(102, 204, 204, 0.3)"
        ]

        cells.forEach(function(cell, i) {
            if (!cell.points || cell.points.length < 3) return

            drawPolygon(ctx, cell.points, "#33B5E5", colors[i % colors.length], 1)
            drawAreaLabel(ctx, cell)
        })
    }

    function drawOrientedRect(ctx) {
        var rect = decomposer.orientedRect
        if (!rect || rect.length !== 4) return

        drawPolygon(ctx, rect, "#99CC00", "rgba(153, 204, 0, 0.1)", 2, true)
    }

    function drawOrientedHoleRects(ctx) {
        var rectArray = decomposer.orientedHoleRects
        if (!rectArray || rectArray[0].length !== 4) return

        rectArray.forEach(function(currRect) {
            drawPolygon(ctx, currRect, "#99CC00", "rgba(153, 204, 0, 0.1)", 2, true)
        })
    }

    function drawLines(ctx, lines, color){
        ctx.save()
        // ctx.setLineDash([])  // Явно сбрасываем пунктир
        if(!showPolyWithHoles) {
            for (var line of lines) {
                ctx.beginPath()
                ctx.moveTo(line.p1.x, line.p1.y)  // Начало линии
                ctx.lineTo(line.p2.x, line.p2.y)  // Конец линии
                ctx.strokeStyle = color
                ctx.lineWidth = 4
                ctx.stroke()
            }
        }else{
            for (var oneDivLine of lines) {
                for(var i = 0; i < oneDivLine.length-1; i+=2){
                    ctx.beginPath()
                    ctx.moveTo(oneDivLine[i].x, oneDivLine[i].y)  // Начало линии
                    ctx.lineTo(oneDivLine[i+1].x, oneDivLine[i+1].y)  // Конец линии
                    ctx.strokeStyle = color
                    ctx.lineWidth = 4
                    ctx.stroke()
                }

            }
        }

        ctx.restore()  // восстановить состояние контекста
    }

    function drawSweepLine(ctx) {

        var angle = decomposer.sweepAngle
        if (angle === 0) return

        ctx.save()

        var centerX = 300, centerY = 250
        ctx.translate(centerX, centerY)
        ctx.rotate(-angle * Math.PI / 180)

        ctx.beginPath()
        ctx.moveTo(-1000, 0)
        ctx.lineTo(1000, 0)
        ctx.strokeStyle = "#7f8c8d"
        ctx.lineWidth = 1
        ctx.setLineDash([3, 3])
        ctx.stroke()
        ctx.setLineDash([])

        ctx.beginPath()
        ctx.moveTo(50, 0)
        ctx.lineTo(30, -5)
        ctx.lineTo(30, 5)
        ctx.closePath()
        ctx.fillStyle = "#7f8c8d"
        ctx.fill()

        ctx.restore()

        ctx.fillStyle = "#7f8c8d"
        ctx.font = "bold 12px Arial"
        ctx.textAlign = "center"
        ctx.fillText("Sweep Line: " + angle.toFixed(1) + "°",
            centerX + Math.cos(angle * Math.PI / 180) * 70,
            centerY - Math.sin(angle * Math.PI / 180) * 70)

    }

    // Helper functions
    function drawPolygon(ctx, points, strokeColor, fillColor, lineWidth, dashed = false) {
        if (!points || points.length < 3) return

        ctx.save()  // Сохраняем состояние

        try {
            ctx.beginPath()
            ctx.moveTo(points[0].x, points[0].y)

            for (var i = 1; i < points.length; i++) {
                ctx.lineTo(points[i].x, points[i].y)
            }
            ctx.closePath()

            ctx.fillStyle = fillColor
            ctx.strokeStyle = strokeColor
            ctx.lineWidth = lineWidth

            if (dashed) {
                ctx.setLineDash([5, 5])
            }

            ctx.fill()
            ctx.stroke()
        } finally {
            // Всегда сбрасываем пунктир и восстанавливаем состояние
            ctx.setLineDash([])
            ctx.restore()
        }

    }

    function drawVertices(ctx, polygon, fillColor, strokeColor) {
        polygon.forEach(function(vertex) {
            ctx.beginPath()
            ctx.arc(vertex.x, vertex.y, 4, 0, Math.PI * 2)
            ctx.fillStyle = fillColor
            ctx.fill()
            ctx.strokeStyle = strokeColor
            ctx.lineWidth = 1
            ctx.stroke()
        })
    }

    function drawAreaLabel(ctx, cell) {
        if (cell.area <= 10) return

        var centerX = 0, centerY = 0
        cell.points.forEach(function(point) {
            centerX += point.x
            centerY += point.y
        })
        centerX /= cell.points.length
        centerY /= cell.points.length

        ctx.fillStyle = "#33B5E5"
        ctx.font = "10px Arial"
        ctx.textAlign = "center"
        ctx.textBaseline = "middle"
        ctx.fillText(cell.area.toFixed(1), centerX, centerY)
    }
}