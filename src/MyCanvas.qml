import QtQuick 2.15

Canvas {
    id: _root
    property var decomposer
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

        ctx.save()
        ctx.translate(offsetX, offsetY)
        ctx.scale(scaleFactor, scaleFactor)

        drawSweepLine(ctx)

        if (decomposer.showDecomposition) {
            showPolyWithHoles ? drawBPDDecomposition(ctx) : drawDecomposition(ctx)
        }

        if (decomposer.showOrientedRect) {
            showPolyWithHoles ? drawOrientedHoleRects(ctx) : drawOrientedRect(ctx)
        }

        showPolyWithHoles ? drawOriginalPolygonWithHoles(ctx) : drawOriginalPolygon(ctx)

        ctx.restore()
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

        drawPolygon(ctx, polygon, "green", "rgba(0, 0, 0, 0)", 2)

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
            "rgba(102, 204, 204, 0.3)"
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

        if (dashed) {
            ctx.setLineDash([])
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