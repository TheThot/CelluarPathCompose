//
// Created by Admin on 19.12.2025.
//

#ifndef TRYCELLUARPATHCOMPOSE_UTILS_H
#define TRYCELLUARPATHCOMPOSE_UTILS_H

#include <cmath>
#include <QObject>
#include <QPointF>

namespace baseFunc {
    /*class Utilz : public QObject {
        Q_OBJECT
    public:

    };*/
    void intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines)
    {
        resultLines.clear();

        for (const QLineF& line : lineList) {
            QList<QPointF> intersections;

            // Intersect the line with all the polygon edges
            for (int i=0; i<polygon.count(); ++i) {
                QLineF polygonEdge(polygon[i], polygon[(i+1) % polygon.count()]);
                QPointF intersectPoint;

                if (line.intersects(polygonEdge, &intersectPoint) == QLineF::BoundedIntersection) {
                    if (!intersections.contains(intersectPoint))
                        intersections.append(intersectPoint);
                }
            }

            std::sort(intersections.begin(), intersections.end(), [&line](const QPointF& pointStart, const QPointF& pointEnd) {
                return QLineF(line.p1(), pointStart).length() < QLineF(line.p1(), pointEnd).length();
            });

            for (int j=0; j+1<intersections.count(); j+=2)
                resultLines.append(QLineF(intersections[j], intersections[j+1]));
        }

        if (!resultLines.isEmpty()) {
            QList<QLineF> linesList;
            linesList.append(resultLines.first());
            resultLines.removeFirst();

            while (!resultLines.isEmpty()) {
                QLineF& tempLine = linesList.last();
                QPointF tempPoint = tempLine.p2();

                int indexNear = -1;
                double minDistance = std::numeric_limits<double>::max();
                bool reverse = false;

                for (int i = 0; i < resultLines.count(); ++i) {
                    double distanceP1 = QLineF(tempPoint, resultLines[i].p1()).length();
                    double distanceP2 = QLineF(tempPoint, resultLines[i].p2()).length();

                    if (distanceP1 < minDistance) {
                        minDistance = distanceP1;
                        indexNear = i;
                        reverse = false;
                    }
                    if (distanceP2 < minDistance) {
                        minDistance = distanceP2;
                        indexNear = i;
                        reverse = true;
                    }
                }

                if (indexNear != -1) {
                    QLineF nextLine = resultLines.takeAt(indexNear);
                    if (reverse) {
                        nextLine = QLineF(nextLine.p2(), nextLine.p1());
                    }
                    linesList.append(nextLine);
                }
                else
                    break;
            }
            resultLines = linesList;
        }
    }

    QPointF rotatePoint(const QPointF& point, const QPointF& origin, double angle)
    {
        QPointF rotated;

        double radians = (M_PI / 180.0) * -angle;

        rotated.setX(((point.x() - origin.x()) * cos(radians)) - ((point.y() - origin.y()) * sin(radians)) + origin.x());
        rotated.setY(((point.x() - origin.x()) * sin(radians)) + ((point.y() - origin.y()) * cos(radians)) + origin.y());

        return rotated;
    }
}
#endif //TRYCELLUARPATHCOMPOSE_UTILS_H
