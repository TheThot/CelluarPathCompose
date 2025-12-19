#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "src/decomposer.h"

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    // Регистрируем C++ класс в QML
    qmlRegisterType<Decomposer>("Decomposer", 1, 0, "Decomposer");
    qmlRegisterType<Decomposer>("PathGenerator", 1, 0, "PathGenerator");

    QQmlApplicationEngine engine;
    engine.load(QUrl("qrc:/qml/main.qml"));

    return app.exec();
}
