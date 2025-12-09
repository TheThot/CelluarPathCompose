#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "src/decomposer.h"

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    // Регистрируем C++ класс в QML
    qmlRegisterType<Decomposer>("Decomposer", 1, 0, "Decomposer");

    QQmlApplicationEngine engine;
    engine.load(QUrl("qrc:/qml/main.qml"));
    // engine.load(QUrl(QStringLiteral("main.qml")));
    // engine.load(QUrl(QStringLiteral("://main.qml")));
    // engine.load(QUrl(QStringLiteral("..//main.qml")));

    return app.exec();
}
