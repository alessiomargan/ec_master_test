//*************************************************
//*************************************************


#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "calibgui.h"

int main(int argc, char *argv[])
{
    CalibGui data;

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("applicationData", &data);
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    return app.exec();
}
