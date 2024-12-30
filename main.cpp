#include "controlwindow.h"
#include "gazebonode.h"
#include "rosnode.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    rclcpp::init(argc, argv);

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "AUV_controller_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            a.installTranslator(&translator);
            break;
        }
    }

    RosNode ros_node;
    GazeboNode gazebo_node;

    ControlWindow w(&ros_node, &gazebo_node);

    w.show();
    return a.exec();
}
