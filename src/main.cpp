#include <QApplication>
#include "UIwindow.h"

using namespace ui_server;

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    UIWindow s(argc, argv);
    ROS_INFO("hi~ui");
    return app.exec();
}

