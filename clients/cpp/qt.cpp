/*
 * Make sure to have the add-on "ZMQ remote API" running in
 * CoppeliaSim and have following scene loaded:
 *
 * scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
 *
 * Do not launch simulation, but run this script
 */

#include "RemoteAPIClient.h"

#include <QtCore>
#include <QtWidgets>

int main(int argc, char **argv)
{
    RemoteAPIClient client;

    auto visionSensorHandle = client.call("sim.getObject", {"/VisionSensor"})[0];

    client.setStepping(true);
    client.call("sim.startSimulation");

    QApplication app(argc, argv);
    auto mainWindow = new QMainWindow();
    auto vbox = new QVBoxLayout();
    auto centralWidget = new QWidget();
    mainWindow->setCentralWidget(centralWidget);
    centralWidget->setLayout(vbox);
    auto btnStep = new QPushButton("Step simulation");
    auto label = new QLabel();
    vbox->addWidget(label);
    vbox->addWidget(btnStep);
    QObject::connect(btnStep, &QPushButton::clicked, [&] {
        client.step();
        auto ret = client.call("sim.getVisionSensorCharImage", {visionSensorHandle});
        auto img = ret[0].as<std::vector<uint8_t>>();
        auto resX = ret[1].as<int>();
        auto resY = ret[2].as<int>();
        QImage image(img.data(), resX, resY, QImage::Format_RGB888);
        label->setMinimumSize(resX, resY);
        label->setPixmap(QPixmap::fromImage(image.mirrored(), Qt::AutoColor));
    });
    mainWindow->show();
    btnStep->clicked();
    auto ret = app.exec();

    client.call("sim.stopSimulation");

    return ret;
}

