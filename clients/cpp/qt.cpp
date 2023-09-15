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
    auto sim = client.getObject().sim();

    auto visionSensorHandle = sim.getObject("/VisionSensor");

    sim.setStepping(true);
    sim.startSimulation();

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
        sim.step();
        auto [img, res] = sim.getVisionSensorImg(visionSensorHandle);
        QImage image(img.data(), res[0], res[1], QImage::Format_RGB888);
        label->setMinimumSize(res[0], res[1]);
        label->setPixmap(QPixmap::fromImage(image.mirrored(), Qt::AutoColor));
    });
    mainWindow->show();
    btnStep->clicked();
    auto ret = app.exec();

    sim.stopSimulation();

    return ret;
}

