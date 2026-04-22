#include "StartupDialog.h"
#include "MainWindow.h"
#include <QSurfaceFormat>
#include <QApplication>
#include <QInputDialog>
#include <QStringList>
#include <QString>

#include <QVTKOpenGLNativeWidget.h>

int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication a(argc, argv);

    std::cerr << "C++ standard: " << __cplusplus << std::endl;

    /*bool ok;
    QStringList robotVersions;
    robotVersions << "1.0" << "2.2" << "3.3";
    QString item = QInputDialog::getItem(nullptr, "Choose robot version", "Robot version:", robotVersions, 2, false, &ok);
    if (!ok)
        return 1;*/

    //QDir::setCurrent(qApp->applicationDirPath() + "/../../../");

    RobotVer_t robotVersion;

    QString versionId;
    bool simulationMode;
    {
        StartupDialog su;
        su.setModal(true);
        if (su.exec() == QDialog::Accepted) {
            versionId = su.getRobotVersion();
            simulationMode = su.getSimulationMode();
        }
        else
            return 1;
    }

    if (versionId.compare("1.0") == 0)
        robotVersion = V1_ROBOT;
    else if (versionId.compare("2.2") == 0)
        robotVersion = V2_ROBOT;
    else if (versionId.compare("3.3") == 0)
        robotVersion = V3P3_ROBOT;
    else if (versionId.compare("3.4") == 0)
        robotVersion = V3P4_ROBOT;
    else
        robotVersion = UNKNOWN_ROBOT;

    MainWindow w(nullptr, robotVersion, simulationMode);
    if (!w.setupOk())
        return 1;
    w.show();

    return a.exec();
}
