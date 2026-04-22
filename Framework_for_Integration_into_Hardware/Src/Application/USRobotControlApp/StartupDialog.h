#ifndef STARTUPDIALOG_H
#define STARTUPDIALOG_H

#include <QDialog>
#include <QString>

namespace Ui {
class StartupDialog;
}

class StartupDialog : public QDialog
{
    Q_OBJECT

public:
    explicit StartupDialog(QWidget *parent = nullptr);
    ~StartupDialog();

    // Access functions
    QString getRobotVersion();
    bool getSimulationMode();

private:
    Ui::StartupDialog *ui;
};

#endif // STARTUPDIALOG_H
