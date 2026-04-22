#include <iostream>

#include "StartupDialog.h"
#include "ui_StartupDialog.h"

#include <QStringList>

#include <QDomDocument>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
StartupDialog::StartupDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::StartupDialog)
{
    ui->setupUi(this);
    //this->setAttribute(Qt::WA_DeleteOnClose);

    QDomDocument doc("robotInfo");
    XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);
    QDomNodeList robotVersionList = doc.elementsByTagName("Robot");

    // Read robot versions contained in xml file
    QStringList robotVersions;
    for (int s = 0; s < robotVersionList.length(); s++)
        robotVersions << robotVersionList.at(s).attributes().namedItem("version").nodeValue();

    // Add robot versions to list in dialog
    ui->comboBox_robotVersion->clear();
    ui->comboBox_robotVersion->addItems(robotVersions);
    ui->comboBox_robotVersion->setCurrentIndex(robotVersions.size() - 1);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &StartupDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &StartupDialog::reject);
}

StartupDialog::~StartupDialog()
{
    delete ui;
}

//----------------------------------------------------------------------
// Access functions
//----------------------------------------------------------------------
QString StartupDialog::getRobotVersion()
{
    return ui->comboBox_robotVersion->currentText();
}

bool StartupDialog::getSimulationMode()
{
    return ui->checkBox_simulated->isChecked();
}
