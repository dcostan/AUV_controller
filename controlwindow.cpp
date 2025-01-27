#include "controlwindow.h"
#include "./ui_controlwindow.h"

ControlWindow::ControlWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::ControlWindow)
    , _rosNode(std::make_unique<RosNode>(this))
{
    ui->setupUi(this);
    ui->portBox->setValidator(new QIntValidator(0, 65535, this));
}

ControlWindow::~ControlWindow()
{
    delete ui;
}

void ControlWindow::getPropeller(int k)
{
    double adjusted_value = k - (ui->propellerSlider->maximum() / 2);

    ui->propellerBox->setValue(adjusted_value);
}

void ControlWindow::getPropeller(double k)
{
    double reversed_value = k + (ui->propellerSlider->maximum() / 2);

    _rosNode->propellerCallback(k);

    ui->propellerSlider->setValue(reversed_value);
}

void ControlWindow::getVerticalFins(int k)
{
    double adjusted_value = (k - (ui->verticalFinsSlider->maximum() / 2)) * 0.25 / 50.0;

    ui->verticalFinsBox->setValue(adjusted_value);
}

void ControlWindow::getVerticalFins(double k)
{
    int reversed_value = (k * 50.0 / 0.25) + (ui->verticalFinsSlider->maximum() / 2);

    _rosNode->verticalFinsCallback(k);

    ui->verticalFinsSlider->setValue(reversed_value);
}

void ControlWindow::getHorizontalFins(int k)
{
    double adjusted_value = (k - (ui->horizontalFinsSlider->maximum() / 2)) * 0.25 / 50.0;

    ui->horizontalFinsBox->setValue(adjusted_value);
}

void ControlWindow::getHorizontalFins(double k)
{
    int reversed_value = (k * 50.0 / 0.25) + (ui->horizontalFinsSlider->maximum() / 2);

    _rosNode->horizontalFinsCallback(k);

    ui->horizontalFinsSlider->setValue(reversed_value);
}

void ControlWindow::restartGazeboTransport()
{
    _gazeboNode.reset();
    _gazeboNode = std::make_unique<GazeboNode>(ui->hostBox->text(), ui->portBox->text().toInt(), ui->nameBox->text().toStdString(), ui->evologicsRadio->isChecked(), this);
}

void ControlWindow::onDynamicPositionToggled(bool checked)
{
    if (checked)
    {
        QRadioButton *btn = static_cast<QRadioButton *>(sender());

        if (btn->objectName() == ui->disabledRadio->objectName())
        {
            ui->hostBox->setEnabled(false);
            ui->portBox->setEnabled(false);
            ui->nameBox->setEnabled(false);

            _gazeboNode.reset();
        }
        else if (btn->objectName() == ui->udpPositionRadio->objectName())
        {
            ui->hostBox->setEnabled(true);
            ui->portBox->setEnabled(true);
            ui->nameBox->setEnabled(true);

            _gazeboNode = std::make_unique<GazeboNode>(ui->hostBox->text(), ui->portBox->text().toInt(), ui->nameBox->text().toStdString(), false, this);
        }
        else if (btn->objectName() == ui->evologicsRadio->objectName())
        {
            ui->hostBox->setEnabled(true);
            ui->portBox->setEnabled(true);
            ui->nameBox->setEnabled(true);

            _gazeboNode = std::make_unique<GazeboNode>(ui->hostBox->text(), ui->portBox->text().toInt(), ui->nameBox->text().toStdString(), true, this);
        }
    }

}
