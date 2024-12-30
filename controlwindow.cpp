#include "controlwindow.h"
#include "./ui_controlwindow.h"

ControlWindow::ControlWindow(RosNode *rosNode, GazeboNode *gazeboNode, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::ControlWindow)
    , _rosNode(rosNode)
    , _gazeboNode(gazeboNode)
{
    ui->setupUi(this);

    ui->propellerSlider->setMaximum(100);
    ui->propellerSlider->setValue(ui->propellerSlider->maximum() / 2);
    connect(ui->propellerSlider, SIGNAL(valueChanged(int)), SLOT(getPropeller(int)));

    ui->verticalFinsSlider->setMaximum(100);
    ui->verticalFinsSlider->setValue(ui->verticalFinsSlider->maximum() / 2);
    connect(ui->verticalFinsSlider, SIGNAL(valueChanged(int)), SLOT(getVerticalFins(int)));

    ui->horizontalFinsSlider->setMaximum(100);
    ui->horizontalFinsSlider->setValue(ui->horizontalFinsSlider->maximum() / 2);
    connect(ui->horizontalFinsSlider, SIGNAL(valueChanged(int)), SLOT(getHorizontalFins(int)));

    ui->propellerBox->setMinimum(-50);
    ui->propellerBox->setMaximum(50);
    ui->propellerBox->setSingleStep(1);
    connect(ui->propellerBox, SIGNAL(valueChanged(double)), SLOT(getPropeller(double)));

    ui->verticalFinsBox->setMinimum(-0.25);
    ui->verticalFinsBox->setMaximum(0.25);
    ui->verticalFinsBox->setSingleStep(0.05);
    connect(ui->verticalFinsBox, SIGNAL(valueChanged(double)), SLOT(getVerticalFins(double)));

    ui->horizontalFinsBox->setMinimum(-0.25);
    ui->horizontalFinsBox->setMaximum(0.25);
    ui->horizontalFinsBox->setSingleStep(0.05);
    connect(ui->horizontalFinsBox, SIGNAL(valueChanged(double)), SLOT(getHorizontalFins(double)));

    connect(ui->disabledRadio, &QRadioButton::toggled, this, &ControlWindow::onDynamicPositionToggled);
    connect(ui->udpPositionRadio, &QRadioButton::toggled, this, &ControlWindow::onDynamicPositionToggled);
    connect(ui->evologicsRadio, &QRadioButton::toggled, this, &ControlWindow::onDynamicPositionToggled);
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

void ControlWindow::onDynamicPositionToggled(bool checked)
{
    if (checked)
    {
        QRadioButton *btn = static_cast<QRadioButton *>(sender());

        if (btn->objectName() == ui->disabledRadio->objectName())
        {
            _gazeboNode->stopUdpPosition();
            _gazeboNode->stopEvologicsPosition();
        }
        else if (btn->objectName() == ui->udpPositionRadio->objectName())
        {
            _gazeboNode->startUdpPosition();
            _gazeboNode->stopEvologicsPosition();
        }
        else if (btn->objectName() == ui->evologicsRadio->objectName())
        {
            _gazeboNode->stopUdpPosition();
            _gazeboNode->startEvologicsPosition();
        }
    }

}
