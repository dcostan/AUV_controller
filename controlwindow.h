#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QMainWindow>
#include <QSlider>

#include "rosnode.h"
#include "gazebonode.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class ControlWindow;
}
QT_END_NAMESPACE

class ControlWindow : public QMainWindow
{
    Q_OBJECT

public:
    ControlWindow(RosNode *rosNode, GazeboNode *gazeboNode, QWidget *parent = nullptr);
    ~ControlWindow();

public slots:
    void getPropeller(int k);
    void getPropeller(double k);

    void getVerticalFins(int k);
    void getVerticalFins(double k);

    void getHorizontalFins(int k);
    void getHorizontalFins(double k);

    void onDynamicPositionToggled(bool checked);

private:
    Ui::ControlWindow *ui;
    RosNode *_rosNode;
    GazeboNode *_gazeboNode;
};
#endif // CONTROLWINDOW_H