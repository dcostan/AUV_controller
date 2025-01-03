#ifndef GAZEBONODE_H
#define GAZEBONODE_H

#include <QObject>
#include <QTimer>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QDebug>
#include <string>
#include <thread>
#include <memory>

#ifndef Q_MOC_RUN

#include <gz/msgs.hh>
#include <gz/transport.hh>

#endif

class GazeboNode : public QObject
{
    Q_OBJECT
public:
    explicit GazeboNode(QObject *parent = nullptr);

    void startUdpPosition();
    void startEvologicsPosition();

    void stopUdpPosition();
    void stopEvologicsPosition();

private:
    void subscribe();
    void positionCallback(const gz::msgs::Pose_V &pose);

private slots:
    void sendPosition();

private:
    gz::transport::Node _node;
    gz::msgs::Vector3d _position;
    std::string _topic = "/model/my_lrauv/pose";

    QTimer *_timer;

    std::unique_ptr<QTcpSocket> _evologicsSocket;
    std::unique_ptr<QUdpSocket> _udpPositionSocket;
};

#endif // GAZEBONODE_H
