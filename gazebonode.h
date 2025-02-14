#ifndef GAZEBONODE_H
#define GAZEBONODE_H

#include <QTimer>
#include <QDebug>
#include <QObject>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QDataStream>

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
    explicit GazeboNode(QString host, quint16 port, std::string auv_name, bool evologics, QObject *parent = nullptr);

private:
    void positionCallback(const gz::msgs::Pose_V &pose);

private slots:
    void sendPosition();

private:
    gz::transport::Node _node;
    gz::msgs::Vector3d _position;

    QString _host;
    quint16 _port;

    QTimer *_timer;

    std::unique_ptr<QTcpSocket> _evologicsSocket;
    std::unique_ptr<QUdpSocket> _udpPositionSocket;
};

#endif // GAZEBONODE_H
