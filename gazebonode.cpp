#include "gazebonode.h"

GazeboNode::GazeboNode(QString host, quint16 port, std::string auv_name, bool evologics, QObject *parent)
    : QObject{parent}
    , _host(host)
    , _port(port)
    , _timer(new QTimer(this))
    , _evologicsSocket(nullptr)
    , _udpPositionSocket(nullptr)
{
    std::string topic_name = "/model/" + auv_name + "/pose";

    if (!_node.Subscribe(topic_name, &GazeboNode::positionCallback, this))
    {
        qDebug() << "Error subscribing to gazebo position topic";
    }

    if (evologics)
    {
        _evologicsSocket = std::make_unique<QTcpSocket>(this);
    }
    else
    {
        _udpPositionSocket = std::make_unique<QUdpSocket>(this);
    }

    connect(_timer, SIGNAL(timeout()), this, SLOT(sendPosition()));
    _timer->start(1000);
}

void GazeboNode::positionCallback(const gz::msgs::Pose_V &pose_v)
{
    _position = pose_v.pose(0).position();
}

void GazeboNode::sendPosition()
{
    if(_evologicsSocket)
    {
        _evologicsSocket->connectToHost(QHostAddress(_host), _port);

        if (!_evologicsSocket->waitForConnected())
        {
            qDebug() << "The following error occurred: " << _evologicsSocket->errorString();
        }

        if(_evologicsSocket->isOpen())
        {
            int x = qRound(_position.x());
            int y = qRound(_position.y());
            int z = qRound(_position.z());

            QString message = QString("position = %1 %2 %3\n").arg(x).arg(y).arg(z);
            QByteArray byteArray = message.toUtf8();

            _evologicsSocket->write(byteArray);
            _evologicsSocket->close();
        }
    }

    if(_udpPositionSocket)
    {
        QByteArray byteArray;

        QDataStream out (&byteArray,QIODevice::WriteOnly);
        out.setByteOrder(QDataStream::LittleEndian);

        out << _position.x();
        out << _position.y();
        out << _position.z();

        _udpPositionSocket->writeDatagram(byteArray, QHostAddress(_host), _port);
    }
}
