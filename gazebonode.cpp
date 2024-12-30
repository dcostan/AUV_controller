#include "gazebonode.h"

GazeboNode::GazeboNode(QObject *parent)
    : QObject{parent}
    , _timer(new QTimer(this))
    , _evologicsSocket(nullptr)
    , _udpPositionSocket(nullptr)
{
    std::thread positionThread = std::thread(&GazeboNode::subscribe, this);
    positionThread.detach();

    connect(_timer, SIGNAL(timeout()), this, SLOT(sendPosition()));
    _timer->start(1000);
}

void GazeboNode::startUdpPosition()
{
    _udpPositionSocket = std::make_unique<QUdpSocket>(this);
}

void GazeboNode::startEvologicsPosition()
{
    _evologicsSocket = std::make_unique<QTcpSocket>(this);
}

void GazeboNode::stopUdpPosition()
{
    _udpPositionSocket.reset();
}

void GazeboNode::stopEvologicsPosition()
{
    _evologicsSocket.reset();
}

void GazeboNode::subscribe()
{
    if (!_node.Subscribe(_topic, &GazeboNode::positionCallback, this))
    {
        qDebug() << "Error subscribing to gazebo position topic";
    }

    gz::transport::waitForShutdown();
}

void GazeboNode::positionCallback(const gz::msgs::Pose_V &pose_v)
{
    _position = pose_v.pose(0).position();
}

void GazeboNode::sendPosition()
{
    if(_evologicsSocket)
    {
        _evologicsSocket->connectToHost(QHostAddress("10.42.9.1"), 4242);

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

            QDataStream socketStream(_evologicsSocket.get());
            socketStream << byteArray;

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

        _udpPositionSocket->writeDatagram(byteArray, QHostAddress::LocalHost, 7000);
    }
}
