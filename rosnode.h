#ifndef ROSNODE_H
#define ROSNODE_H

#include <QObject>
#include <QDebug>

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

typedef struct{
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr propeller;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr verticalFins;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr horizontalFins;
}Pub_t;

class RosNode : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    explicit RosNode(QObject *parent = nullptr);
    ~RosNode();
    Q_INVOKABLE void propellerCallback(double value);
    Q_INVOKABLE void verticalFinsCallback(double value);
    Q_INVOKABLE void horizontalFinsCallback(double value);

private:
    Pub_t pub;
    void rosSpin();
    std::thread spinThread;

    bool _valuesChanged = true;

    std_msgs::msg::Float64 _propellerValue;
    std_msgs::msg::Float64 _verticalFinsValue;
    std_msgs::msg::Float64 _horizontalFinsValue;

    void publisherCallback();
    rclcpp::TimerBase::SharedPtr _timer;

};

#endif // ROSNODE_H
