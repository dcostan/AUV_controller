#include "rosnode.h"

RosNode::RosNode(QObject *parent)
    : QObject{parent}
    , Node{"auvController"}
{
    spinThread = std::thread{ std::bind(&RosNode::rosSpin, this) };

    pub.propeller = this->create_publisher<std_msgs::msg::Float64>("propeller_joint/cmd_thrust", 10);
    pub.verticalFins = this->create_publisher<std_msgs::msg::Float64>("vertical_fins_joint/cmd_pos", 10);
    pub.horizontalFins = this->create_publisher<std_msgs::msg::Float64>("horizontal_fins_joint/cmd_pos", 10);

    _timer = this->create_wall_timer(200ms, std::bind(&RosNode::publisherCallback, this));
}

RosNode::~RosNode()
{
    rclcpp::shutdown();
}

void RosNode::rosSpin()
{
    rclcpp::spin(this->get_node_base_interface());
    rclcpp::shutdown();
}

void RosNode::propellerCallback(double value)
{
    _valuesChanged = true;
    _propellerValue.data = value;
}

void RosNode::verticalFinsCallback(double value)
{
    _valuesChanged = true;
    _verticalFinsValue.data = value;
}

void RosNode::horizontalFinsCallback(double value)
{
    _valuesChanged = true;
    _horizontalFinsValue.data = value;
}

void RosNode::publisherCallback(){
    if (_valuesChanged)
    {
        _valuesChanged = false;

        pub.propeller->publish(_propellerValue);
        pub.verticalFins->publish(_verticalFinsValue);
        pub.horizontalFins->publish(_horizontalFinsValue);
    }
}
