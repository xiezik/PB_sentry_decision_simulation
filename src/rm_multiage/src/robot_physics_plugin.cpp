#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_physics.hpp"  // 假设你的节点类定义在该头文件中

namespace rmMultistage {

class RobotPhysicsNode : public rclcpp::Node {
public:
    RobotPhysicsNode(const rclcpp::NodeOptions& options)
        : Node("robot_physics_node", options)
        , robotPhysics_(std::make_shared<RobotPhysics>(rclcpp::NodeOptions())) {
        // 创建一个定时器来调用 update 函数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotPhysicsNode::update, this));
    }

private:
    void update() {
        // 调用 robotPhysics_ 的成员函数
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<RobotPhysics> robotPhysics_;
};

}  // namespace rmMultistage

RCLCPP_COMPONENTS_REGISTER_NODE(rmMultistage::RobotPhysicsNode)