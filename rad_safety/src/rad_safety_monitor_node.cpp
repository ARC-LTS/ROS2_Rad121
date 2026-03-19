#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>

class RadSafetyMonitorNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    RadSafetyMonitorNode()
        : Node("rad_safety_monitor_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          is_retreating_(false)
    {
        // Parameters
        this->declare_parameter<double>("emergency_threshold_mrem_hr", 10.0);
        this->declare_parameter<double>("retreat_distance_m", 1.5);
        this->declare_parameter<std::string>("robot_base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");

        emergency_threshold_ = this->get_parameter("emergency_threshold_mrem_hr").as_double();
        retreat_distance_ = this->get_parameter("retreat_distance_m").as_double();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();

        // Subscriptions
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&RadSafetyMonitorNode::radCallback, this, std::placeholders::_1));

        // Action Client
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Radiation Safety Monitor has started.");
        RCLCPP_INFO(this->get_logger(), "Emergency threshold set to: %.2f mrem/hr", emergency_threshold_);
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            return;
        }

        double current_mR_hr = msg->data[1];
        if (current_mR_hr > emergency_threshold_) {
            triggerRetreat();
        }
    }

    void triggerRetreat()
    {
        if (is_retreating_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Retreat already in progress.");
            return;
        }

        RCLCPP_WARN(this->get_logger(), "EMERGENCY: Radiation threshold exceeded! Triggering retreat.");
        is_retreating_ = true;

        // Cancel any existing goal - preemptive goal sending is the simplest way
        // In a more complex system, we'd explicitly cancel the exploration goal.

        geometry_msgs::msg::PoseStamped current_pose;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
                map_frame_, robot_base_frame_, tf2::TimePointZero);
            
            current_pose.header = t.header;
            current_pose.pose.position.x = t.transform.translation.x;
            current_pose.pose.position.y = t.transform.translation.y;
            current_pose.pose.orientation = t.transform.rotation;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not get robot pose to calculate retreat goal: %s", ex.what());
            is_retreating_ = false;
            return;
        }

        // Calculate retreat goal `retreat_distance_` meters behind the robot
        tf2::Quaternion q;
        tf2::fromMsg(current_pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = map_frame_;
        goal_msg->pose.header.stamp = this->get_clock()->now();
        goal_msg->pose.pose.position.x = current_pose.pose.position.x - retreat_distance_ * cos(yaw);
        goal_msg->pose.pose.position.y = current_pose.pose.position.y - retreat_distance_ * sin(yaw);
        goal_msg->pose.pose.orientation = current_pose.pose.orientation;

        RCLCPP_INFO(this->get_logger(), "Sending retreat goal to (%.2f, %.2f)",
            goal_msg->pose.pose.position.x, goal_msg->pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&RadSafetyMonitorNode::retreatResultCallback, this, std::placeholders::_1);
        
        nav_action_client_->async_send_goal(*goal_msg, send_goal_options);
    }

    void retreatResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Retreat successful.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Retreat failed or was preempted.");
        }
        is_retreating_ = false;

        // Here you could add logic to notify the exploration to resume,
        // for example, by publishing to a topic.
    }

    // Parameters
    double emergency_threshold_;
    double retreat_distance_;
    std::string robot_base_frame_;
    std::string map_frame_;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State
    bool is_retreating_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadSafetyMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
