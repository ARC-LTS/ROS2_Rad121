#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <optional>
#include <cmath>

class StuckEscapeNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    StuckEscapeNode() : Node("stuck_escape_node")
    {
        this->declare_parameter<double>("stuck_timeout_s", 12.0);
        this->declare_parameter<double>("escape_duration_s", 2.5);
        this->declare_parameter<double>("escape_speed_m_s", 0.10);
        this->declare_parameter<double>("min_movement_m", 0.1);
        this->declare_parameter<std::string>("odom_topic", "/odometry");

        stuck_timeout_  = this->get_parameter("stuck_timeout_s").as_double();
        escape_duration_= this->get_parameter("escape_duration_s").as_double();
        escape_speed_   = this->get_parameter("escape_speed_m_s").as_double();
        min_movement_   = this->get_parameter("min_movement_m").as_double();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_x_ = msg->pose.pose.position.x;
                current_y_ = msg->pose.pose.position.y;
            });

        // Track whether a navigate_to_pose goal is currently executing
        nav_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                nav_executing_ = false;
                for (const auto & s : msg->status_list) {
                    if (s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                        nav_executing_ = true;
                        break;
                    }
                }
            });

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&StuckEscapeNode::monitorTick, this));

        RCLCPP_INFO(this->get_logger(),
            "Stuck escape node started. Timeout: %.1fs, escape: %.2fm/s for %.1fs.",
            stuck_timeout_, escape_speed_, escape_duration_);
    }

private:
    void monitorTick()
    {
        if (is_escaping_) return;

        if (!nav_executing_) {
            // Robot not navigating — reset position baseline for next session
            last_moved_x_ = current_x_;
            last_moved_y_ = current_y_;
            stuck_start_time_.reset();
            return;
        }

        double dx = current_x_ - last_moved_x_;
        double dy = current_y_ - last_moved_y_;
        if (std::sqrt(dx*dx + dy*dy) > min_movement_) {
            last_moved_x_ = current_x_;
            last_moved_y_ = current_y_;
            stuck_start_time_.reset();
            return;
        }

        if (!stuck_start_time_) {
            stuck_start_time_ = this->now();
            return;
        }

        double stuck_s = (this->now() - *stuck_start_time_).seconds();
        if (stuck_s >= stuck_timeout_) {
            RCLCPP_WARN(this->get_logger(),
                "Robot stuck for %.1fs at (%.2f, %.2f). Canceling goal and driving backward.",
                stuck_s, current_x_, current_y_);
            triggerEscape();
        }
    }

    void triggerEscape()
    {
        is_escaping_ = true;
        stuck_start_time_.reset();

        // Cancel the active navigation goal so Nav2 releases cmd_vel ownership
        if (nav_client_->action_server_is_ready()) {
            nav_client_->async_cancel_all_goals();
        }

        // Give Nav2 ~400 ms to process the cancel and stop publishing cmd_vel,
        // then drive backward for escape_duration_ seconds.
        escape_elapsed_ms_ = 0;
        escape_phase_ = EscapePhase::DELAY;

        escape_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                escape_elapsed_ms_ += 50;

                if (escape_phase_ == EscapePhase::DELAY) {
                    if (escape_elapsed_ms_ >= 400) {
                        escape_phase_ = EscapePhase::DRIVING;
                        escape_elapsed_ms_ = 0;
                        RCLCPP_INFO(this->get_logger(), "Escape: driving backward.");
                    }
                    return;
                }

                if (escape_phase_ == EscapePhase::DRIVING) {
                    if (escape_elapsed_ms_ < static_cast<int>(escape_duration_ * 1000.0)) {
                        geometry_msgs::msg::Twist twist;
                        twist.linear.x = -escape_speed_;
                        cmd_vel_pub_->publish(twist);
                    } else {
                        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // stop
                        escape_timer_->cancel();
                        escape_timer_.reset();
                        is_escaping_ = false;
                        last_moved_x_ = current_x_;
                        last_moved_y_ = current_y_;
                        RCLCPP_INFO(this->get_logger(),
                            "Escape complete. Explorer will select a new goal.");
                    }
                }
            });
    }

    enum class EscapePhase { DELAY, DRIVING };

    // Parameters
    double stuck_timeout_;
    double escape_duration_;
    double escape_speed_;
    double min_movement_;

    // State
    double current_x_{0.0}, current_y_{0.0};
    double last_moved_x_{0.0}, last_moved_y_{0.0};
    bool nav_executing_{false};
    bool is_escaping_{false};
    int escape_elapsed_ms_{0};
    EscapePhase escape_phase_{EscapePhase::DELAY};
    std::optional<rclcpp::Time> stuck_start_time_;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr escape_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StuckEscapeNode>());
    rclcpp::shutdown();
    return 0;
}
