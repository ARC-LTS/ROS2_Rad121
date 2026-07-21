#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <std_msgs/msg/float64.hpp>
#include <optional>
#include <cmath>

class StuckEscapeNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using Spin = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

    StuckEscapeNode() : Node("stuck_escape_node")
    {
        this->declare_parameter<double>("stuck_timeout_s", 12.0);
        this->declare_parameter<double>("escape_duration_s", 2.5);
        this->declare_parameter<double>("escape_speed_m_s", 0.10);
        this->declare_parameter<double>("min_movement_m", 0.1);
        this->declare_parameter<double>("min_rotation_rad", 0.08);
        this->declare_parameter<std::string>("odom_topic", "/odometry");
        this->declare_parameter<double>("spin_target_yaw_rad", 3.14159);
        this->declare_parameter<double>("spin_timeout_s", 8.0);

        stuck_timeout_   = this->get_parameter("stuck_timeout_s").as_double();
        escape_duration_ = this->get_parameter("escape_duration_s").as_double();
        escape_speed_    = this->get_parameter("escape_speed_m_s").as_double();
        min_movement_    = this->get_parameter("min_movement_m").as_double();
        min_rotation_    = this->get_parameter("min_rotation_rad").as_double();
        spin_target_yaw_ = this->get_parameter("spin_target_yaw_rad").as_double();
        spin_timeout_    = this->get_parameter("spin_timeout_s").as_double();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_x_ = msg->pose.pose.position.x;
                current_y_ = msg->pose.pose.position.y;
                current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
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
        // One message per completed stuck-and-recover cycle, payload is the
        // wall-clock duration from stuck-detected to escape-finished. Lets
        // rad_evaluator_node count how often the robot got physically wedged
        // and how long each recovery took, independent of the radiation-
        // triggered recovery metrics local_recovery_node already feeds it.
        stuck_event_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/stuck_escape/event", 10);

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");

        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&StuckEscapeNode::monitorTick, this));

        RCLCPP_INFO(this->get_logger(),
            "Stuck escape node started. Timeout: %.1fs, escape: %.2fm/s for %.1fs, then spin %.2f rad.",
            stuck_timeout_, escape_speed_, escape_duration_, spin_target_yaw_);
    }

private:
    // Manual quaternion->yaw (avoids pulling in tf2_geometry_msgs just for this).
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    static double angleDiff(double a, double b)
    {
        double d = a - b;
        while (d > M_PI) d -= 2.0 * M_PI;
        while (d < -M_PI) d += 2.0 * M_PI;
        return d;
    }

    void monitorTick()
    {
        if (is_escaping_) return;

        if (!nav_executing_) {
            // Robot not navigating — reset baseline for next session
            last_moved_x_ = current_x_;
            last_moved_y_ = current_y_;
            last_moved_yaw_ = current_yaw_;
            stuck_start_time_.reset();
            return;
        }

        double dx = current_x_ - last_moved_x_;
        double dy = current_y_ - last_moved_y_;
        double dyaw = std::abs(angleDiff(current_yaw_, last_moved_yaw_));
        // A robot doing a legitimate final in-place rotation to satisfy Nav2's
        // yaw_goal_tolerance (RotateToGoal critic) can sit at a near-fixed
        // (x,y) for many seconds while still making real progress -- observed
        // directly: returnToInitialPose() retries fighting this exact
        // watchdog in a loop, cancelling the goal every ~12s just as
        // RotateToGoal was converging, backing up, and spinning a full 180
        // degrees away from the target heading, resetting progress each time
        // until the retry budget ran out. Position-only stuck detection has
        // no way to distinguish that from genuinely wedged.
        if (std::sqrt(dx*dx + dy*dy) > min_movement_ || dyaw > min_rotation_) {
            last_moved_x_ = current_x_;
            last_moved_y_ = current_y_;
            last_moved_yaw_ = current_yaw_;
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
        escape_began_time_ = this->now();
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
                        // Backing straight up doesn't change orientation -- if the
                        // robot was wedged facing the wrong way, it can end up back
                        // in the same trap facing the same direction (observed
                        // directly: two stuck-triggers ~16s apart at nearly the same
                        // position). Follow up with Nav2's own Spin recovery
                        // (costmap-aware, unlike a raw cmd_vel turn) now that
                        // backing up has created more clearance to safely rotate in.
                        triggerSpin();
                    }
                }
            });
    }

    void triggerSpin()
    {
        if (!spin_client_->action_server_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Spin action server not ready; skipping reorientation.");
            finishEscape();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Escape: spinning %.2f rad to reorient.", spin_target_yaw_);
        auto goal = Spin::Goal();
        goal.target_yaw = spin_target_yaw_;

        // Safety net: if the spin result callback never fires -- e.g. the
        // action server never resolves because it's being starved by another
        // node's competing navigate_to_pose activity, as observed directly
        // (local_recovery_node mid-retry-loop at the same time) -- this node
        // would otherwise stay is_escaping_==true forever, permanently
        // disabling its own watchdog for the rest of the trial. Force
        // completion after spin_timeout_s regardless of whether a result
        // ever arrives.
        spin_result_received_ = false;
        spin_timeout_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(spin_timeout_),
            [this]() {
                spin_timeout_timer_->cancel();
                if (spin_result_received_) return;
                RCLCPP_WARN(this->get_logger(),
                    "Spin result never arrived after %.1fs; forcing escape completion anyway.",
                    spin_timeout_);
                if (spin_goal_handle_) {
                    spin_client_->async_cancel_goal(spin_goal_handle_);
                }
                finishEscape();
            });

        auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
        options.goal_response_callback =
            [this](const GoalHandleSpin::SharedPtr & handle) {
                spin_goal_handle_ = handle;
            };
        options.result_callback =
            [this](const GoalHandleSpin::WrappedResult & result) {
                if (spin_result_received_) return;  // timeout already handled it
                spin_result_received_ = true;
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_WARN(this->get_logger(),
                        "Spin reorientation did not complete cleanly (code %d); continuing anyway.",
                        static_cast<int>(result.code));
                }
                finishEscape();
            };
        spin_client_->async_send_goal(goal, options);
    }

    void finishEscape()
    {
        spin_result_received_ = true;
        if (spin_timeout_timer_) {
            spin_timeout_timer_->cancel();
            spin_timeout_timer_.reset();
        }
        is_escaping_ = false;
        last_moved_x_ = current_x_;
        last_moved_y_ = current_y_;
        last_moved_yaw_ = current_yaw_;

        double recovery_s = (this->now() - escape_began_time_).seconds();
        std_msgs::msg::Float64 event_msg;
        event_msg.data = recovery_s;
        stuck_event_pub_->publish(event_msg);

        RCLCPP_INFO(this->get_logger(),
            "Escape complete after %.1fs. Explorer will select a new goal.", recovery_s);
    }

    enum class EscapePhase { DELAY, DRIVING };

    // Parameters
    double stuck_timeout_;
    double escape_duration_;
    double escape_speed_;
    double min_movement_;
    double min_rotation_;
    double spin_target_yaw_;
    double spin_timeout_;

    // State
    double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
    double last_moved_x_{0.0}, last_moved_y_{0.0}, last_moved_yaw_{0.0};
    bool nav_executing_{false};
    bool is_escaping_{false};
    int escape_elapsed_ms_{0};
    EscapePhase escape_phase_{EscapePhase::DELAY};
    std::optional<rclcpp::Time> stuck_start_time_;
    rclcpp::Time escape_began_time_;
    bool spin_result_received_{false};
    GoalHandleSpin::SharedPtr spin_goal_handle_;
    rclcpp::TimerBase::SharedPtr spin_timeout_timer_;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stuck_event_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<Spin>::SharedPtr spin_client_;
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
