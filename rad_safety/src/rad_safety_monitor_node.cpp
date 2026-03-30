#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>
#include <numeric>

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
        this->declare_parameter<int>("dose_smoothing_window", 5);

        emergency_threshold_ = this->get_parameter("emergency_threshold_mrem_hr").as_double();
        retreat_distance_ = this->get_parameter("retreat_distance_m").as_double();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        dose_smoothing_window_ = this->get_parameter("dose_smoothing_window").as_int();

        // Subscriptions
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&RadSafetyMonitorNode::radCallback, this, std::placeholders::_1));

        rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
        costmap_qos.transient_local().reliable();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", costmap_qos,
            std::bind(&RadSafetyMonitorNode::costmapCallback, this, std::placeholders::_1));

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

        dose_history_.push_back(msg->data[1]);
        if (static_cast<int>(dose_history_.size()) > dose_smoothing_window_) {
            dose_history_.pop_front();
        }

        double avg_dose = std::accumulate(dose_history_.begin(), dose_history_.end(), 0.0)
                          / static_cast<double>(dose_history_.size());

        if (avg_dose > emergency_threshold_) {
            triggerRetreat();
        }
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_costmap_ = msg;
    }

    // Scan candidate retreat points along the backward direction and angular offsets,
    // then at reduced distances, to find the nearest free cell in the costmap.
    // Returns true if a navigable point was found; always populates goal_x/goal_y.
    bool findFreeRetreatPoint(double robot_x, double robot_y, double retreat_yaw,
                               double &goal_x, double &goal_y)
    {
        // Angular offsets to try around the primary retreat direction (±15°, ±30°)
        const double angle_offsets[] = {0.0, M_PI / 12.0, -M_PI / 12.0, M_PI / 6.0, -M_PI / 6.0};
        // Distance fractions to try (full, 75%, 50%)
        const double dist_fracs[] = {1.0, 0.75, 0.5};

        if (latest_costmap_) {
            const auto &cm = *latest_costmap_;
            float res = cm.info.resolution;
            float ox = cm.info.origin.position.x;
            float oy = cm.info.origin.position.y;
            int w = static_cast<int>(cm.info.width);
            int h = static_cast<int>(cm.info.height);

            auto isFree = [&](double x, double y) -> bool {
                int px = static_cast<int>((x - ox) / res);
                int py = static_cast<int>((y - oy) / res);
                if (px < 2 || px >= w - 2 || py < 2 || py >= h - 2) return false;
                return cm.data[py * w + px] >= 0 && cm.data[py * w + px] < 70;
            };

            for (double frac : dist_fracs) {
                double dist = retreat_distance_ * frac;
                for (double da : angle_offsets) {
                    double tx = robot_x + dist * cos(retreat_yaw + da);
                    double ty = robot_y + dist * sin(retreat_yaw + da);
                    if (isFree(tx, ty)) {
                        goal_x = tx;
                        goal_y = ty;
                        return true;
                    }
                }
            }
        }

        // No costmap or no free cell found: use raw backward point as fallback
        goal_x = robot_x + retreat_distance_ * cos(retreat_yaw);
        goal_y = robot_y + retreat_distance_ * sin(retreat_yaw);
        return false;
    }

    void triggerRetreat()
    {
        if (is_retreating_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Retreat already in progress.");
            return;
        }

        RCLCPP_WARN(this->get_logger(), "EMERGENCY: Radiation threshold exceeded! Triggering retreat.");
        is_retreating_ = true;

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

        tf2::Quaternion q;
        tf2::fromMsg(current_pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Retreat direction is directly behind the robot
        double retreat_yaw = yaw + M_PI;

        double goal_x, goal_y;
        bool free = findFreeRetreatPoint(
            current_pose.pose.position.x, current_pose.pose.position.y,
            retreat_yaw, goal_x, goal_y);
        if (!free) {
            RCLCPP_WARN(this->get_logger(), "No free retreat cell found; using best-effort fallback point.");
        }

        // Orient toward the chosen goal point so the robot faces its destination.
        // findFreeRetreatPoint may have picked a point at an angular offset from
        // retreat_yaw, so recompute yaw from current position to the actual goal.
        double goal_yaw = std::atan2(goal_y - current_pose.pose.position.y,
                                     goal_x - current_pose.pose.position.x);
        tf2::Quaternion retreat_q;
        retreat_q.setRPY(0.0, 0.0, goal_yaw);

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = map_frame_;
        goal_msg->pose.header.stamp = this->get_clock()->now();
        goal_msg->pose.pose.position.x = goal_x;
        goal_msg->pose.pose.position.y = goal_y;
        goal_msg->pose.pose.orientation = tf2::toMsg(retreat_q);

        RCLCPP_INFO(this->get_logger(), "Sending retreat goal to (%.2f, %.2f), facing yaw=%.2f rad",
            goal_x, goal_y, goal_yaw);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr &handle) {
                if (!handle) {
                    RCLCPP_ERROR(this->get_logger(), "Retreat goal rejected by Nav2.");
                    is_retreating_ = false;
                }
            };
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
    int dose_smoothing_window_;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State
    bool is_retreating_;
    std::deque<double> dose_history_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadSafetyMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
