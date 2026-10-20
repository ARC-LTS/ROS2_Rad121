#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
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
        this->declare_parameter<double>("emergency_threshold_mrem_hr", 12.0);
        this->declare_parameter<double>("retreat_distance_m", 2.5);
        this->declare_parameter<std::string>("robot_base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<int>("dose_smoothing_window", 5);
        // Matches local_recovery_node's own triggerRecovery() cooldown -- see
        // triggerRetreat() for why this needs to exist at all.
        this->declare_parameter<double>("retreat_cooldown_s", 10.0);
        // How long to let local_recovery_node's own (more sophisticated,
        // escalating) escape search keep trying on its own before this node
        // steps in as a true last resort. See triggerRetreat().
        this->declare_parameter<double>("local_recovery_patience_s", 30.0);
        // Dose-budget-aware threshold relaxation. k=0 reproduces the original
        // fixed-threshold behavior exactly (the pilot's built-in ablation).
        // See radCallback()/effectiveThreshold() for the formula.
        this->declare_parameter<double>("dose_budget_relaxation_k", 0.0);
        this->declare_parameter<double>("dose_budget_relaxation_ceiling_mult", 2.0);
        this->declare_parameter<double>("max_radiation_dose_mrem", 100.0);

        emergency_threshold_ = this->get_parameter("emergency_threshold_mrem_hr").as_double();
        retreat_distance_ = this->get_parameter("retreat_distance_m").as_double();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        dose_smoothing_window_ = this->get_parameter("dose_smoothing_window").as_int();
        retreat_cooldown_s_ = this->get_parameter("retreat_cooldown_s").as_double();
        local_recovery_patience_s_ = this->get_parameter("local_recovery_patience_s").as_double();
        dose_budget_relaxation_k_ = this->get_parameter("dose_budget_relaxation_k").as_double();
        dose_budget_ceiling_mult_ = this->get_parameter("dose_budget_relaxation_ceiling_mult").as_double();
        max_radiation_dose_mrem_ = this->get_parameter("max_radiation_dose_mrem").as_double();

        // Subscriptions
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&RadSafetyMonitorNode::radCallback, this, std::placeholders::_1));

        rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
        costmap_qos.transient_local().reliable();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", costmap_qos,
            std::bind(&RadSafetyMonitorNode::costmapCallback, this, std::placeholders::_1));

        // Defer to local_recovery_node's own escape search while it's
        // actively working the same emergency -- see triggerRetreat().
        local_recovery_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/local_recovery_node/active", 10,
            std::bind(&RadSafetyMonitorNode::localRecoveryActiveCallback, this, std::placeholders::_1));

        // Symmetric to the above: lets local_recovery_node stand down while
        // this node currently owns an in-flight retreat goal, rather than
        // both nodes firing independently on their own cooldowns once this
        // node has taken over. Published on every state change, in
        // triggerRetreat() and retreatResultCallback().
        active_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/active", 10);

        // Cross-layer contract plumbing, mirroring local_recovery_node: use
        // the dose-budget authority's remaining fraction while fresh so all
        // layers scale their thresholds by the same multiplier by
        // construction; fall back to this node's own /rad integration
        // before the evaluator is up. The published multiplier feeds the
        // evaluator's per-trial contract-coherence check.
        budget_authority_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/dose_budget/remaining_fraction", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                authority_fraction_ = msg->data;
                authority_fraction_time_ = this->get_clock()->now();
            });
        multiplier_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "~/dose_budget_multiplier", 10);
        multiplier_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), [this]() {
                std_msgs::msg::Float64 m;
                m.data = currentBudgetMultiplier();
                multiplier_pub_->publish(m);
            });

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

        // Cumulative dose, integrated the same way rad_evaluator_node does
        // (dt in hours since dose is a rate in mrem/hr), so this node has its
        // own independent notion of how much of the mission dose budget
        // remains without depending on the evaluator. Needed for
        // effectiveThreshold() below.
        auto now = this->get_clock()->now();
        if (last_rad_time_.nanoseconds() > 0) {
            double dt = (now - last_rad_time_).seconds();
            cumulative_dose_mrem_ += msg->data[1] * (dt / 3600.0);
        }
        last_rad_time_ = now;

        dose_history_.push_back(msg->data[1]);
        if (static_cast<int>(dose_history_.size()) > dose_smoothing_window_) {
            dose_history_.pop_front();
        }

        double avg_dose = std::accumulate(dose_history_.begin(), dose_history_.end(), 0.0)
                          / static_cast<double>(dose_history_.size());

        if (avg_dose > effectiveThreshold()) {
            triggerRetreat();
        }
    }

    // Dose-budget-aware relaxation of the fixed emergency threshold: with
    // most of the mission's dose budget still unspent, tolerate a higher
    // instantaneous dose rate before retreating, since retreating forecloses
    // whatever unexplored area lies beyond a moderately elevated corridor for
    // the rest of the mission. Bounded by dose_budget_ceiling_mult_ so this
    // never loosens past a fixed multiple of the base threshold regardless of
    // how much budget remains -- k controls how much of that headroom is
    // actually used. k=0 (the default) reproduces the original fixed
    // threshold exactly.
    double effectiveThreshold()
    {
        return emergency_threshold_ * currentBudgetMultiplier();
    }

    // Shared relaxation multiplier, min(ceiling, 1 + k * f_remaining), from
    // the authority's fraction when fresh, else this node's own integrator.
    double currentBudgetMultiplier()
    {
        double remaining_fraction = 1.0;
        bool authority_fresh = authority_fraction_ >= 0.0 &&
            authority_fraction_time_.nanoseconds() > 0 &&
            (this->get_clock()->now() - authority_fraction_time_).seconds() < 5.0;
        if (authority_fresh) {
            remaining_fraction = authority_fraction_;
        } else if (max_radiation_dose_mrem_ > 0.0) {
            remaining_fraction = std::max(
                0.0, (max_radiation_dose_mrem_ - cumulative_dose_mrem_) / max_radiation_dose_mrem_);
        }
        return std::min(1.0 + dose_budget_relaxation_k_ * remaining_fraction,
                        dose_budget_ceiling_mult_);
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_costmap_ = msg;
    }

    void localRecoveryActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !local_recovery_active_) {
            // Just became active; start the patience clock from here, not
            // from whenever this node happens to next check it.
            local_recovery_active_since_ = this->get_clock()->now();
        }
        local_recovery_active_ = msg->data;
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

        // Defer to local_recovery_node's own, more sophisticated (utility-
        // scored, escalating-radius) escape search while it's actively
        // working the same emergency -- only step in as a true last resort
        // if it's been trying for longer than local_recovery_patience_s_
        // without resolving. The retreat_cooldown_s_ guard below reduces
        // *how often* this node can fire, but on its own does nothing to
        // establish which node should go first; this check is what actually
        // lets local_recovery_node's attempt run for a real, uninterrupted
        // window before this node ever becomes eligible to preempt it.
        if (local_recovery_active_) {
            double active_for = (this->get_clock()->now() - local_recovery_active_since_).seconds();
            if (active_for < local_recovery_patience_s_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "local_recovery_node is already handling this (active %.1fs of %.1fs patience); yielding.",
                    active_for, local_recovery_patience_s_);
                return;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "local_recovery_node has been active for %.1fs with no resolution; taking over as last resort.",
                active_for);
            // Restart local_recovery's patience clock at the moment this node
            // takes over, not when local_recovery_active_ first went true.
            // Without this, once patience first elapses, local_recovery_node
            // (which respects this node's own ~/active state symmetrically,
            // see local_recovery_node.cpp) only ever gets the sliver of time
            // between this node's own retreat attempts to try again before
            // being judged "already past patience" and preempted right back,
            // rather than the full uninterrupted window the patience
            // parameter is meant to guarantee it. This turns the interaction
            // into a fair alternation, local_recovery gets a full patience
            // window, then this node gets one bounded attempt, then
            // local_recovery gets another full window, and so on, rather
            // than a live-lock where the two nodes preempt each other
            // indefinitely on their own separate cooldowns (observed
            // directly: c05_rectangle_obst_r08_fix, 99 preemptions over a
            // single 1500s trial, despite the one-time patience check).
            local_recovery_active_since_ = this->get_clock()->now();
        }

        // Cooldown mirroring local_recovery_node's own 10s guard. Without
        // this, the two nodes were observed fighting: is_retreating_ resets
        // to false the instant this node's goal gets preempted by anyone
        // (including local_recovery_node's own independent retreat, which
        // fires on essentially the same acute-dose trigger), so the very
        // next radCallback tick immediately re-fires a competing goal that
        // preempts local_recovery_node's in-flight attempt right back --
        // observed directly in a live trial: "EMERGENCY... Triggering
        // retreat" / "Retreat failed or was preempted" alternating with
        // local_recovery's own goal every ~0.1-0.4s for the entire 900s
        // wall-clock cap, cumulative dose reaching 16,678 mrem (166x the
        // 100 mrem mission budget) because neither node's retreat attempt
        // ever got an uninterrupted window to actually complete. A shared
        // cooldown order-of-magnitude gives whichever node fires first a
        // real chance to finish before the other becomes eligible to
        // preempt it again.
        auto now_time = this->get_clock()->now();
        if ((now_time - last_retreat_time_).seconds() < retreat_cooldown_s_) {
            return;
        }
        last_retreat_time_ = now_time;

        RCLCPP_WARN(this->get_logger(), "EMERGENCY: Radiation threshold exceeded! Triggering retreat.");
        is_retreating_ = true;
        publishActive(true);

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
            publishActive(false);
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
                    publishActive(false);
                }
            };
        send_goal_options.result_callback =
            std::bind(&RadSafetyMonitorNode::retreatResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, send_goal_options);
    }

    void publishActive(bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        active_pub_->publish(msg);
    }

    void retreatResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Retreat successful.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Retreat failed or was preempted.");
        }
        is_retreating_ = false;
        publishActive(false);

        // Here you could add logic to notify the exploration to resume,
        // for example, by publishing to a topic.
    }

    // Parameters
    double emergency_threshold_;
    double retreat_distance_;
    std::string robot_base_frame_;
    std::string map_frame_;
    int dose_smoothing_window_;
    double retreat_cooldown_s_;
    double local_recovery_patience_s_;
    double dose_budget_relaxation_k_;
    double dose_budget_ceiling_mult_;
    double max_radiation_dose_mrem_;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr local_recovery_active_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State
    bool is_retreating_;
    rclcpp::Time last_retreat_time_{0, 0, RCL_ROS_TIME};
    bool local_recovery_active_ = false;
    rclcpp::Time local_recovery_active_since_{0, 0, RCL_ROS_TIME};
    std::deque<double> dose_history_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    double cumulative_dose_mrem_ = 0.0;
    rclcpp::Time last_rad_time_{0, 0, RCL_ROS_TIME};
    double authority_fraction_ = -1.0;
    rclcpp::Time authority_fraction_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr budget_authority_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr multiplier_pub_;
    rclcpp::TimerBase::SharedPtr multiplier_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadSafetyMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
