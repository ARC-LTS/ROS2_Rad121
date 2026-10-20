#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <mutex>
#include <deque>
#include <numeric>

class LocalRecoveryNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    LocalRecoveryNode()
        : Node("local_recovery_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          is_recovering_(false),
          exploration_finished_(false),
          consecutive_recovery_failures_(0)
    {
        // Parameters
        this->declare_parameter<int>("dose_confirmation_count", 3);
        this->declare_parameter<double>("acute_dose_rate_mrem_hr", 10.0);
        // Aligned with params/local_recovery.yaml (was 0.25 here against 1.0
        // there). The YAML wins wherever it is loaded, so deployment was
        // unaffected, but any launch path that omits it silently produced a
        // materially different robot -- a four-fold tighter resume threshold.
        this->declare_parameter<double>("safe_dose_threshold_mrem_hr", 1.0);
        this->declare_parameter<double>("recovery_search_distance_m", 15.0); // Increased radius
        this->declare_parameter<std::string>("robot_base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("fallback_retreat_distance_m", 3.0);  // matches YAML
        this->declare_parameter<bool>("return_to_origin", false);  // matches YAML
        this->declare_parameter<double>("origin_x", 0.0);
        this->declare_parameter<double>("origin_y", 0.0);
        this->declare_parameter<std::string>("stop_explore_service", "/explore_node/stop");
        this->declare_parameter<std::string>("start_explore_service", "/explore_node/start");
        // Dose-budget-aware relaxation of acute_dose_rate_ -- mirrors
        // rad_safety_monitor_node's own effectiveThreshold(). k=0 reproduces
        // the fixed threshold exactly.
        // Hard ceiling on how many distinct recovery episodes one mission may
        // run. consecutive_recovery_failures_ resets to zero on every
        // successful resume, so it bounds a losing streak but not the total:
        // a robot oscillating across the resume threshold can cycle
        // recover -> resume -> recover indefinitely on the cooldown interval
        // and consume the entire mission clock without ever failing four
        // times in a row. 0 (default) preserves the original unbounded
        // behavior; when positive, exploration is left running once the cap
        // is hit rather than being stopped again, so the mission ends on its
        // own terms instead of on the wall clock.
        this->declare_parameter<int>("max_recovery_episodes", 0);
        this->declare_parameter<double>("dose_budget_relaxation_k", 0.0);
        this->declare_parameter<double>("dose_budget_relaxation_ceiling_mult", 2.0);
        this->declare_parameter<double>("max_radiation_dose_mrem", 100.0);

        dose_confirmation_count_ = this->get_parameter("dose_confirmation_count").as_int();
        acute_dose_rate_ = this->get_parameter("acute_dose_rate_mrem_hr").as_double();
        safe_dose_threshold_ = this->get_parameter("safe_dose_threshold_mrem_hr").as_double();
        max_recovery_episodes_ = static_cast<int>(this->get_parameter("max_recovery_episodes").as_int());
        recovery_search_distance_ = this->get_parameter("recovery_search_distance_m").as_double();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        fallback_retreat_distance_ = this->get_parameter("fallback_retreat_distance_m").as_double();
        return_to_origin_ = this->get_parameter("return_to_origin").as_bool();
        origin_x_ = this->get_parameter("origin_x").as_double();
        origin_y_ = this->get_parameter("origin_y").as_double();
        stop_explore_srv_name_ = this->get_parameter("stop_explore_service").as_string();
        start_explore_srv_name_ = this->get_parameter("start_explore_service").as_string();
        dose_budget_relaxation_k_ = this->get_parameter("dose_budget_relaxation_k").as_double();
        dose_budget_ceiling_mult_ = this->get_parameter("dose_budget_relaxation_ceiling_mult").as_double();
        max_radiation_dose_mrem_ = this->get_parameter("max_radiation_dose_mrem").as_double();

        // Subscriptions
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&LocalRecoveryNode::radCallback, this, std::placeholders::_1));
        
        rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
        costmap_qos.transient_local().reliable();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", costmap_qos, std::bind(&LocalRecoveryNode::costmapCallback, this, std::placeholders::_1));
        
        heatmap_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rad_heatmap/gp_mean_map", 10, std::bind(&LocalRecoveryNode::heatmapCallback, this, std::placeholders::_1));

        explore_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "explore/is_finished", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->exploration_finished_ = msg->data;
                if (this->exploration_finished_) {
                    RCLCPP_INFO(this->get_logger(), "Exploration finished detected. Disabling local recovery logic.");
                }
            });

        // Symmetric to rad_safety_monitor_node's own subscription to this
        // node's ~/active: stand down while rad_safety_monitor_node
        // currently owns an in-flight retreat goal, rather than both nodes
        // firing independently once it has taken over past its patience
        // window. See triggerRecovery() and recoveryResultCallback().
        rad_safety_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/rad_safety_monitor_node/active", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->rad_safety_active_ = msg->data;
            });

        // Action Client
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Service Clients
        stop_explore_client_ = this->create_client<std_srvs::srv::Trigger>(stop_explore_srv_name_);
        start_explore_client_ = this->create_client<std_srvs::srv::Trigger>(start_explore_srv_name_);

        // Physical escape publisher (direct cmd_vel when stuck in lethal space)
    //    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Lets rad_safety_monitor_node defer to this node's own (more
        // sophisticated) escape search instead of firing a competing
        // NavigateToPose goal on every acute reading -- see that node's
        // triggerRetreat() for the consumer side. Published every
        // recoveryTick() (2s) rather than only on state transitions, so a
        // late subscriber always gets a fresh value within one tick.
        active_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/active", 10);

        // Cross-layer contract plumbing. The dose-budget authority
        // (rad_evaluator_node, the same integrator whose cumulative dose is
        // the campaign's reported ground truth) publishes the remaining
        // budget fraction; while that value is fresh this node uses it in
        // place of its own independent /rad integration, so every layer's
        // effective threshold scales by the same multiplier by construction
        // rather than by the approximate agreement of four separate
        // integrators. The internal integrator is kept as a fallback for
        // the window before the evaluator is up.
        budget_authority_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/dose_budget/remaining_fraction", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                authority_fraction_ = msg->data;
                authority_fraction_time_ = this->get_clock()->now();
            });
        // The multiplier actually in use, published for the evaluator's
        // runtime contract-coherence check (max divergence / mis-ordering
        // across layers, reported per trial).
        multiplier_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "~/dose_budget_multiplier", 10);
        multiplier_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), [this]() {
                std_msgs::msg::Float64 m;
                m.data = currentBudgetMultiplier();
                multiplier_pub_->publish(m);
            });

        // Dynamic recovery timer
        recovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&LocalRecoveryNode::recoveryTick, this));

        RCLCPP_INFO(this->get_logger(), "Local Recovery Node overhaul complete. Enhanced search radius: %.1fm", recovery_search_distance_);
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;

        // Cumulative dose, integrated the same way rad_evaluator_node and
        // rad_safety_monitor_node both do (dt in hours). Needed for
        // effectiveAcuteThreshold() below.
        auto now = this->get_clock()->now();
        if (last_rad_time_.nanoseconds() > 0) {
            double dt = (now - last_rad_time_).seconds();
            cumulative_dose_mrem_ += msg->data[1] * (dt / 3600.0);
        }
        last_rad_time_ = now;

        dose_history_.push_back(msg->data[1]);
        if (static_cast<int>(dose_history_.size()) > dose_confirmation_count_) {
            dose_history_.pop_front();
        }
        // Rolling average — smooths transient spikes before recovery is considered
        current_rad_val_ = std::accumulate(dose_history_.begin(), dose_history_.end(), 0.0)
                           / static_cast<double>(dose_history_.size());
    }

    // The shared relaxation multiplier, min(ceiling, 1 + k * f_remaining).
    // Prefers the dose-budget authority's remaining fraction when fresh
    // (received within the last 5s); falls back to this node's own /rad
    // integration otherwise.
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

    // Mirrors rad_safety_monitor_node's own effectiveThreshold(): tolerate a
    // higher instantaneous dose rate before triggering recovery while most of
    // the mission's dose budget is still unspent. k=0 (default) reproduces
    // the fixed acute_dose_rate_ exactly.
    double effectiveAcuteThreshold()
    {
        return acute_dose_rate_ * currentBudgetMultiplier();
    }

    // The resume threshold scales by the same multiplier as the acute
    // trigger so the entry/exit hysteresis ratio stays invariant across k;
    // a fixed exit under a relaxed entry would make recovery dwell time a
    // hidden function of remaining budget.
    double effectiveSafeThreshold()
    {
        return safe_dose_threshold_ * currentBudgetMultiplier();
    }

    void recoveryTick()
    {
        std_msgs::msg::Bool active_msg;
        active_msg.data = is_recovering_;
        active_pub_->publish(active_msg);

        if (exploration_finished_) return;

        // Self-heal: if we're marked as recovering but the robot's current
        // dose has already dropped to a genuinely safe level, resume
        // immediately rather than waiting on a specific navigate_to_pose goal
        // to resolve. Without this, once dose drops below acute_dose_rate_
        // (so the check below stops re-firing triggerRecovery()) while
        // is_recovering_ is still true -- e.g. the in-flight goal resolves
        // via a path that doesn't hit the SUCCEEDED branch, or simply hasn't
        // resolved yet -- the state machine has no way to progress at all:
        // it's not above acute (so won't retry/escalate to the 4-failure
        // origin fallback) and never got the SUCCEEDED callback (so
        // is_recovering_ never clears). Exploration stays stopped forever.
        // Observed directly: dose sat at 2.5-4 mrem/hr (above safe, below
        // acute) with no active navigate_to_pose goal and no further log
        // activity for 5+ minutes, requiring manual /explore_node/start.
        if (is_recovering_ && current_rad_val_ < effectiveSafeThreshold()) {
            RCLCPP_INFO(this->get_logger(),
                "Dose (%.2f) now below safe threshold (%.2f). Resuming exploration.",
                current_rad_val_, effectiveSafeThreshold());
            nav_action_client_->async_cancel_all_goals();
            is_recovering_ = false;
            consecutive_recovery_failures_ = 0;
            failed_escape_points_.clear();
            callTriggerService(start_explore_client_, "Start Exploration");
            return;
        }

        // Keep actively retrying for as long as we're either above the acute
        // threshold (a fresh emergency) OR already mid-recovery and not yet
        // confirmed safe -- otherwise there's a dead zone between
        // safe_dose_threshold_ and acute_dose_rate_ (exactly the 2.5-4
        // mrem/hr observed above) where recovery would neither resolve nor
        // keep progressing.
        if (current_rad_val_ > effectiveAcuteThreshold() || is_recovering_) {
            triggerRecovery();
        }
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_costmap_ = msg;
    }

    void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(heatmap_mutex_);
        latest_heatmap_ = msg;
    }

    void triggerRecovery()
    {
        // Stand down entirely while rad_safety_monitor_node currently owns
        // an in-flight retreat goal, rather than firing a competing one that
        // just preempts it right back. This is the other half of the mutual
        // exclusion; see that node's triggerRetreat() for the corresponding
        // check in the other direction, and recoveryResultCallback() below
        // for how a failure that happens anyway (a narrow race between this
        // check and rad_safety_monitor_node taking over) is not blacklisted.
        if (rad_safety_active_) {
            return;
        }

        if (max_recovery_episodes_ > 0 && !is_recovering_ &&
            recovery_episodes_ >= max_recovery_episodes_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                "Recovery episode cap (%d) reached; leaving exploration running and "
                "deferring to the dose-budget and emergency-brake layers.",
                max_recovery_episodes_);
            return;
        }

        auto now = this->get_clock()->now();

        // Don't preempt an active recovery within the cooldown window
        if (is_recovering_ && (now - last_goal_time_).seconds() < 10.0) {
            return;
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Critical radiation (%.2f)! Executing recovery (attempt #%d).",
            current_rad_val_, consecutive_recovery_failures_ + 1);

        if (!is_recovering_) {
            callTriggerService(stop_explore_client_, "Stop Exploration");
        }

        geometry_msgs::msg::PoseStamped current_pose;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
                map_frame_, robot_base_frame_, tf2::TimePointZero);
            current_pose.header = t.header;
            current_pose.pose.position.x = t.transform.translation.x;
            current_pose.pose.position.y = t.transform.translation.y;
            current_pose.pose.orientation = t.transform.rotation;
        } catch (...) { return; }

        geometry_msgs::msg::Point escape_point;
        bool point_found = false;

        // After 4 consecutive failures, try the map origin as a guaranteed
        // destination -- but only if it hasn't already failed itself.
        // Without this check, once a goal to the origin fails once, this
        // branch retries the exact same doomed point forever with no
        // escalation (observed directly: stuck retrying (0,0) from a corner
        // for 10+ consecutive attempts, each failing instantly). If origin
        // is already a known dead end, fall through to the normal
        // blacklist-respecting search instead.
        bool origin_already_failed = false;
        for (const auto& fe : failed_escape_points_) {
            double dx = origin_x_ - fe.x, dy = origin_y_ - fe.y;
            if (dx * dx + dy * dy < 4.0) { origin_already_failed = true; break; }
        }
        if (consecutive_recovery_failures_ >= 4 && !origin_already_failed) {
            RCLCPP_WARN(this->get_logger(),
                "4+ recovery failures. Forcing navigation to map origin as last resort.");
            escape_point.x = origin_x_;
            escape_point.y = origin_y_;
            escape_point.z = 0.0;
            point_found = true;
        } else if (return_to_origin_) {
            escape_point.x = origin_x_;
            escape_point.y = origin_y_;
            escape_point.z = 0.0;
            std::lock_guard<std::mutex> map_lock(map_mutex_);
            if (latest_costmap_) {
                float res = latest_costmap_->info.resolution;
                float ox = latest_costmap_->info.origin.position.x;
                float oy = latest_costmap_->info.origin.position.y;
                int w = latest_costmap_->info.width;
                int h = latest_costmap_->info.height;
                int px = (escape_point.x - ox) / res;
                int py = (escape_point.y - oy) / res;
                if (px < 2 || px >= w - 2 || py < 2 || py >= h - 2) {
                    RCLCPP_WARN(this->get_logger(), "Origin out of map bounds. Falling back to safe point search.");
                    return_to_origin_ = false;
                    // Expand radius after failures to break out of local minima
                    double search_radius = recovery_search_distance_ *
                        (consecutive_recovery_failures_ >= 2 ? 1.5 : 1.0);
                    point_found = findSafePoint(current_pose.pose.position, escape_point, search_radius);
                } else {
                    point_found = true;
                    RCLCPP_INFO(this->get_logger(), "Evacuating to origin (%.2f, %.2f)", origin_x_, origin_y_);
                }
            }
        } else {
            double search_radius = recovery_search_distance_ *
                (consecutive_recovery_failures_ >= 2 ? 1.5 : 1.0);
            if (consecutive_recovery_failures_ >= 2) {
                RCLCPP_INFO(this->get_logger(),
                    "Expanding search radius to %.1fm after %d failures.",
                    search_radius, consecutive_recovery_failures_);
            }
            point_found = findSafePoint(current_pose.pose.position, escape_point, search_radius);
        }

        if (!point_found) {
            // Improved fallback: sample 8 directions and pick the one with the
            // lowest costmap cost that hasn't already been tried.
            tf2::Quaternion q;
            tf2::fromMsg(current_pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            float best_cost = std::numeric_limits<float>::max();
            bool directional_found = false;
            {
                std::lock_guard<std::mutex> map_lock(map_mutex_);
                if (latest_costmap_) {
                    float res = latest_costmap_->info.resolution;
                    float ox = latest_costmap_->info.origin.position.x;
                    float oy = latest_costmap_->info.origin.position.y;
                    int w = latest_costmap_->info.width;
                    int h = latest_costmap_->info.height;
                    for (int i = 0; i < 8; ++i) {
                        double angle = yaw + i * M_PI / 4.0;
                        geometry_msgs::msg::Point candidate;
                        candidate.x = current_pose.pose.position.x + fallback_retreat_distance_ * cos(angle);
                        candidate.y = current_pose.pose.position.y + fallback_retreat_distance_ * sin(angle);
                        int px = (candidate.x - ox) / res;
                        int py = (candidate.y - oy) / res;
                        if (px < 2 || px >= w - 2 || py < 2 || py >= h - 2) continue;
                        unsigned char cost = latest_costmap_->data[py * w + px];
                        if (cost >= 70) continue;
                        // Skip if too close to a previously failed escape point
                        bool near_failed = false;
                        for (const auto& fe : failed_escape_points_) {
                            double dx = candidate.x - fe.x, dy = candidate.y - fe.y;
                            if (dx * dx + dy * dy < 4.0) { near_failed = true; break; }
                        }
                        if (!near_failed && static_cast<float>(cost) < best_cost) {
                            best_cost = static_cast<float>(cost);
                            escape_point = candidate;
                            directional_found = true;
                        }
                    }
                }
            }
            bool last_resort_found = false;
            if (!directional_found) {
                // True last resort, tried before the raw unchecked backward
                // point: both searches above have already failed with the
                // blacklist applied, so at this point a previously-tried
                // point (which may only have failed because it got preempted
                // -- see recoveryResultCallback's CANCELED branch) is still
                // far safer than committing to a point with no costmap
                // validation at all. Mirrors rad_safety_monitor_node's own
                // findFreeRetreatPoint() multi-candidate sweep, which this
                // fallback previously had no equivalent of.
                last_resort_found = findAnyNavigablePoint(
                    current_pose.pose.position, yaw, escape_point);
                if (!last_resort_found) {
                    // Nothing navigable found anywhere in the sweep either;
                    // go straight backward with no validation as the
                    // absolute final fallback.
                    escape_point.x = current_pose.pose.position.x - fallback_retreat_distance_ * cos(yaw);
                    escape_point.y = current_pose.pose.position.y - fallback_retreat_distance_ * sin(yaw);
                }
            }
            RCLCPP_ERROR(this->get_logger(), "No safe point found! Using %s fallback.",
                directional_found ? "best-direction" : (last_resort_found ? "validated-last-resort" : "unchecked-backward"));
        }

        last_escape_attempt_ = escape_point;

        // Orient toward the escape point so the robot faces its destination
        // and doesn't need to spin at the goal.
        double escape_yaw = atan2(escape_point.y - current_pose.pose.position.y,
                                   escape_point.x - current_pose.pose.position.x);
        tf2::Quaternion escape_q;
        escape_q.setRPY(0.0, 0.0, escape_yaw);
        escape_q.normalize();

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = map_frame_;
        goal_msg->pose.header.stamp = now;
        goal_msg->pose.pose.position = escape_point;
        goal_msg->pose.pose.orientation = tf2::toMsg(escape_q);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr &handle) {
                if (!handle) {
                    RCLCPP_WARN(this->get_logger(), "Recovery goal rejected by Nav2. Will retry.");
                    is_recovering_ = false;
                }
            };
        send_goal_options.result_callback =
            std::bind(&LocalRecoveryNode::recoveryResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, send_goal_options);
        last_goal_time_ = now;
        if (!is_recovering_) recovery_episodes_++;
        is_recovering_ = true;
    }
    
    bool findSafePoint(const geometry_msgs::msg::Point& robot_pos,
                       geometry_msgs::msg::Point& safe_point,
                       double search_radius = -1.0)
    {
        if (search_radius < 0.0) search_radius = recovery_search_distance_;

        nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
        sensor_msgs::msg::Image::SharedPtr heatmap;
        {
            std::scoped_lock lock(map_mutex_, heatmap_mutex_);
            if (!latest_costmap_ || !latest_heatmap_) return false;
            costmap = latest_costmap_;
            heatmap = latest_heatmap_;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(heatmap, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (...) { return false; }

        float res = costmap->info.resolution;
        float origin_x = costmap->info.origin.position.x;
        float origin_y = costmap->info.origin.position.y;
        int width = costmap->info.width;
        int height = costmap->info.height;

        int robot_px = (robot_pos.x - origin_x) / res;
        int robot_py = (robot_pos.y - origin_y) / res;
        int max_search_px = search_radius / res;

        float best_utility = -std::numeric_limits<float>::max();
        bool found = false;

        // Safety margin from map edges
        int margin = 5;

        // More granular search: balance safety and distance
        for (int r = 5; r < max_search_px; r += 5) {
            for (int i = 0; i < 360; i += 15) {
                double angle = i * M_PI / 180.0;
                int px = robot_px + r * cos(angle);
                int py = robot_py + r * sin(angle);

                // Strict boundary check
                if (px < margin || px >= width - margin || py < margin || py >= height - margin) continue;

                int map_idx = py * width + px;
                unsigned char cost = costmap->data[map_idx];
                // Check if navigable (cost < 70 avoids inflation from walls better)
                if (cost >= 0 && cost < 70) {
                    // Guard against heatmap/costmap dimension mismatch
                    float rad = (px < cv_ptr->image.cols && py < cv_ptr->image.rows)
                                ? cv_ptr->image.at<float>(py, px) : 0.0f;
                    float dist_m = r * res;

                    geometry_msgs::msg::Point candidate;
                    candidate.x = origin_x + (px + 0.5) * res;
                    candidate.y = origin_y + (py + 0.5) * res;

                    // Skip points too close to previously failed escape destinations
                    bool near_failed = false;
                    for (const auto& fe : failed_escape_points_) {
                        double dx = candidate.x - fe.x, dy = candidate.y - fe.y;
                        if (dx * dx + dy * dy < 4.0) { near_failed = true; break; }
                    }
                    if (near_failed) continue;

                    // Utility = -Radiation + Normalized_Distance - Normalized_Cost
                    float utility = -rad * 10.0f +
                                    (dist_m / search_radius) -
                                    (static_cast<float>(cost) / 50.0f);

                    if (utility > best_utility) {
                        best_utility = utility;
                        safe_point = candidate;
                        safe_point.z = 0;
                        found = true;
                    }

                    // Early exit: genuinely safe and far enough away
                    if (rad < safe_dose_threshold_ && dist_m > 5.0) {
                        return true;
                    }
                }
            }
        }
        return found;
    }

    // True last resort, used only after findSafePoint() and the directional
    // 8-way fallback have both already failed with the blacklist applied.
    // Deliberately ignores failed_escape_points_ -- by this point a
    // previously-tried point (which may only have failed because it was
    // preempted, not because it was actually unsafe) is still a validated,
    // costmap-checked point, which is strictly better than the fully
    // unchecked raw-backward point this function exists to avoid falling
    // back to. Mirrors rad_safety_monitor_node's own findFreeRetreatPoint().
    bool findAnyNavigablePoint(const geometry_msgs::msg::Point& robot_pos, double yaw,
                                geometry_msgs::msg::Point& out_point)
    {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        if (!latest_costmap_) return false;

        float res = latest_costmap_->info.resolution;
        float ox = latest_costmap_->info.origin.position.x;
        float oy = latest_costmap_->info.origin.position.y;
        int w = latest_costmap_->info.width;
        int h = latest_costmap_->info.height;

        const double angle_offsets[] = {0.0, M_PI / 12.0, -M_PI / 12.0, M_PI / 6.0, -M_PI / 6.0,
                                         M_PI / 4.0, -M_PI / 4.0};
        const double dist_fracs[] = {1.0, 0.75, 0.5};

        for (double frac : dist_fracs) {
            double dist = fallback_retreat_distance_ * frac;
            for (double da : angle_offsets) {
                double angle = yaw + M_PI + da;  // roughly behind the robot, +/- a spread
                double cx = robot_pos.x + dist * cos(angle);
                double cy = robot_pos.y + dist * sin(angle);
                int px = static_cast<int>((cx - ox) / res);
                int py = static_cast<int>((cy - oy) / res);
                if (px < 2 || px >= w - 2 || py < 2 || py >= h - 2) continue;
                unsigned char cost = latest_costmap_->data[py * w + px];
                if (cost >= 0 && cost < 70) {
                    out_point.x = cx;
                    out_point.y = cy;
                    out_point.z = 0.0;
                    return true;
                }
            }
        }
        return false;
    }
 /*
    void startPhysicalEscape()
    {
        // Drive backward slowly for ~2.5 s (≈0.25 m) to physically leave lethal space
        // before the next navigate_to_pose goal is sent. is_recovering_ stays true
        // during the escape so that recoveryTick() does not re-trigger navigation.
        RCLCPP_WARN(this->get_logger(),
            "Starting physical escape: backing up at 0.10 m/s for 2.5 s.");
        escape_elapsed_ms_ = 0;
        escape_vel_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                if (escape_elapsed_ms_ < 2500) {
                    geometry_msgs::msg::Twist twist;
                    twist.linear.x = -0.10;
                    cmd_vel_pub_->publish(twist);
                    escape_elapsed_ms_ += 50;
                } else {
                    geometry_msgs::msg::Twist stop;
                    cmd_vel_pub_->publish(stop);
                    escape_vel_timer_->cancel();
                    escape_vel_timer_.reset();
                    is_recovering_ = false;
                }
            });
    }
*/
    void recoveryResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            is_recovering_ = false;
            RCLCPP_INFO(this->get_logger(), "Successfully reached safe zone. Resuming exploration.");
            consecutive_recovery_failures_ = 0;
            failed_escape_points_.clear();
            callTriggerService(start_explore_client_, "Start Exploration");
        } else if (rad_safety_active_ || result.code == rclcpp_action::ResultCode::CANCELED) {
            // A failure while rad_safety_monitor_node currently owns an
            // in-flight goal is not evidence the point itself was unsafe --
            // the dominant cause observed directly is rad_safety_monitor_node
            // preempting this exact goal on the shared Nav2 action server
            // while reacting to the same acute-dose event ("EMERGENCY...
            // Triggering retreat" immediately followed by "Recovery
            // navigation failed" for the same escape point). This originally
            // checked only for a CANCELED result code, on the assumption
            // that preemption would produce one; it does not in this stack
            // (Nav2 reports ABORTED, code 6, for a preempted goal here, not
            // CANCELED, code 5), so the check now keys off
            // rad_safety_monitor_node's own reported active state directly,
            // which is what the code value was really trying to infer in the
            // first place. Blacklisting here compounds with every future
            // preemption, eventually excluding most of a small map's
            // reachable area over a long enough failure streak. Don't
            // blacklist or escalate the search radius for this outcome
            // specifically; recoveryTick's cooldown-gated retry will try
            // again with fresh state regardless.
            RCLCPP_WARN(this->get_logger(),
                "Recovery navigation to (%.2f, %.2f) failed while rad_safety_monitor_node was active "
                "(likely preempted); not blacklisting.",
                last_escape_attempt_.x, last_escape_attempt_.y);
        } else {
            consecutive_recovery_failures_++;
            failed_escape_points_.push_back(last_escape_attempt_);
            // Cap the blacklist so a genuine bad streak still can't lock
            // this node out of an entire small/cluttered map -- without
            // this, enough ABORTED failures in a row (each excluding a 2m
            // radius) can leave findSafePoint() and the directional
            // fallback with nothing left to offer.
            if (failed_escape_points_.size() > kMaxFailedEscapePoints) {
                failed_escape_points_.erase(failed_escape_points_.begin());
            }
            RCLCPP_WARN(this->get_logger(),
                "Recovery navigation failed (code %d, attempt #%d). "
                "Blacklisting escape point (%.2f, %.2f). Will retry.",
                static_cast<int>(result.code), consecutive_recovery_failures_,
                last_escape_attempt_.x, last_escape_attempt_.y);
            // Physical escape keeps is_recovering_ = true; recoveryTick will
            // re-trigger once the escape timer clears is_recovering_.
            // startPhysicalEscape();
        }
    }

    void callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string& label)
    {
        // Non-blocking check instead of wait_for_service(1s): this is called
        // from triggerRecovery(), a radiation-emergency path, and a 1s block
        // on the executor here stalls rad/costmap/heatmap callbacks and the
        // nav action result callback at exactly the wrong moment.
        if (!client->service_is_ready()) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available!", client->get_service_name());
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client->async_send_request(request, [this, label](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "%s succeeded: %s", label.c_str(), response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "%s failed: %s", label.c_str(), response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call %s failed: %s", label.c_str(), e.what());
            }
        });
    }

    // Parameters
    double acute_dose_rate_;    // hard evacuation trigger (10.0 mrem/hr)
    double safe_dose_threshold_;
    int max_recovery_episodes_{0};
    int recovery_episodes_{0};
    double recovery_search_distance_;
    std::string robot_base_frame_;
    std::string map_frame_;
    double fallback_retreat_distance_;
    bool return_to_origin_;
    double origin_x_;
    double origin_y_;
    std::string stop_explore_srv_name_;
    std::string start_explore_srv_name_;
    double dose_budget_relaxation_k_;
    double dose_budget_ceiling_mult_;
    double max_radiation_dose_mrem_;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_finished_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rad_safety_active_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_explore_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_explore_client_;
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
    rclcpp::TimerBase::SharedPtr recovery_timer_;
    //rclcpp::TimerBase::SharedPtr escape_vel_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State
    double current_rad_val_ = 0.0;
    std::deque<double> dose_history_;
    int dose_confirmation_count_;
    bool is_recovering_ = false;
    bool exploration_finished_ = false;
    bool rad_safety_active_ = false;
    double cumulative_dose_mrem_ = 0.0;
    rclcpp::Time last_rad_time_{0, 0, RCL_ROS_TIME};
    double authority_fraction_ = -1.0;
    rclcpp::Time authority_fraction_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr budget_authority_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr multiplier_pub_;
    rclcpp::TimerBase::SharedPtr multiplier_timer_;
    rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    sensor_msgs::msg::Image::SharedPtr latest_heatmap_;
    std::mutex map_mutex_;
    std::mutex heatmap_mutex_;
    // Recovery escalation tracking
    //int escape_elapsed_ms_{0};
    static constexpr size_t kMaxFailedEscapePoints = 8;
    int consecutive_recovery_failures_;
    std::vector<geometry_msgs::msg::Point> failed_escape_points_;
    geometry_msgs::msg::Point last_escape_attempt_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalRecoveryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
