#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rad_simulation/msg/rad_source_array.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

#include <vector>
#include <random>
#include <string>
#include <memory>
#include <mutex>

// Struct to hold information about a single radiation source
struct RadSource
{
    int id;
    geometry_msgs::msg::Point position;
    double intensity; // mrem/hr
};

#include "rad_simulation/srv/spawn_source.hpp"
#include "rad_simulation/srv/spawn_random_sources.hpp"
#include "rad_simulation/srv/set_float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class RadFieldServer : public rclcpp::Node
{
public:
    RadFieldServer() : Node("rad_field_server"), tf_buffer_(this->get_clock()), next_source_id_(0)
    {
        // Initialize TF listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Initialize random number generation
        rng_.seed(std::random_device()());

        // Declare parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<bool>("enable_sources", true);
        this->declare_parameter<std::vector<double>>("known_sources", std::vector<double>{});
        this->declare_parameter<int>("num_unknown_sources", 3);
        this->declare_parameter<int>("num_high_intensity_sources", 1);
        this->declare_parameter<double>("high_intensity_threshold", 10.0);
        this->declare_parameter<double>("low_intensity_min", 2.5);
        this->declare_parameter<double>("low_intensity_max", 5.0);
        this->declare_parameter<bool>("background_radiation_enabled", false);
        this->declare_parameter<double>("background_radiation_min", 0.05);
        this->declare_parameter<double>("background_radiation_max", 1.0);
        this->declare_parameter<std::string>("output_yaml_file", "/home/brendoncintas/radmon_ws/src/ROS2_Rad121/rad_simulation/params/saved_sources.yaml");

        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        output_file_ = this->get_parameter("output_yaml_file").as_string();

        // Create the service to spawn a new source at specific coordinates
        spawn_service_ = this->create_service<rad_simulation::srv::SpawnSource>(
            "~/spawn_source",
            std::bind(&RadFieldServer::spawn_source_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create the service to spawn random sources on the map
        spawn_random_service_ = this->create_service<rad_simulation::srv::SpawnRandomSources>(
            "~/spawn_random_sources",
            std::bind(&RadFieldServer::spawn_random_sources_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create the service to spawn a new source at robot's pose
        spawn_at_robot_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/spawn_at_robot",
            std::bind(&RadFieldServer::spawn_at_robot_callback, this, std::placeholders::_1, std::placeholders::_2));

        // New services for real-time control
        enable_sources_service_ = this->create_service<std_srvs::srv::SetBool>(
            "~/enable_sources",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                this->set_parameter(rclcpp::Parameter("enable_sources", request->data));
                response->success = true;
                response->message = request->data ? "Sources enabled" : "Sources disabled";
                RCLCPP_INFO(this->get_logger(), "Real-time control: %s", response->message.c_str());
            });

        enable_background_service_ = this->create_service<std_srvs::srv::SetBool>(
            "~/enable_background",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                this->set_parameter(rclcpp::Parameter("background_radiation_enabled", request->data));
                response->success = true;
                response->message = request->data ? "Background enabled" : "Background disabled";
                RCLCPP_INFO(this->get_logger(), "Real-time control: %s", response->message.c_str());
            });

        set_background_level_service_ = this->create_service<rad_simulation::srv::SetFloat64>(
            "~/set_background_level",
            [this](const std::shared_ptr<rad_simulation::srv::SetFloat64::Request> request,
                   std::shared_ptr<rad_simulation::srv::SetFloat64::Response> response) {
                this->set_parameter(rclcpp::Parameter("background_radiation_min", request->data));
                this->set_parameter(rclcpp::Parameter("background_radiation_max", request->data));
                response->success = true;
                response->message = "Fixed background level set to " + std::to_string(request->data);
                RCLCPP_INFO(this->get_logger(), "Real-time control: %s", response->message.c_str());
            });

        clear_sources_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/clear_sources",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                sources_.clear();
                response->success = true;
                response->message = "All sources cleared";
                RCLCPP_INFO(this->get_logger(), "Real-time control: %s", response->message.c_str());
                publish_markers();
            });

        // Save sources to file service
        save_sources_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/save_sources",
            std::bind(&RadFieldServer::save_sources_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Load sources from file service
        load_sources_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/load_sources",
            std::bind(&RadFieldServer::load_sources_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create the publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/sources", 10);
        
        // Create the publisher for background radiation level
        background_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/background_radiation", 10);

        // Create the publisher for raw source data
        sources_data_pub_ = this->create_publisher<rad_simulation::msg::RadSourceArray>("~/sources_data", 10);

        // Create a timer to periodically publish markers and background radiation
        update_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RadFieldServer::update_callback, this));
        
        // Subscribe to the map
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&RadFieldServer::map_callback, this, std::placeholders::_1));

        // Setup parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RadFieldServer::on_set_parameters, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Radiation Field Server has started.");
    }

private:
    void update_callback()
    {
        publish_markers();
        publish_background();
        publish_sources_data();
    }

    void publish_sources_data()
    {
        rad_simulation::msg::RadSourceArray msg;
        if (this->get_parameter("enable_sources").as_bool()) {
            for (const auto &source : sources_) {
                rad_simulation::msg::RadSource s;
                s.id = source.id;
                s.position = source.position;
                s.intensity = source.intensity;
                msg.sources.push_back(s);
            }
        }
        sources_data_pub_->publish(msg);
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = *msg;
        bool first_time = !map_received_;
        map_received_ = true;
        
        if (first_time) {
            RCLCPP_INFO(this->get_logger(), "Map received. Spawning initial sources.");
            spawn_initial_sources();
        }
    }

    void spawn_initial_sources()
    {
        // 1. Spawn known sources from parameters
        std::vector<double> known_sources = this->get_parameter("known_sources").as_double_array();
        for (size_t i = 0; i + 2 < known_sources.size(); i += 3) {
            RadSource source;
            source.id = next_source_id_++;
            source.position.x = known_sources[i];
            source.position.y = known_sources[i+1];
            source.position.z = 0.0;
            source.intensity = known_sources[i+2];
            sources_.push_back(source);
            RCLCPP_INFO(this->get_logger(), "Spawned known source %d at (%.2f, %.2f) with intensity %.2f mrem/hr.",
                        source.id, source.position.x, source.position.y, source.intensity);
        }

        // 2. Spawn random sources
        int num_unknown = this->get_parameter("num_unknown_sources").as_int();
        int num_high = this->get_parameter("num_high_intensity_sources").as_int();
        spawn_random_sources(num_unknown, num_high);
    }

    void spawn_random_sources(int num_unknown, int num_high)
    {
        if (!map_received_) return;

        unsigned int map_width = current_map_.info.width;
        unsigned int map_height = current_map_.info.height;
        double map_res = current_map_.info.resolution;
        double map_origin_x = current_map_.info.origin.position.x;
        double map_origin_y = current_map_.info.origin.position.y;

        std::uniform_int_distribution<unsigned int> x_dist(0, map_width - 1);
        std::uniform_int_distribution<unsigned int> y_dist(0, map_height - 1);
        
        double low_min = this->get_parameter("low_intensity_min").as_double();
        double low_max = this->get_parameter("low_intensity_max").as_double();
        double high_threshold = this->get_parameter("high_intensity_threshold").as_double();
        
        std::uniform_real_distribution<double> low_intensity_dist(low_min, low_max);
        std::uniform_real_distribution<double> high_intensity_dist(high_threshold, high_threshold * 2.0);

        for (int i = 0; i < num_unknown; ++i) {
            unsigned int rand_mx, rand_my;
            int attempts = 0;
            const int max_attempts = 100;

            do {
                rand_mx = x_dist(rng_);
                rand_my = y_dist(rng_);
                attempts++;
            } while (current_map_.data[rand_my * map_width + rand_mx] != 0 && attempts < max_attempts);

            if (attempts == max_attempts) continue;

            RadSource new_source;
            new_source.id = next_source_id_++;
            new_source.position.x = map_origin_x + (rand_mx + 0.5) * map_res;
            new_source.position.y = map_origin_y + (rand_my + 0.5) * map_res;
            new_source.position.z = 0.0;
            
            if (i < num_high) {
                new_source.intensity = high_intensity_dist(rng_);
                RCLCPP_INFO(this->get_logger(), "Spawned high-intensity unknown source %d at (%.2f, %.2f) with intensity %.2f mrem/hr.",
                            new_source.id, new_source.position.x, new_source.position.y, new_source.intensity);
            } else {
                new_source.intensity = low_intensity_dist(rng_);
                RCLCPP_INFO(this->get_logger(), "Spawned unknown source %d at (%.2f, %.2f) with intensity %.2f mrem/hr.",
                            new_source.id, new_source.position.x, new_source.position.y, new_source.intensity);
            }

            sources_.push_back(new_source);
        }
    }

    void spawn_source_callback(const std::shared_ptr<rad_simulation::srv::SpawnSource::Request> request,
                               std::shared_ptr<rad_simulation::srv::SpawnSource::Response> response)
    {
        RadSource new_source;
        new_source.id = next_source_id_++;
        new_source.position.x = request->x;
        new_source.position.y = request->y;
        new_source.position.z = 0.0;
        new_source.intensity = request->intensity;

        sources_.push_back(new_source);

        RCLCPP_INFO(this->get_logger(), "Spawned source %d at (%.2f, %.2f) with intensity %.2f mrem/hr.",
                    new_source.id, new_source.position.x, new_source.position.y, new_source.intensity);

        publish_markers();

        response->success = true;
        response->message = "Radiation source spawned successfully.";
    }

    void spawn_random_sources_callback(const std::shared_ptr<rad_simulation::srv::SpawnRandomSources::Request> request,
                                       std::shared_ptr<rad_simulation::srv::SpawnRandomSources::Response> response)
    {
        if (!map_received_) {
            response->success = false;
            response->message = "Map not received yet.";
            return;
        }
        int num_high = this->get_parameter("num_high_intensity_sources").as_int();
        spawn_random_sources(request->num_sources, num_high);
        response->success = true;
        response->message = "Successfully spawned " + std::to_string(request->num_sources) + " random sources.";
    }

    void spawn_at_robot_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused

        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                         robot_frame_.c_str(), map_frame_.c_str(), ex.what());
            response->success = false;
            response->message = "TF lookup failed.";
            return;
        }

        RadSource new_source;
        new_source.id = next_source_id_++;
        new_source.position.x = t.transform.translation.x;
        new_source.position.y = t.transform.translation.y;
        new_source.position.z = t.transform.translation.z;
        
        std::uniform_real_distribution<double> dist(2.5, 10.0);
        new_source.intensity = dist(rng_);

        sources_.push_back(new_source);

        RCLCPP_INFO(this->get_logger(), "Spawned source %d at robot pose (%.2f, %.2f, %.2f) with intensity %.2f mrem/hr.",
                    new_source.id, new_source.position.x, new_source.position.y, new_source.position.z, new_source.intensity);

        publish_markers();

        response->success = true;
        response->message = "Radiation source spawned at robot pose successfully.";
    }

    void save_sources_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try {
            YAML::Node node;
            std::vector<double> flat_list;
            for (const auto& s : sources_) {
                flat_list.push_back(s.position.x);
                flat_list.push_back(s.position.y);
                flat_list.push_back(s.intensity);
            }
            node["rad_field_server"]["ros__parameters"]["known_sources"] = flat_list;

            std::filesystem::path p(output_file_);
            if (p.has_parent_path()) {
                std::filesystem::create_directories(p.parent_path());
            }

            std::ofstream fout(output_file_);
            fout << node;
            fout.close();

            response->success = true;
            response->message = "Saved " + std::to_string(sources_.size()) + " sources to " + output_file_;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to save sources: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void load_sources_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try {
            if (!std::filesystem::exists(output_file_)) {
                response->success = false;
                response->message = "Source file does not exist: " + output_file_;
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            YAML::Node node = YAML::LoadFile(output_file_);
            if (!node["rad_field_server"] || !node["rad_field_server"]["ros__parameters"] || 
                !node["rad_field_server"]["ros__parameters"]["known_sources"]) {
                response->success = false;
                response->message = "Invalid YAML format in " + output_file_;
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            std::vector<double> flat_list = node["rad_field_server"]["ros__parameters"]["known_sources"].as<std::vector<double>>();
            
            // Clear existing sources
            sources_.clear();
            next_source_id_ = 0;

            for (size_t i = 0; i + 2 < flat_list.size(); i += 3) {
                RadSource source;
                source.id = next_source_id_++;
                source.position.x = flat_list[i];
                source.position.y = flat_list[i+1];
                source.position.z = 0.0;
                source.intensity = flat_list[i+2];
                sources_.push_back(source);
            }

            // Enable sources
            this->set_parameter(rclcpp::Parameter("enable_sources", true));

            response->success = true;
            response->message = "Loaded " + std::to_string(sources_.size()) + " sources from " + output_file_ + " and enabled them.";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            
            publish_markers();
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to load sources: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        auto now = this->get_clock()->now();

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_;
        clear_marker.header.stamp = now;
        clear_marker.id = 0;
        clear_marker.ns = "rad_sources";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        if (this->get_parameter("enable_sources").as_bool()) {
            for (const auto &source : sources_)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = map_frame_;
                marker.header.stamp = now;
                marker.ns = "rad_sources";
                marker.id = source.id + 1;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position = source.position;
                marker.pose.orientation.w = 1.0;

                double high_thresh = this->get_parameter("high_intensity_threshold").as_double();
                double low_min = this->get_parameter("low_intensity_min").as_double();
                double normalized_intensity = (source.intensity - low_min) / (high_thresh - low_min);
                normalized_intensity = std::max(0.0, std::min(1.0, normalized_intensity));

                marker.scale.x = 0.2 + normalized_intensity * 0.8;
                marker.scale.y = 0.2 + normalized_intensity * 0.8;
                marker.scale.z = 0.2 + normalized_intensity * 0.8;
                
                marker.color.a = 0.8;
                marker.color.r = static_cast<float>(normalized_intensity);
                marker.color.g = static_cast<float>(1.0 - normalized_intensity);
                marker.color.b = 0.0;

                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_->publish(marker_array);
    }

    void publish_background()
    {
        std_msgs::msg::Float64 msg;
        if (this->get_parameter("background_radiation_enabled").as_bool()) {
            double bg_min = this->get_parameter("background_radiation_min").as_double();
            double bg_max = this->get_parameter("background_radiation_max").as_double();
            
            if (bg_max <= bg_min) {
                msg.data = bg_min;
            } else {
                std::uniform_real_distribution<double> background_dist(bg_min, bg_max);
                msg.data = background_dist(rng_);
            }
        } else {
            msg.data = 0.0;
        }
        background_pub_->publish(msg);
    }

    rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters) {
            if (param.get_name() == "known_sources" || param.get_name() == "num_unknown_sources") {
                // We don't re-spawn everything on parameter change to avoid duplicates,
                // but one could implement a reset logic here.
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed. Note: Sources already spawned are not affected.", param.get_name().c_str());
            }
        }
        return result;
    }

    // ROS Comms
    rclcpp::Service<rad_simulation::srv::SpawnSource>::SharedPtr spawn_service_;
    rclcpp::Service<rad_simulation::srv::SpawnRandomSources>::SharedPtr spawn_random_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr spawn_at_robot_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_sources_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_background_service_;
    rclcpp::Service<rad_simulation::srv::SetFloat64>::SharedPtr set_background_level_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_sources_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_sources_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_sources_service_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr background_pub_;
    rclcpp::Publisher<rad_simulation::msg::RadSourceArray>::SharedPtr sources_data_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;
    std::string output_file_;

    // State
    std::vector<RadSource> sources_;
    int next_source_id_;
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;

    // Random number generation
    std::mt19937 rng_;
    
    // Mutex for map access
    std::mutex map_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadFieldServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}