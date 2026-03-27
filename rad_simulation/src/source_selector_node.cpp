#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rad_simulation/srv/spawn_source.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <mutex>
#include <filesystem>

struct SourceDef {
    double x, y, intensity;
};

class SourceSelectorNode : public rclcpp::Node
{
public:
    SourceSelectorNode() : Node("source_selector_node")
    {
        this->declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
        this->declare_parameter<std::string>("output_yaml_file",
            ament_index_cpp::get_package_share_directory("rad_simulation") + "/params/manual_sources.yaml");
        this->declare_parameter<double>("default_intensity", 5.0);

        std::string clicked_point_topic = this->get_parameter("clicked_point_topic").as_string();
        output_file_ = this->get_parameter("output_yaml_file").as_string();
        default_intensity_ = this->get_parameter("default_intensity").as_double();

        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            clicked_point_topic, 10, std::bind(&SourceSelectorNode::pointCallback, this, std::placeholders::_1));

        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/save_sources", std::bind(&SourceSelectorNode::saveCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        clear_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/clear_sources", std::bind(&SourceSelectorNode::clearCallback, this, std::placeholders::_1, std::placeholders::_2));

        spawn_source_client_ = this->create_client<rad_simulation::srv::SpawnSource>("/rad_field_server/spawn_source");

        RCLCPP_INFO(this->get_logger(), "Source Selector Node started. Use 'Publish Point' in RViz2 to define sources.");
        RCLCPP_INFO(this->get_logger(), "Sources will be saved to: %s", output_file_.c_str());
    }

private:
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        SourceDef s;
        s.x = msg->point.x;
        s.y = msg->point.y;
        s.intensity = default_intensity_;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            sources_.push_back(s);
        }

        RCLCPP_INFO(this->get_logger(), "Received point from RViz: (%.2f, %.2f)", s.x, s.y);

        // Try to push to simulation in real-time
        if (spawn_source_client_->service_is_ready()) {
            auto request = std::make_shared<rad_simulation::srv::SpawnSource::Request>();
            request->x = s.x;
            request->y = s.y;
            request->intensity = s.intensity;
            
            spawn_source_client_->async_send_request(request, 
                [this](rclcpp::Client<rad_simulation::srv::SpawnSource>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Successfully pushed source to simulation: %s", response->message.c_str());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to push source to simulation: %s", response->message.c_str());
                    }
                });
        } else {
            RCLCPP_WARN(this->get_logger(), "Simulation server not ready. Point stored locally but not pushed to sim.");
        }
    }

    void saveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
            YAML::Node node;
            std::vector<double> flat_list;
            for (const auto& s : sources_) {
                flat_list.push_back(s.x);
                flat_list.push_back(s.y);
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
            response->message = std::string("Failed to save: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void clearCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sources_.clear();
        response->success = true;
        response->message = "Cleared local source list.";
        RCLCPP_INFO(this->get_logger(), "Cleared local source list.");
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
    rclcpp::Client<rad_simulation::srv::SpawnSource>::SharedPtr spawn_source_client_;

    std::vector<SourceDef> sources_;
    std::mutex mutex_;
    std::string output_file_;
    double default_intensity_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SourceSelectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
