#include "CUSB_RAD121.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <deque>
#include <numeric>

class Rad121MonitorNode : public rclcpp::Node
{
public:
Rad121MonitorNode()
    : Node("rad121_monitor_node"), decayRate(0.99), updateInterval(1)
{
    this->declare_parameter<bool>("auto_calibrate", true);
    this->declare_parameter<int>("calibration_duration_s", 30);
}

void init()
{
    rad121_ = std::make_shared<CUSB_RAD121>();
    rad121_->ConfigureFromNode(shared_from_this());

    if (!rad121_->Open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open FTDI device");
        rclcpp::shutdown();
    }

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rad", 10);
    background_pub_ = this->create_publisher<std_msgs::msg::Float64>("/rad_field_server/background_radiation", rclcpp::QoS(1).transient_local());

    auto_calibrate_ = this->get_parameter("auto_calibrate").as_bool();
    calibration_duration_s_ = this->get_parameter("calibration_duration_s").as_int();
    is_calibrating_ = auto_calibrate_;
    calibration_start_time_ = this->now();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / updateInterval),
        std::bind(&Rad121MonitorNode::MonitorClicks, this));
}

    ~Rad121MonitorNode()
    {
        rad121_->Close();
    }

private:
    void MonitorClicks()
    {
        unsigned char buffer[64];
        auto currentTime = std::chrono::steady_clock::now();

        int count = rad121_->ReadData(buffer, sizeof(buffer));
        if (count < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading data from FTDI device");
            rclcpp::shutdown();
        }

        if (count > 0)
        {
            for (int i = 0; i < count; ++i)
            {
                countTimestamps.push_back(currentTime);
            }
        }

        while (!countTimestamps.empty() &&
               std::chrono::duration_cast<std::chrono::seconds>(currentTime - countTimestamps.front()).count() >= 1)
        {
            countTimestamps.pop_front();
        }

        double cps = countTimestamps.size();
        double cpm = cps * 60.0;
        double cpm_comp = rad121_->Calculate_CompensatedCPM(cpm);
        double mR_hr = rad121_->Calculate_mRhr(cpm_comp);

        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(cpm_comp);
        message.data.push_back(mR_hr);
        publisher_->publish(message);

        if (is_calibrating_) {
            calibration_samples_.push_back(mR_hr);
            auto elapsed = this->now() - calibration_start_time_;
            if (elapsed.seconds() >= calibration_duration_s_) {
                double sum = std::accumulate(calibration_samples_.begin(), calibration_samples_.end(), 0.0);
                double mean = sum / (double)calibration_samples_.size();
                
                std_msgs::msg::Float64 bg_msg;
                bg_msg.data = mean;
                background_pub_->publish(bg_msg);
                
                RCLCPP_INFO(this->get_logger(), "Auto-calibration complete. Background level set to: %.6f mR/hr", mean);
                is_calibrating_ = false;
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                    "Calibrating background... %.0f%% complete", (elapsed.seconds() / (double)calibration_duration_s_) * 100.0);
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Compensated CPM: %.2f | mR/hr: %.6f", cpm_comp, mR_hr);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr background_pub_;
    std::shared_ptr<CUSB_RAD121> rad121_;
    std::deque<std::chrono::steady_clock::time_point> countTimestamps;

    bool auto_calibrate_;
    bool is_calibrating_;
    int calibration_duration_s_;
    rclcpp::Time calibration_start_time_;
    std::vector<double> calibration_samples_;

    const double decayRate;
    const int updateInterval;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Rad121MonitorNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

