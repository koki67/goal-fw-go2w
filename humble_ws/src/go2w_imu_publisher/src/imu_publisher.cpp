#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/low_state.hpp"

class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher()
    : Node("imu_publisher"),
      first_tick_(0),
      base_time_(0, 0, RCL_ROS_TIME),
      have_anchor_(false) {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/go2w/imu", 10);
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate", qos, std::bind(&ImuPublisher::stateCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Republishing /lowstate IMU on /go2w/imu");
    }

private:
    void stateCallback(const unitree_go::msg::LowState::SharedPtr data) {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = tickToRosTime(data->tick);
        imu_msg.header.frame_id = "imu_link";

        imu_msg.orientation.w = data->imu_state.quaternion[0];
        imu_msg.orientation.x = data->imu_state.quaternion[1];
        imu_msg.orientation.y = data->imu_state.quaternion[2];
        imu_msg.orientation.z = data->imu_state.quaternion[3];

        imu_msg.angular_velocity.x = data->imu_state.gyroscope[0];
        imu_msg.angular_velocity.y = data->imu_state.gyroscope[1];
        imu_msg.angular_velocity.z = data->imu_state.gyroscope[2];

        imu_msg.linear_acceleration.x = data->imu_state.accelerometer[0];
        imu_msg.linear_acceleration.y = data->imu_state.accelerometer[1];
        imu_msg.linear_acceleration.z = data->imu_state.accelerometer[2];

        imu_pub_->publish(imu_msg);
    }

    // Convert the LowState `tick` field (Unitree controller cycle counter, milliseconds)
    // into a ROS time. Anchored once against node clock so the IMU stream preserves the
    // controller's native sample spacing even when DDS delivers LowState in bursts —
    // necessary for D-LIO's deskew/observer to see meaningful angular-velocity deltas.
    rclcpp::Time tickToRosTime(uint32_t tick_ms) {
        if (!have_anchor_) {
            first_tick_ = tick_ms;
            base_time_ = this->get_clock()->now();
            have_anchor_ = true;
            return base_time_;
        }

        int64_t delta_ms =
            static_cast<int64_t>(tick_ms) - static_cast<int64_t>(first_tick_);

        // uint32 ms wraps every ~49.7 days; treat a huge negative delta as wrap.
        if (delta_ms < std::numeric_limits<int32_t>::min()) {
            delta_ms += static_cast<int64_t>(1) << 32;
        }

        // Backward jump > 1 s indicates a controller restart; re-anchor to keep
        // the published stamp stream monotonic.
        if (delta_ms < -1000) {
            first_tick_ = tick_ms;
            base_time_ = this->get_clock()->now();
            return base_time_;
        }

        return base_time_ + rclcpp::Duration(std::chrono::milliseconds(delta_ms));
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub_;

    uint32_t first_tick_;
    rclcpp::Time base_time_;
    bool have_anchor_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
