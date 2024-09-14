#ifndef SYNAPSE_ROS_CLIENT_HPP__
#define SYNAPSE_ROS_CLIENT_HPP__

#include <actuator_msgs/msg/actuators.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <synapse_msgs/msg/bezier_trajectory.hpp>
#include <synapse_msgs/msg/input.hpp>
#include <synapse_msgs/msg/status.hpp>

#include <synapse_pb/actuators.pb.h>
#include <synapse_pb/battery_state.pb.h>
#include <synapse_pb/bezier_trajectory.pb.h>
#include <synapse_pb/clock_offset.pb.h>
#include <synapse_pb/frame.pb.h>
#include <synapse_pb/imu.pb.h>
#include <synapse_pb/magnetic_field.pb.h>
#include <synapse_pb/nav_sat_fix.pb.h>
#include <synapse_pb/odometry.pb.h>
#include <synapse_pb/status.pb.h>
#include <synapse_pb/twist.pb.h>
#include <synapse_pb/wheel_odometry.pb.h>

class UDPLink;

void udp_entry_point();

class SynapseRos : public rclcpp::Node {
public:
    SynapseRos();
    virtual ~SynapseRos();
    void udp_send(const synapse_pb::Frame& frame) const;
    void publish_actuators(const synapse_pb::Actuators& msg);
    void publish_odometry(const synapse_pb::Odometry& msg);
    void publish_battery_state(const synapse_pb::BatteryState& msg);
    void publish_nav_sat_fix(const synapse_pb::NavSatFix& msg);
    void publish_status(const synapse_pb::Status& msg);
    void publish_uptime(const synapse_pb::ClockOffset& msg);

private:
    builtin_interfaces::msg::Time ros_clock_offset_ {};
    builtin_interfaces::msg::Time compute_stamp(const synapse_pb::Timestamp& msg);
    void udp_run(void);

    // subscriptions ros -> cerebri
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr sub_actuators_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sub_clock_offset_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_state_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheel_odometry_;
    rclcpp::Subscription<synapse_msgs::msg::Input>::SharedPtr sub_input_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_magnetic_field_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_nav_sat_fix_;
    rclcpp::Subscription<synapse_msgs::msg::BezierTrajectory>::SharedPtr sub_bezier_trajectory_;

    // subscription callbacks
    void actuators_callback(const actuator_msgs::msg::Actuators& msg) const;
    void battery_state_callback(const sensor_msgs::msg::BatteryState& msg) const;
    void bezier_trajectory_callback(const synapse_msgs::msg::BezierTrajectory& msg) const;
    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) const;
    void imu_callback(const sensor_msgs::msg::Imu& msg) const;
    void input_callback(const synapse_msgs::msg::Input& msg) const;
    void magnetic_field_callback(const sensor_msgs::msg::MagneticField& msg) const;
    void nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix& msg) const;
    void odometry_callback(const nav_msgs::msg::Odometry& msg) const;
    void clock_offset_callback(const builtin_interfaces::msg::Time& msg) const;
    void wheel_odometry_callback(const sensor_msgs::msg::JointState& msg) const;

    // publications cerebri -> ros
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_actuators_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;
    rclcpp::Publisher<synapse_msgs::msg::Status>::SharedPtr pub_status_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_uptime_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_clock_offset_;

    // threading
    std::shared_ptr<std::thread> udp_thread_;
    std::shared_ptr<UDPLink> udp_link_;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__
