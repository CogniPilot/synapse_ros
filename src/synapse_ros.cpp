#include "synapse_ros.hpp"
#include "link/udp_link.hpp"
#include <google/protobuf/util/delimited_message_util.h>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/detail/battery_state__struct.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sensor_msgs/msg/detail/magnetic_field__struct.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <synapse_pb/battery_state.pb.h>
#include <synapse_pb/frame.pb.h>
#include <synapse_pb/input.pb.h>
#include <synapse_pb/magnetic_field.pb.h>
#include <synapse_pb/nav_sat_fix.pb.h>
#include <synapse_pb/wheel_odometry.pb.h>

using namespace google::protobuf::util;

using std::placeholders::_1;
std::shared_ptr<UDPLink> g_udp_link { NULL };

void udp_entry_point()
{
    while (rclcpp::ok()) {
        g_udp_link->run_for(std::chrono::seconds(1));
    }
}

SynapseRos::SynapseRos()
    : Node("synapse_ros")
{
    this->declare_parameter("host", "192.0.2.1");
    this->declare_parameter("port", 4242);
    this->declare_parameter("hil_mode", false);

    std::string host = this->get_parameter("host").as_string();
    int port = this->get_parameter("port").as_int();
    bool hil_mode = this->get_parameter("hil_mode").as_bool();

    // subscriptions ros -> cerebri
    sub_actuators_ = this->create_subscription<actuator_msgs::msg::Actuators>(
        "in/actuators", 10, std::bind(&SynapseRos::actuators_callback, this, _1));

    sub_bezier_trajectory_ = this->create_subscription<synapse_msgs::msg::BezierTrajectory>(
        "in/bezier_trajectory", 10, std::bind(&SynapseRos::bezier_trajectory_callback, this, _1));

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "in/cmd_vel", 10, std::bind(&SynapseRos::cmd_vel_callback, this, _1));

    sub_input_ = this->create_subscription<synapse_msgs::msg::Input>(
        "in/input", 10, std::bind(&SynapseRos::input_callback, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "in/odometry", 10, std::bind(&SynapseRos::odometry_callback, this, _1));

    sub_clock_offset_ = this->create_subscription<builtin_interfaces::msg::Time>(
        "out/clock_offset", 10, std::bind(&SynapseRos::clock_offset_callback, this, _1));

    if (hil_mode) {
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "in/imu", 10, std::bind(&SynapseRos::imu_callback, this, _1));

        sub_wheel_odometry_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "in/wheel_odometry", 10, std::bind(&SynapseRos::wheel_odometry_callback, this, _1));

        sub_battery_state_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "in/battery_state", 10, std::bind(&SynapseRos::battery_state_callback, this, _1));

        sub_magnetic_field_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "in/magnetic_field", 10, std::bind(&SynapseRos::magnetic_field_callback, this, _1));

        sub_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "in/nav_sat_fix", 10, std::bind(&SynapseRos::nav_sat_fix_callback, this, _1));
    }

    // publications cerebri -> ros
    pub_actuators_ = this->create_publisher<actuator_msgs::msg::Actuators>("out/actuators", 10);
    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("out/odometry", 10);
    pub_battery_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>("out/battery_state", 10);
    pub_nav_sat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("out/nav_sat_fix", 10);

    pub_status_ = this->create_publisher<synapse_msgs::msg::Status>("out/status", 10);
    pub_uptime_ = this->create_publisher<builtin_interfaces::msg::Time>("out/uptime", 10);
    pub_clock_offset_ = this->create_publisher<builtin_interfaces::msg::Time>("out/clock_offset", 10);

    // create udp link
    g_udp_link = std::make_shared<UDPLink>(host, port);
    g_udp_link.get()->ros_ = this;
    udp_thread_ = std::make_shared<std::thread>(udp_entry_point);
}

SynapseRos::~SynapseRos()
{
    // join threads
    udp_thread_->join();
}

std_msgs::msg::Header SynapseRos::compute_header(const synapse_pb::Header& msg)
{
    std_msgs::msg::Header ros_msg;
    ros_msg.frame_id = msg.frame_id();
    if (msg.has_stamp()) {
        int64_t sec = msg.stamp().sec() + ros_clock_offset_.sec;
        int64_t nanos = msg.stamp().nanosec() + ros_clock_offset_.nanosec;
        int extra_sec = nanos / 1e9;
        nanos -= extra_sec * 1e9;
        sec += extra_sec;
        ros_msg.stamp.sec = sec;
        ros_msg.stamp.nanosec = nanos;
    }
    return ros_msg;
}

void SynapseRos::publish_actuators(const synapse_pb::Actuators& msg)
{
    actuator_msgs::msg::Actuators ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header = compute_header(msg.header());
    }

    // actuators
    for (auto it = msg.position().begin(); it != msg.position().end(); it++) {
        ros_msg.position.push_back(*it);
    }

    for (auto it = msg.velocity().begin(); it != msg.velocity().end(); it++) {
        ros_msg.velocity.push_back(*it);
    }

    for (auto it = msg.normalized().begin(); it != msg.normalized().end(); it++) {
        ros_msg.normalized.push_back(*it);
    }

    pub_actuators_->publish(ros_msg);
}

void SynapseRos::publish_odometry(const synapse_pb::Odometry& msg)
{
    nav_msgs::msg::Odometry ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header = compute_header(msg.header());
    }

    // child frame id
    ros_msg.child_frame_id = msg.child_frame_id();

    // pose
    ros_msg.pose.pose.position.x = msg.pose().pose().position().x();
    ros_msg.pose.pose.position.y = msg.pose().pose().position().y();
    ros_msg.pose.pose.position.z = msg.pose().pose().position().z();
    ros_msg.pose.pose.orientation.x = msg.pose().pose().orientation().x();
    ros_msg.pose.pose.orientation.y = msg.pose().pose().orientation().y();
    ros_msg.pose.pose.orientation.z = msg.pose().pose().orientation().z();
    ros_msg.pose.pose.orientation.w = msg.pose().pose().orientation().w();

    // twist
    ros_msg.twist.twist.linear.x = msg.twist().twist().linear().x();
    ros_msg.twist.twist.linear.y = msg.twist().twist().linear().y();
    ros_msg.twist.twist.linear.z = msg.twist().twist().linear().z();
    ros_msg.twist.twist.angular.x = msg.twist().twist().angular().x();
    ros_msg.twist.twist.angular.y = msg.twist().twist().angular().y();
    ros_msg.twist.twist.angular.z = msg.twist().twist().angular().z();

    pub_odometry_->publish(ros_msg);
}

void SynapseRos::publish_battery_state(const synapse_pb::BatteryState& msg)
{
    sensor_msgs::msg::BatteryState ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header = compute_header(msg.header());
    }

    ros_msg.voltage = msg.voltage();
    pub_battery_state_->publish(ros_msg);
}

void SynapseRos::publish_status(const synapse_pb::Status& msg)
{
    synapse_msgs::msg::Status ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header = compute_header(msg.header());
    }

    ros_msg.arming = msg.arming();
    ros_msg.status_message = msg.status_message();
    ros_msg.input_status = msg.input_status();
    ros_msg.input_source = msg.input_source();
    ros_msg.topic_status = msg.topic_status();
    ros_msg.topic_source = msg.topic_source();
    ros_msg.electrode_status = msg.electrode_status();
    ros_msg.mode = msg.mode();
    ros_msg.safety = msg.safety();
    ros_msg.fuel = msg.fuel();
    ros_msg.fuel_percentage = msg.fuel_percentage();
    ros_msg.power = msg.power();
    ros_msg.request_seq = msg.request_seq();
    ros_msg.request_rejected = msg.request_rejected();

    pub_status_->publish(ros_msg);
}

void SynapseRos::publish_nav_sat_fix(const synapse_pb::NavSatFix& msg)
{
    sensor_msgs::msg::NavSatFix ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header = compute_header(msg.header());
    }

    ros_msg.latitude = msg.latitude();
    ros_msg.longitude = msg.longitude();
    ros_msg.altitude = msg.altitude();

    pub_nav_sat_fix_->publish(ros_msg);
}

void SynapseRos::publish_uptime(const synapse_pb::Time& msg)
{
    builtin_interfaces::msg::Time ros_uptime;
    rclcpp::Time now = get_clock()->now();

    int64_t uptime_nanos = msg.sec() * 1e9 + msg.nanosec();
    int64_t clock_offset_nanos = now.nanoseconds() - uptime_nanos;

    ros_uptime.sec = msg.sec();
    ros_uptime.nanosec = msg.nanosec();

    ros_clock_offset_.sec = clock_offset_nanos / 1e9;
    ros_clock_offset_.nanosec = clock_offset_nanos - ros_clock_offset_.sec * 1e9;

    pub_uptime_->publish(ros_uptime);
    pub_clock_offset_->publish(ros_clock_offset_);
}

void SynapseRos::actuators_callback(const actuator_msgs::msg::Actuators& msg) const
{
    synapse_pb::Actuators syn_msg;

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    // actuators
    for (auto i = 0u; i < msg.position.size(); ++i) {
        syn_msg.add_position(msg.position[i]);
    }

    for (auto i = 0u; i < msg.velocity.size(); ++i) {
        syn_msg.add_velocity(msg.velocity[i]);
    }
    for (auto i = 0u; i < msg.normalized.size(); ++i) {
        syn_msg.add_normalized(msg.normalized[i]);
    }

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_ACTUATORS);
    frame.set_allocated_actuators(&syn_msg);
    udp_send(frame);
    frame.release_actuators();
}

void SynapseRos::bezier_trajectory_callback(const synapse_msgs::msg::BezierTrajectory& msg) const
{
    synapse_pb::BezierTrajectory syn_msg;

    syn_msg.set_time_start(msg.time_start);

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    for (auto i = 0u; i < msg.curves.size(); ++i) {
        synapse_pb::BezierCurve* curve = syn_msg.add_curves();

        curve->set_time_stop(msg.curves[i].time_stop);

        for (auto j = 0u; j < msg.curves[i].x.size(); ++j) {
            curve->add_x(msg.curves[i].x[j]);
        }

        for (auto j = 0u; j < msg.curves[i].y.size(); ++j) {
            curve->add_y(msg.curves[i].y[j]);
        }

        for (auto j = 0u; j < msg.curves[i].z.size(); ++j) {
            curve->add_z(msg.curves[i].z[j]);
        }

        for (auto j = 0u; j < msg.curves[i].yaw.size(); ++j) {
            curve->add_yaw(msg.curves[i].yaw[j]);
        }
    }

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_BEZIER_TRAJECTORY);
    frame.set_allocated_bezier_trajectory(&syn_msg);
    udp_send(frame);
    frame.release_bezier_trajectory();
}

void SynapseRos::cmd_vel_callback(const geometry_msgs::msg::Twist& msg) const
{
    synapse_pb::Twist syn_msg;

    // twist
    syn_msg.mutable_linear()->set_x(msg.linear.x);
    syn_msg.mutable_linear()->set_y(msg.linear.y);
    syn_msg.mutable_linear()->set_z(msg.linear.z);
    syn_msg.mutable_angular()->set_x(msg.angular.x);
    syn_msg.mutable_angular()->set_y(msg.angular.y);
    syn_msg.mutable_angular()->set_z(msg.angular.z);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_CMD_VEL);
    frame.set_allocated_twist(&syn_msg);
    udp_send(frame);
    frame.release_twist();
}

void SynapseRos::input_callback(const synapse_msgs::msg::Input& msg) const
{
    synapse_pb::Input syn_msg;
    for (auto i = 0u; i < msg.channel.size(); ++i) {
        syn_msg.add_channel(msg.channel[i]);
    }

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_INPUT);
    frame.set_allocated_input(&syn_msg);
    udp_send(frame);
    frame.release_input();
}

void SynapseRos::odometry_callback(const nav_msgs::msg::Odometry& msg) const
{
    // construct empty syn_msg
    synapse_pb::Odometry syn_msg {};

    // child frame
    syn_msg.set_child_frame_id(msg.child_frame_id);

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    // pose
    syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_x(msg.pose.pose.position.x);
    syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_y(msg.pose.pose.position.y);
    syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_z(msg.pose.pose.position.z);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(msg.pose.pose.orientation.x);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(msg.pose.pose.orientation.y);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(msg.pose.pose.orientation.z);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(msg.pose.pose.orientation.w);
    // skipping covariance

    // twist
    syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_x(msg.twist.twist.linear.x);
    syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_y(msg.twist.twist.linear.y);
    syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_z(msg.twist.twist.linear.z);
    syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_x(msg.twist.twist.angular.x);
    syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_y(msg.twist.twist.angular.y);
    syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_z(msg.twist.twist.angular.z);
    // skipping covariance

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_ODOMETRY);
    frame.set_allocated_odometry(&syn_msg);
    udp_send(frame);
    frame.release_odometry();
}

void SynapseRos::imu_callback(const sensor_msgs::msg::Imu& msg) const
{
    // construct empty syn_msg
    synapse_pb::Imu syn_msg {};

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    // construct message
    syn_msg.mutable_linear_acceleration()->set_x(msg.linear_acceleration.x);
    syn_msg.mutable_linear_acceleration()->set_y(msg.linear_acceleration.y);
    syn_msg.mutable_linear_acceleration()->set_z(msg.linear_acceleration.z);
    syn_msg.mutable_angular_velocity()->set_x(msg.angular_velocity.x);
    syn_msg.mutable_angular_velocity()->set_y(msg.angular_velocity.y);
    syn_msg.mutable_angular_velocity()->set_z(msg.angular_velocity.z);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_WHEEL_ODOMETRY);
    frame.set_allocated_imu(&syn_msg);
    udp_send(frame);
    frame.release_imu();
}

void SynapseRos::wheel_odometry_callback(const sensor_msgs::msg::JointState& msg) const
{
    // construct empty syn_msg
    synapse_pb::WheelOdometry syn_msg {};

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    // construct message
    int n_wheels = msg.position.size();
    double rotation = 0;
    for (int i = 0; i < n_wheels; i++) {
        rotation += msg.position[i];
    }
    syn_msg.set_rotation(rotation);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_WHEEL_ODOMETRY);
    frame.set_allocated_wheel_odometry(&syn_msg);
    udp_send(frame);
    frame.release_wheel_odometry();
}

void SynapseRos::clock_offset_callback(const builtin_interfaces::msg::Time& msg) const
{
    // construct empty syn_msg
    synapse_pb::Time syn_msg {};

    syn_msg.set_sec(msg.sec);
    syn_msg.set_nanosec(msg.nanosec);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_CLOCK_OFFSET);
    frame.set_allocated_time(&syn_msg);
    udp_send(frame);
    frame.release_time();
}

void SynapseRos::battery_state_callback(const sensor_msgs::msg::BatteryState& msg) const
{
    // construct empty syn_msg
    synapse_pb::BatteryState syn_msg {};

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    syn_msg.set_voltage(msg.voltage);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_BATTERY_STATE);
    frame.set_allocated_battery_state(&syn_msg);
    udp_send(frame);
    frame.release_battery_state();
}

void SynapseRos::magnetic_field_callback(const sensor_msgs::msg::MagneticField& msg) const
{
    // construct empty syn_msg
    synapse_pb::MagneticField syn_msg {};

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    syn_msg.mutable_magnetic_field()->set_x(msg.magnetic_field.x);
    syn_msg.mutable_magnetic_field()->set_y(msg.magnetic_field.y);
    syn_msg.mutable_magnetic_field()->set_z(msg.magnetic_field.z);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_NAV_SAT_FIX);
    frame.set_allocated_magnetic_field(&syn_msg);
    udp_send(frame);
    frame.release_magnetic_field();
}

void SynapseRos::nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix& msg) const
{
    // construct empty syn_msg
    synapse_pb::NavSatFix syn_msg {};

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    syn_msg.set_latitude(msg.latitude);
    syn_msg.set_longitude(msg.longitude);
    syn_msg.set_altitude(msg.altitude);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_NAV_SAT_FIX);
    frame.set_allocated_nav_sat_fix(&syn_msg);
    udp_send(frame);
    frame.release_nav_sat_fix();
}

void SynapseRos::udp_send(const synapse_pb::Frame& frame) const
{
    std::stringstream stream;
    if (!SerializeDelimitedToOstream(frame, &stream)) {
        std::cerr << "Failed to serialize " << frame.topic() << std::endl;
        return;
    }
    if (g_udp_link != nullptr) {
        g_udp_link.get()->write((const uint8_t*)stream.str().c_str(), stream.str().length());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynapseRos>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
