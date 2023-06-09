#include "tcp_client.hpp"
#include "../synapse_ros.hpp"

#include "synapse_protobuf/actuators.pb.h"
#include "synapse_protobuf/joy.pb.h"
#include "synapse_protobuf/odometry.pb.h"
#include "synapse_protobuf/twist.pb.h"

#include "synapse_tinyframe/SynapseTopics.h"
#include <boost/asio/detail/socket_option.hpp>
#include <boost/asio/error.hpp>
#include <boost/date_time/posix_time/posix_time_config.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/system/error_code.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

static void write_tcp(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    // get tcp client attached to tf pointer in userdata
    TcpClient* tcp_client = (TcpClient*)tf->userdata;

    // write buffer to tcp client
    tcp_client->write(buf, len);
}

TcpClient::TcpClient(std::string host, int port, const std::shared_ptr<TinyFrame>& tf)
    : host_(host)
    , port_(port)
{
    // Set socket options
    std::cout << "creating option" << std::endl;
    sockfd_.open(boost::asio::ip::tcp::v4());
    if (!sockfd_.is_open()) {
        std::cerr << "failed to open socket" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_KEEPALIVE> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keep alive" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_KEEPIDLE> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keepidle" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_KEEPCNT> { 3 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keepcnt" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_KEEPINTVL> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keepintvl" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_SYNCNT> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set user syncnt" << std::endl;
    }

    // Set up the TinyFrame library
    tf_ = tf;
    tf_->usertag = 0;
    tf_->userdata = this;
    tf_->write = write_tcp;
    TF_AddGenericListener(tf_.get(), TcpClient::genericListener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_OUT_CMD_VEL_TOPIC, TcpClient::out_cmd_vel_Listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_OUT_ACTUATORS_TOPIC, TcpClient::actuatorsListener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_OUT_ODOMETRY_TOPIC, TcpClient::odometryListener);
    timer_.async_wait(std::bind(&TcpClient::tick, this, _1));
}

void TcpClient::handle_connect(
    const boost::system::error_code& ec,
    const boost::asio::ip::tcp::endpoint& endpoint)
{
    if (ec.failed()) {
        connected_ = false;
        std::cerr << "failed to connect: " << ec.message() << std::endl;
        if (sockfd_.is_open()) {
            sockfd_.close();
        }
    } else {
        connected_ = true;
        std::cout << "tcp connected: " << endpoint << std::endl;
    }
}

void TcpClient::tick(const boost::system::error_code& /*e*/)
{
    if (connected_) {
        sockfd_.async_receive(boost::asio::buffer(rx_buf_, rx_buf_length_),
            std::bind(&TcpClient::rx_handler, this, _1, _2));
    } else {
        std::cout << "tcp connecting to: " << host_ << ":" << port_ << std::endl;
        boost::asio::async_connect(
            sockfd_,
            resolver_.resolve(host_, std::to_string(port_)),
            std::bind(&TcpClient::handle_connect, this, _1, _2));
    }

    boost::posix_time::time_duration wait;
    if (connected_) {
        wait = boost::posix_time::milliseconds(100);
    } else {
        wait = boost::posix_time::seconds(1);
    }

    timer_.expires_at(timer_.expires_at() + wait);
    timer_.async_wait(std::bind(&TcpClient::tick, this, _1));
}

void TcpClient::tx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    (void)bytes_transferred;
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
        connected_ = false;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
        connected_ = false;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "tx error: " << ec.message() << std::endl;
    }
}

void TcpClient::rx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
        connected_ = false;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
        connected_ = false;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "rx error: " << ec.message() << std::endl;
    } else if (ec == boost::system::errc::success) {
        const std::lock_guard<std::mutex> lock(guard_rx_buf_);
        TF_Accept(tf_.get(), rx_buf_, bytes_transferred);
    }
}

TF_Result TcpClient::actuatorsListener(TinyFrame* tf, TF_Msg* frame)
{
    // parse protobuf message
    synapse::msgs::Actuators msg;
    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse actuators" << std::endl;
        return TF_STAY;
    }

    // send to ros
    TcpClient* tcp_client = (TcpClient*)tf->userdata;
    if (tcp_client->ros_ != NULL) {
        tcp_client->ros_->publish_actuators(msg);
    }
    return TF_STAY;
}

TF_Result TcpClient::odometryListener(TinyFrame* tf, TF_Msg* frame)
{
    // parse protobuf message
    synapse::msgs::Odometry syn_msg;
    if (!syn_msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse odometry" << std::endl;
        return TF_STAY;
    }

    // build ros msg
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = syn_msg.header().frame_id();
    msg.header.stamp.sec = syn_msg.header().stamp().sec();
    msg.header.stamp.nanosec = syn_msg.header().stamp().nanosec();
    msg.child_frame_id = syn_msg.child_frame_id();
    msg.pose.pose.position.x = syn_msg.pose().pose().position().x();
    msg.pose.pose.position.y = syn_msg.pose().pose().position().y();
    msg.pose.pose.position.z = syn_msg.pose().pose().position().z();
    msg.pose.pose.orientation.w = syn_msg.pose().pose().orientation().w();
    msg.pose.pose.orientation.x = syn_msg.pose().pose().orientation().x();
    msg.pose.pose.orientation.y = syn_msg.pose().pose().orientation().y();
    msg.pose.pose.orientation.z = syn_msg.pose().pose().orientation().z();

    // send to ros
    TcpClient* tcp_client = (TcpClient*)tf->userdata;
    if (tcp_client->ros_ != NULL) {
        tcp_client->ros_->publish_odometry(syn_msg);
    }
    return TF_STAY;
}

TF_Result TcpClient::out_cmd_vel_Listener(TinyFrame* tf, TF_Msg* frame)
{
    (void)tf;
    synapse::msgs::Twist msg;
    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to out_cmd_vel" << std::endl;
        return TF_STAY;
    } else {
        std::cout << "out cmd vel"
                  << msg.linear().x()
                  << msg.linear().y()
                  << msg.linear().z()
                  << msg.angular().x()
                  << msg.angular().y()
                  << msg.angular().z()
                  << std::endl;
    }
    return TF_STAY;
}

TF_Result TcpClient::genericListener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    int type = msg->type;
    std::cout << "generic listener id:" << type << std::endl;
    dumpFrameInfo(msg);
    return TF_STAY;
}

void TcpClient::run_for(std::chrono::seconds sec)
{
    io_context_.run_for(std::chrono::seconds(sec));
}

void TcpClient::write(const uint8_t* buf, uint32_t len)
{
    if (connected_) {
        boost::asio::async_write(sockfd_, boost::asio::buffer(buf, len),
            std::bind(&TcpClient::tx_handler, this, _1, _2));
    }
}

// vi: ts=4 sw=4 et
