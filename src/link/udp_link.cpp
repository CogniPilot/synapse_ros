#include <synapse_protobuf/frame.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/twist.pb.h>

#include <boost/asio/error.hpp>
#include <boost/system/error_code.hpp>

#include "../synapse_ros.hpp"
#include "udp_link.hpp"
#include <google/protobuf/util/delimited_message_util.h>

using namespace google::protobuf::util;

using boost::asio::ip::udp;
using std::placeholders::_1;
using std::placeholders::_2;

UDPLink::UDPLink(std::string host, int port)
{
    remote_endpoint_ = *udp::resolver(io_context_).resolve(udp::resolver::query(host, std::to_string(port)));
    my_endpoint_ = udp::endpoint(udp::v4(), 4242);

    // schedule new rx
    sock_.async_receive_from(boost::asio::buffer(rx_buf_, rx_buf_length_),
        my_endpoint_,
        std::bind(&UDPLink::rx_handler, this, _1, _2));
}

void UDPLink::tx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    (void)bytes_transferred;
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "tx error: " << ec.message() << std::endl;
    }
}

void UDPLink::rx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "rx error: " << ec.message() << std::endl;
    } else if (ec == boost::system::errc::success) {
        const std::lock_guard<std::mutex> lock(guard_rx_buf_);

        // parse protobuf message
        static synapse::msgs::Frame frame;
        frame.Clear();
        auto stream = google::protobuf::io::CodedInputStream(rx_buf_, bytes_transferred);
        while (true) {
            bool clean_eof = true;
            if (!ParseDelimitedFromCodedStream(&frame, &stream, &clean_eof)) {
                if (!clean_eof) {
                    std::cerr << "Failed to parse frame: bytes: " << bytes_transferred << std::endl;
                }
                break;
            } else {
                if (frame.msg_case() == synapse::msgs::Frame::kActuators) {
                    if (frame.topic() == synapse::msgs::TOPIC_ACTUATORS) {
                        ros_->publish_actuators(frame.actuators());
                    }
                } else if (frame.msg_case() == synapse::msgs::Frame::kOdometry) {
                    if (frame.topic() == synapse::msgs::TOPIC_ODOMETRY) {
                        ros_->publish_odometry(frame.odometry());
                    }
                } else if (frame.msg_case() == synapse::msgs::Frame::kTwist) {
                    if (frame.topic() == synapse::msgs::TOPIC_CMD_VEL) {
                        ros_->publish_odometry(frame.odometry());
                    }
                } else if (frame.msg_case() == synapse::msgs::Frame::kNavSatFix) {
                    if (frame.topic() == synapse::msgs::TOPIC_NAV_SAT_FIX) {
                        ros_->publish_nav_sat_fix(frame.nav_sat_fix());
                    }
                } else if (frame.msg_case() == synapse::msgs::Frame::kStatus) {
                    if (frame.topic() == synapse::msgs::TOPIC_STATUS) {
                        ros_->publish_status(frame.status());
                    }
                } else if (frame.msg_case() == synapse::msgs::Frame::kTime) {
                    if (frame.topic() == synapse::msgs::TOPIC_UPTIME) {
                        ros_->publish_uptime(frame.time());
                    }
                } else {
                    std::cerr << "unhandled message case" << frame.msg_case() << std::endl;
                    break;
                }
            }
        }
    }

    // schedule new rx
    sock_.async_receive_from(boost::asio::buffer(rx_buf_, rx_buf_length_),
        my_endpoint_,
        std::bind(&UDPLink::rx_handler, this, _1, _2));
}

void UDPLink::run_for(std::chrono::seconds sec)
{
    io_context_.run_for(std::chrono::seconds(sec));
}

void UDPLink::write(const uint8_t* buf, uint32_t len)
{
    sock_.async_send_to(boost::asio::buffer(buf, len),
        remote_endpoint_,
        std::bind(&UDPLink::tx_handler, this, _1, _2));
}

// vi: ts=4 sw=4 et
