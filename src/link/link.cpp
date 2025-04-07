#include "link.hpp"
#include "../synapse_ros.hpp"
#include <synapse_pb/frame.pb.h>
#include <google/protobuf/util/delimited_message_util.h>

using namespace google::protobuf::util;

void Link::parse_protobuf_message(const uint8_t* buf, uint32_t len)
{
    static synapse_pb::Frame frame;
    frame.Clear();
    auto stream = google::protobuf::io::CodedInputStream(buf, len);

    while (true) {
        bool clean_eof = true;
        if (!ParseDelimitedFromCodedStream(&frame, &stream, &clean_eof)) {
            if (!clean_eof) {
                std::cerr << "Failed to parse frame: bytes: " << len << std::endl;
            }
            break;
        } else {
            if (frame.msg_case() == synapse_pb::Frame::kActuators) {
                ros_->publish_actuators(frame.actuators());
            } else if (frame.msg_case() == synapse_pb::Frame::kOdometry) {
                ros_->publish_odometry(frame.odometry());
            } else if (frame.msg_case() == synapse_pb::Frame::kNavSatFix) {
                ros_->publish_nav_sat_fix(frame.nav_sat_fix());
            } else if (frame.msg_case() == synapse_pb::Frame::kStatus) {
                ros_->publish_status(frame.status());
            } else if (frame.msg_case() == synapse_pb::Frame::kClockOffset) {
                ros_->publish_uptime(frame.clock_offset());
            } else {
                std::cerr << "unhandled message case" << frame.msg_case() << std::endl;
                break;
            }
        }
    }
}

// vi: ts=4 sw=4 et