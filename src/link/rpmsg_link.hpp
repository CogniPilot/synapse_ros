#pragma once

#include "link.hpp"

#include <memory>

namespace std {
class thread;
}

class RPMsgLink : public Link {

public:
    RPMsgLink(const std::string &rpmsg_device);
    ~RPMsgLink();
    void write(const uint8_t* buf, uint32_t len);
    void run_for(std::chrono::seconds sec);

private:
    void rpmsg_read_thread();

    int f;

    static constexpr uint32_t rx_buf_length_ = 1024;
    uint8_t rx_buf_[rx_buf_length_];
};

// vi: ts=4 sw=4 et
