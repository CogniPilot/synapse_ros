#include "rpmsg_link.hpp"

#include <iostream>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <thread>
#include <sys/poll.h>

RPMsgLink::RPMsgLink(const std::string &rpmsg_device) 
{
    f = open(rpmsg_device.c_str(), O_RDWR);
    if (f < 0) {
        std::cerr << "cannot open rpmsg device: " << rpmsg_device << std::endl;
        return;
    }

    // Send dummy message to let zephyr know our channel ID
    uint8_t dummy_message[] = {0xaa};
    ::write(f, dummy_message, sizeof(dummy_message));
}

RPMsgLink::~RPMsgLink()
{
    if (f >= 0) {
        close(f);
    }
}

void RPMsgLink::run_for(std::chrono::seconds sec)
{
    (void)sec; // don't care about the seconds to run..

    struct pollfd fds[1] = {f, POLLIN, 0};

    auto ret = poll(fds, 1, 100);
    if (ret < 0) {
        std::cerr << "error reading bytes from rpmsg device: poll error " << ret << std::endl;
        return;
    }

    if (ret == 0) {
        // timeout so continue polling..
        return;
    }

    auto read_size = read(f, rx_buf_, rx_buf_length_);
    if (read_size < 0) {
        std::cerr << "error reading bytes from rpmsg device: read error " << read_size << std::endl;
        return;
    }

    parse_protobuf_message(rx_buf_, read_size);
}

void RPMsgLink::write(const uint8_t* buf, uint32_t len)
{
    if (f < 0) {
        return;
    }

    auto written_size = ::write(f, buf, len);
    if (written_size != len) {
        std::cerr << "error sending bytes to rpmsg device: " << written_size << std::endl;
    }
}

// vi: ts=4 sw=4 et
