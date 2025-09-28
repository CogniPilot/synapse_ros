#pragma once

#include <chrono>

class SynapseRos;

class Link {

public:
    SynapseRos* ros_ { NULL };
    virtual void run_for(std::chrono::seconds sec) = 0;
    virtual void write(const uint8_t* buf, uint32_t len) = 0;

protected:
    virtual void parse_protobuf_message(const uint8_t* buf, uint32_t len);
};

// vi: ts=4 sw=4 et
