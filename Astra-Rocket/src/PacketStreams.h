#ifndef PACKET_STREAMS_H
#define PACKET_STREAMS_H

#include <Arduino.h>

#include "AvionicsPacketProtocol.h"

class PacketStreams
{
public:
    void addStream(Stream &stream);
    bool send(const uint8_t *data, size_t len);
    bool send(const avionics_packet::PacketBuffer &packet);

private:
    static constexpr size_t kMaxStreams = 4;

    Stream *streams[kMaxStreams] = {};
    size_t streamCount = 0;
};

#endif // PACKET_STREAMS_H
