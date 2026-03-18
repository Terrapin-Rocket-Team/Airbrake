#include "PacketStreams.h"

void PacketStreams::addStream(Stream &stream)
{
    if (streamCount >= kMaxStreams)
        return;

    streams[streamCount++] = &stream;
}

bool PacketStreams::send(const uint8_t *data, size_t len)
{
    if (data == nullptr || len == 0)
        return false;

    bool ok = true;
    for (size_t i = 0; i < streamCount; ++i)
    {
        Stream *stream = streams[i];
        if (stream == nullptr)
            continue;

        ok = ok && (stream->write(data, len) == len);
    }

    return ok;
}

bool PacketStreams::send(const avionics_packet::PacketBuffer &packet)
{
    return send(packet.data, packet.size);
}
