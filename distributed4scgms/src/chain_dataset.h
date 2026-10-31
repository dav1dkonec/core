#pragma once

#include <scgms/iface/SolverIface.h>

#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

/**
 * Encoding of the replayed events shared by the adapter and the chain UDP.
 *
 * The events are identical for every work unit, so they are not serialized into the UDP.
 * The adapter encodes them once, registers them as a blob, and the UDP carries only the key;
 * see udp_base::required_blob_key().
 *
 * Fixed little-endian-agnostic layout is not attempted: both ends are built from this header
 * and run the same encoding, and a mixed-endian cluster is out of scope.
 */
namespace chain_dataset
{
    // Bumped whenever the layout below changes; a worker built against another value rejects it.
    constexpr uint32_t Format_Version = 1;

    struct TEvent_Header
    {
        uint32_t event_code;
        GUID device_id;
        GUID signal_id;
        double device_time;
        uint64_t segment_id;
        double level;
        uint64_t parameter_count;   // fixed width, the count is followed by that many doubles
    };

    inline void Append(std::vector<std::byte>& out, const void* data, const size_t size)
    {
        const auto* bytes = static_cast<const std::byte*>(data);
        out.insert(out.end(), bytes, bytes + size);
    }

    inline std::vector<std::byte> Encode(const solver::TOptimizing_Event* events, const size_t count)
    {
        std::vector<std::byte> out;
        out.reserve(count * sizeof(TEvent_Header) + 2 * sizeof(uint64_t));

        const uint32_t version = Format_Version;
        const uint64_t eventCount = count;
        Append(out, &version, sizeof(version));
        Append(out, &eventCount, sizeof(eventCount));

        for (size_t i = 0; i < count; i++)
        {
            const auto& source = events[i];

            TEvent_Header header{};
            header.event_code = source.event_code;
            header.device_id = source.device_id;
            header.signal_id = source.signal_id;
            header.device_time = source.device_time;
            header.segment_id = source.segment_id;
            header.level = source.level;
            header.parameter_count = source.parameters ? source.parameter_count : 0;

            Append(out, &header, sizeof(header));
            if (header.parameter_count > 0)
            {
                Append(out, source.parameters, header.parameter_count * sizeof(double));
            }
        }

        return out;
    }

    /**
     * @return false if the blob is truncated or was written by another format version
     */
    template <typename TSink>
    bool Decode(const std::vector<std::byte>& blob, TSink&& sink)
    {
        size_t offset = 0;

        const auto read = [&blob, &offset](void* target, const size_t size)
        {
            if (offset + size > blob.size())
            {
                return false;
            }

            std::memcpy(target, blob.data() + offset, size);
            offset += size;
            return true;
        };

        uint32_t version = 0;
        uint64_t eventCount = 0;
        if (!read(&version, sizeof(version)) || version != Format_Version || !read(&eventCount, sizeof(eventCount)))
        {
            return false;
        }

        for (uint64_t i = 0; i < eventCount; i++)
        {
            TEvent_Header header{};
            if (!read(&header, sizeof(header)))
            {
                return false;
            }

            std::vector<double> parameters(static_cast<size_t>(header.parameter_count));
            if (header.parameter_count > 0 && !read(parameters.data(), parameters.size() * sizeof(double)))
            {
                return false;
            }

            sink(header, std::move(parameters));
        }

        return true;
    }

    /**
     * What the adapter puts into TUdpParams::data for the chain UDP: the problem description
     * plus the key under which its events were registered.
     */
    struct TChain_Udp_Data
    {
        const solver::TOptimizing_Problem* problem;
        const char* dataset_key;
    };
}
