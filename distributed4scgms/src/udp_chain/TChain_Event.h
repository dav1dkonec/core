#pragma once

#include <scgms/iface/DeviceIface.h>
#include <scgms/rtl/DeviceLib.h>

#include <cstdint>
#include <vector>

#include <pagmo/s11n.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>

// A device event in a form that survives the trip to a worker.
// Info events are never replayed, so only the level and the parameter vector are carried.
struct TChain_Event
{
    uint32_t event_code{};
    GUID device_id{};
    GUID signal_id{};
    double device_time{};
    uint64_t segment_id{};

    double level{};
    std::vector<double> parameters{};
    bool has_parameters{};

    static TChain_Event From_Event(const scgms::UDevice_Event& event);

    // Builds a fresh event; the caller owns it and feeds it to the chain.
    scgms::UDevice_Event To_Event() const;

private:
    friend class boost::serialization::access;

    // GUID has no serializer of its own; Data4 is a raw array, hence make_array.
    template <typename Archive>
    static void serialize_guid(Archive& ar, GUID& guid)
    {
        ar & guid.Data1;
        ar & guid.Data2;
        ar & guid.Data3;
        ar & boost::serialization::make_array(guid.Data4, sizeof(guid.Data4));
    }

    template <typename Archive>
    void serialize(Archive& ar, const unsigned)
    {
        ar & event_code;
        serialize_guid(ar, device_id);
        serialize_guid(ar, signal_id);
        ar & device_time;
        ar & segment_id;
        ar & level;
        ar & parameters;
        ar & has_parameters;
    }
};
