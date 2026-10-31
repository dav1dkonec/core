#pragma once

#include "../CRemap.h"
#include "../TUdpParams.h"
#include "TChain_Event.h"

#include "udp_base.h"
#include <scgms/rtl/FilterLib.h>

#include <any>
#include <mutex>
#include <stack>
#include <string>
#include <vector>

#include <pagmo/s11n.hpp>
#include <boost/serialization/export.hpp>

// One filter the worker has to patch before every evaluation.
struct TChain_Patch_Target
{
    // Index of the filter within the optimizing body, not within the whole chain.
    size_t filter_index{};
    std::wstring parameter_name{};
    // Where this filter's slice starts in the candidate vector, and how long it is.
    size_t offset{};
    size_t count{};

private:
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned)
    {
        ar & filter_index;
        ar & parameter_name;
        ar & offset;
        ar & count;
    }
};

//#############################################################################################
//# TChain_Problem - Pagmo-style UDP that evaluates a SmartCGMS filter chain
//#############################################################################################
// Unlike TProblem, this one does not carry a closed-form benchmark. It carries the optimizing
// part of the chain as ini text plus the events the preceding filters already produced, and the
// worker rebuilds the chain locally. That is what makes a remote fitness possible at all: the
// original TSolver_Setup.objective is a pointer into the controller's process.
class TChain_Problem : public udp_base
{
    CRemap mRemap;
    TUdpParamsSerializable mParams{};

    // Optimizing body of the chain, as produced by Save_To_Memory().
    std::string mBody_Ini{};
    // Output of the filters that precede the body. Replayed once per candidate.
    std::vector<TChain_Event> mEvents{};
    std::vector<TChain_Patch_Target> mPatch_Targets{};

    // Chains are expensive to build, so finished ones go back into a pool.
    // Not serialized: it belongs to whichever process is evaluating.
    mutable std::mutex mPool_Guard{};
    mutable std::stack<scgms::SPersistent_Filter_Chain_Configuration> mPool{};

public:
    TChain_Problem();
    TChain_Problem(const TChain_Problem& other);
    TChain_Problem& operator=(const TChain_Problem& other);

    pagmo::vector_double fitness(const pagmo::vector_double& x) const override;
    std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const override;
    pagmo::vector_double::size_type get_nobj() const override;
    std::string get_name() const override;

    // REQUIRED by udp_base
    std::string get_lib_file_name() override;

    // Filled by the controller before the problem is handed to the solver.
    void Set_Body(std::string bodyIni, std::vector<TChain_Event> events,
                  std::vector<TChain_Patch_Target> patchTargets);
    void Set_Params(const TUdpParamsSerializable& params);

private:
    // Takes a configuration from the pool or builds one, then writes the candidate into it.
    scgms::SPersistent_Filter_Chain_Configuration Pop_Configuration(const double* solution) const;
    void Push_Configuration(scgms::SPersistent_Filter_Chain_Configuration configuration) const;

    //####################################
    //# BOOST SERIALIZE
    //####################################
    friend class boost::serialization::access;

    template <typename Archive>
    void save(Archive& ar, unsigned) const
    {
        boost::serialization::void_cast_register<TChain_Problem, udp_base>();

        ar << mRemap;
        ar << mParams;
        ar << mBody_Ini;
        ar << mEvents;
        ar << mPatch_Targets;
    }

    template <typename Archive>
    void load(Archive& ar, unsigned)
    {
        boost::serialization::void_cast_register<TChain_Problem, udp_base>();

        ar >> mRemap;
        ar >> mParams;
        ar >> mBody_Ini;
        ar >> mEvents;
        ar >> mPatch_Targets;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()
};

extern "C" void run_after_load();

extern "C" TChain_Problem* allocator(const std::any& params);

extern "C" void deleter(TChain_Problem* ptr);

extern "C" TChain_Problem* cloner(const TChain_Problem* other);

BOOST_CLASS_EXPORT_KEY(TChain_Problem)
