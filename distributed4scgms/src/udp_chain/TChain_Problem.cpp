#include "TChain_Problem.h"

#include "../chain_dataset.h"

#include <scgms/rtl/referencedImpl.h>

#include <condition_variable>
#include <limits>

//#############################################################################################
//# TChain_Event
//#############################################################################################

TChain_Event TChain_Event::From_Event(const scgms::UDevice_Event& event)
{
    TChain_Event result;

    result.event_code = static_cast<uint32_t>(event.event_code());
    result.device_id = event.device_id();
    result.signal_id = event.signal_id();
    result.device_time = event.device_time();
    result.segment_id = event.segment_id();

    if (event.is_level_event())
    {
        result.level = event.level();
    }
    else if (event.is_parameters_event() && event.parameters)
    {
        double* begin = nullptr;
        double* end = nullptr;
        if (event.parameters->get(&begin, &end) == S_OK)
        {
            result.parameters.assign(begin, end);
            result.has_parameters = true;
        }
    }

    return result;
}

scgms::UDevice_Event TChain_Event::To_Event() const
{
    scgms::UDevice_Event event{static_cast<scgms::NDevice_Event_Code>(event_code)};

    event.device_id() = device_id;
    event.signal_id() = signal_id;
    event.device_time() = device_time;
    event.segment_id() = segment_id;

    if (has_parameters)
    {
        // Fresh vector per event; filters may modify the parameters in place.
        event.parameters = refcnt::Create_Container_shared<double, scgms::SModel_Parameter_Vector>(
            const_cast<double*>(parameters.data()),
            const_cast<double*>(parameters.data()) + parameters.size());
    }
    else if (event.is_level_event())
    {
        event.level() = level;
    }

    return event;
}

namespace
{
    // Terminal filter of the rebuilt chain, signalling arrival of Shut_Down. Not reference
    // counted: instances live on the stack and must not be deleted by the chain. Modelled on
    // CTerminal_Filter in core/scgms.
    class CTerminal_Sink : public virtual scgms::IFilter, public virtual refcnt::CNotReferenced
    {
        std::mutex mGuard{};
        std::condition_variable mShutdown{};
        bool mShutdown_Received = false;

    public:
        HRESULT IfaceCalling Configure(scgms::IFilter_Configuration*, refcnt::wstr_list*) noexcept override
        {
            return S_OK;
        }

        HRESULT IfaceCalling Execute(scgms::IDevice_Event* event) noexcept override
        {
            if (!event)
            {
                return E_INVALIDARG;
            }

            scgms::TDevice_Event* raw = nullptr;
            const HRESULT rc = event->Raw(&raw);
            if (rc == S_OK && raw->event_code == scgms::NDevice_Event_Code::Shut_Down)
            {
                std::unique_lock<std::mutex> lock{mGuard};
                mShutdown_Received = true;
                mShutdown.notify_all();
            }

            event->Release();   //we are the last one, nobody else will
            return rc;
        }

        void Wait_For_Shutdown()
        {
            std::unique_lock<std::mutex> lock{mGuard};
            mShutdown.wait(lock, [this] { return mShutdown_Received; });
        }
    };

    // Collects the metrics promised by the signal-error filters of the chain.
    class CMetric_Harvester
    {
        solver::TFitness mMetric = solver::Nan_Fitness;
        size_t mCount = 0;

    public:
        HRESULT On_Filter_Created(scgms::IFilter* filter)
        {
            if (mCount < mMetric.size())
            {
                scgms::SSignal_Error_Inspection inspection{scgms::SFilter{filter}};
                if (inspection)
                {
                    if (inspection->Promise_Metric(scgms::All_Segments_Id, &mMetric[mCount], true) != S_OK)
                    {
                        return E_FAIL;
                    }

                    mCount++;
                }
            }

            return S_OK;
        }

        // Reversed to highest priority first, matching CError_Metric_Future.
        size_t Get_Metric(double* const fitness) const
        {
            for (size_t i = 0; i < mCount; i++)
            {
                fitness[i] = mMetric[mCount - i - 1];
            }

            return mCount;
        }
    };

    HRESULT IfaceCalling On_Filter_Created_Wrapper(scgms::IFilter* filter, const void* data)
    {
        return static_cast<CMetric_Harvester*>(const_cast<void*>(data))->On_Filter_Created(filter);
    }
}

//#############################################################################################
//# TChain_Problem
//#############################################################################################

TChain_Problem::TChain_Problem() : mRemap(solver::Default_Solver_Setup)
{
}

TChain_Problem::TChain_Problem(const TChain_Problem& other) : udp_base(other),
                                                              mRemap(other.mRemap),
                                                              mParams(other.mParams),
                                                              mBody_Ini(other.mBody_Ini),
                                                              mEvents(other.mEvents),
                                                              mDataset_Key(other.mDataset_Key),
                                                              mPatch_Targets(other.mPatch_Targets)
{
    // The pool is not copied; its entries belong to the process that built them.
}

TChain_Problem& TChain_Problem::operator=(const TChain_Problem& other)
{
    if (this == &other)
    {
        return *this;
    }

    udp_base::operator=(other);
    mRemap = other.mRemap;
    mParams = other.mParams;
    mBody_Ini = other.mBody_Ini;
    mEvents = other.mEvents;
    mDataset_Key = other.mDataset_Key;
    mPatch_Targets = other.mPatch_Targets;

    return *this;
}

void TChain_Problem::Set_Body(std::string bodyIni, std::vector<TChain_Event> events,
                              std::vector<TChain_Patch_Target> patchTargets)
{
    mBody_Ini = std::move(bodyIni);
    mEvents = std::move(events);
    mPatch_Targets = std::move(patchTargets);
}

void TChain_Problem::Set_Params(const TUdpParamsSerializable& params)
{
    mParams = params;
    mRemap = CRemap{mParams.lower_bound_vec, mParams.upper_bound_vec};
}

void TChain_Problem::Set_From(const solver::TOptimizing_Problem& problem, std::string datasetKey)
{
    mDataset_Key = std::move(datasetKey);

    if (problem.body_ini && problem.body_ini_len > 0)
    {
        mBody_Ini.assign(problem.body_ini, problem.body_ini_len);
    }

    mEvents.clear();
    mEvents.reserve(problem.event_count);
    for (size_t i = 0; i < problem.event_count; i++)
    {
        const auto& source = problem.events[i];

        TChain_Event event;
        event.event_code = source.event_code;
        event.device_id = source.device_id;
        event.signal_id = source.signal_id;
        event.device_time = source.device_time;
        event.segment_id = source.segment_id;
        event.level = source.level;

        if (source.parameters && source.parameter_count > 0)
        {
            event.parameters.assign(source.parameters, source.parameters + source.parameter_count);
            event.has_parameters = true;
        }

        mEvents.push_back(std::move(event));
    }

    mPatch_Targets.clear();
    mPatch_Targets.reserve(problem.parameter_count);
    for (size_t i = 0; i < problem.parameter_count; i++)
    {
        const auto& source = problem.parameters[i];

        TChain_Patch_Target target;
        target.filter_index = source.filter_index;
        target.parameter_name = source.config_name ? source.config_name : L"";
        target.offset = source.offset;
        target.count = source.count;

        mPatch_Targets.push_back(std::move(target));
    }
}

scgms::SPersistent_Filter_Chain_Configuration TChain_Problem::Pop_Configuration(const double* solution) const
{
    // A default-constructed SPersistent_Filter_Chain_Configuration is non-null, so a pool hit
    // cannot be detected by testing the pointer.
    scgms::SPersistent_Filter_Chain_Configuration configuration;
    bool from_pool = false;

    {
        std::lock_guard<std::mutex> lock{mPool_Guard};
        if (!mPool.empty())
        {
            configuration = std::move(mPool.top());
            mPool.pop();
            from_pool = true;
        }
    }

    if (!from_pool)
    {
        if (!configuration)
        {
            return {};
        }

        refcnt::Swstr_list errors;
        if (configuration->Load_From_Memory(mBody_Ini.data(), mBody_Ini.size(), errors.get()) != S_OK)
        {
            configuration.reset();
            return configuration;
        }
    }

    // Apply the candidate to the optimized filters.
    for (const auto& target : mPatch_Targets)
    {
        auto link = configuration[target.filter_index];
        auto parameter = link ? link.Resolve_Parameter(target.parameter_name.c_str()) : scgms::SFilter_Parameter{};

        scgms::IModel_Parameter_Vector* parameters = nullptr;
        if (!parameter || parameter->Get_Model_Parameters(&parameters) != S_OK)
        {
            configuration.reset();
            return configuration;
        }

        double* begin = nullptr;
        double* end = nullptr;
        const bool ok = parameters->get(&begin, &end) == S_OK;
        if (ok)
        {
            // The vector holds lower/default/upper; the candidate replaces the middle third.
            std::copy(solution + target.offset, solution + target.offset + target.count, begin + target.count);
        }

        parameters->Release();

        if (!ok)
        {
            configuration.reset();
            return configuration;
        }
    }

    return configuration;
}

void TChain_Problem::Push_Configuration(scgms::SPersistent_Filter_Chain_Configuration configuration) const
{
    if (configuration)
    {
        std::lock_guard<std::mutex> lock{mPool_Guard};
        mPool.push(std::move(configuration));
    }
}

pagmo::vector_double TChain_Problem::fitness(const pagmo::vector_double& x) const
{
    const auto solution = mRemap.Expand_Solution(x);

    pagmo::vector_double result(solver::Maximum_Objectives_Count, std::numeric_limits<double>::quiet_NaN());

    auto configuration = Pop_Configuration(solution.data());
    if (!configuration)
    {
        result.resize(mParams.objectives_count);
        return result;
    }

    CMetric_Harvester harvester;
    size_t harvested = 0;

    {
        CTerminal_Sink sink;
        refcnt::Swstr_list errors;

        // Scoped: the signal-error filter writes the promised metric in its destructor, so the
        // executor must be released before Get_Metric.
        scgms::SFilter_Executor executor{configuration, On_Filter_Created_Wrapper, &harvester, errors, &sink};
        if (executor)
        {
            bool replayed_shutdown = false;

            for (const auto& event : mEvents)
            {
                replayed_shutdown = event.event_code == static_cast<uint32_t>(scgms::NDevice_Event_Code::Shut_Down);

                if (!Succeeded(executor.Execute(event.To_Event())))
                {
                    replayed_shutdown = false;
                    break;
                }
            }

            // Captured events normally end with Shut_Down. A second one issued after the chain
            // has closed never reaches the sink, so only wait when this call sent it.
            if (!replayed_shutdown)
            {
                executor.Execute(scgms::UDevice_Event{scgms::NDevice_Event_Code::Shut_Down});
                sink.Wait_For_Shutdown();
            }
        }
    }

    harvested = harvester.Get_Metric(result.data());

    Push_Configuration(std::move(configuration));

    if (harvested != mParams.objectives_count)
    {
        std::fill(result.begin(), result.end(), std::numeric_limits<double>::quiet_NaN());
    }

    result.resize(mParams.objectives_count);
    return result;
}

std::pair<pagmo::vector_double, pagmo::vector_double> TChain_Problem::get_bounds() const
{
    return mRemap.get_bounds();
}

pagmo::vector_double::size_type TChain_Problem::get_nobj() const
{
    return mParams.objectives_count;
}

std::string TChain_Problem::get_name() const
{
    return "SmartCGMS filter chain";
}

std::string TChain_Problem::required_blob_key() const
{
    return mDataset_Key;
}

bool TChain_Problem::has_required_blob() const
{
    return mDataset_Key.empty() || !mEvents.empty();
}

void TChain_Problem::accept_blob(const std::vector<std::byte>& blob)
{
    std::vector<TChain_Event> decoded;

    const bool ok = chain_dataset::Decode(blob,
        [&decoded](const chain_dataset::TEvent_Header& header, std::vector<double> parameters)
        {
            TChain_Event event;
            event.event_code = header.event_code;
            event.device_id = header.device_id;
            event.signal_id = header.signal_id;
            event.device_time = header.device_time;
            event.segment_id = header.segment_id;
            event.level = header.level;
            event.has_parameters = !parameters.empty();
            event.parameters = std::move(parameters);

            decoded.push_back(std::move(event));
        });

    if (!ok)
    {
        throw std::runtime_error("Data set " + mDataset_Key + " is truncated or has an unexpected format");
    }

    mEvents = std::move(decoded);
}

std::string TChain_Problem::get_lib_file_name()
{
    return "tchain_udp";
}

//#############################################################################################
//# Extern C lib functions
//#############################################################################################

void run_after_load()
{
}

TChain_Problem* allocator(const std::any& params)
{
    auto* problem = new TChain_Problem();

    if (params.has_value())
    {
        try
        {
            const auto& casted = std::any_cast<TUdpParams>(params);
            problem->Set_Params(casted.params);

            // The adapter puts chain_dataset::TChain_Udp_Data here; see distributed.h.
            if (casted.data)
            {
                const auto* chainData = static_cast<const chain_dataset::TChain_Udp_Data*>(casted.data);
                if (chainData->problem)
                {
                    problem->Set_From(*chainData->problem,
                                      chainData->dataset_key ? chainData->dataset_key : "");
                }
            }
        }
        catch (...)
        {
            delete problem;
            throw std::runtime_error("TChain_Problem UDP only accepts TUdpParams as its parameter.");
        }
    }

    return problem;
}

void deleter(TChain_Problem* ptr)
{
    delete ptr;
}

TChain_Problem* cloner(const TChain_Problem* other)
{
    if (!other)
    {
        return nullptr;
    }

    return new TChain_Problem(*other);
}

BOOST_CLASS_EXPORT_IMPLEMENT(TChain_Problem)
