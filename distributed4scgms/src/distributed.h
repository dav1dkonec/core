/**
 * SmartCGMS - continuous glucose monitoring and controlling framework
 * https://diabetes.zcu.cz/
 *
 * Copyright (c) since 2018 University of West Bohemia.
 *
 * Contact:
 * diabetes@mail.kiv.zcu.cz
 * Medical Informatics, Department of Computer Science and Engineering
 * Faculty of Applied Sciences, University of West Bohemia
 * Univerzitni 8, 301 00 Pilsen
 * Czech Republic
 * 
 * 
 * Purpose of this software:
 * This software is intended to demonstrate work of the diabetes.zcu.cz research
 * group to other scientists, to complement our published papers. It is strictly
 * prohibited to use this software for diagnosis or treatment of any medical condition,
 * without obtaining all required approvals from respective regulatory bodies.
 *
 * Especially, a diabetic patient is warned that unauthorized use of this software
 * may result into severe injure, including death.
 *
 *
 * Licensing terms:
 * Unless required by applicable law or agreed to in writing, software
 * distributed under these license terms is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * a) Without a specific agreement, you are not authorized to make or keep any copies of this file.
 * b) For any use, especially commercial use, you must contact us and obtain specific terms and conditions 
 *    for the use of the software.
 * c) When publishing any derivative work or results obtained using this software, you agree to cite the following paper:
 *    Tomas Koutny and Martin Ubl, "SmartCGMS as a Testbed for a Blood-Glucose Level Prediction and/or 
 *    Control Challenge with (an FDA-Accepted) Diabetic Patient Simulation", Procedia Computer Science,  
 *    Volume 177, pp. 354-362, 2020
 */

#pragma once

#include <scgms/rtl/FilesystemLib.h>
#include <scgms/rtl/SolverLib.h>
#include <scgms/utils/string_utils.h>

#include <algorithm>
#include <vector>
#include <numeric>
#include <limits>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <format>
#include <iostream>
#include <optional>

#include <pagmo/types.hpp>
#include <pagmo/problem.hpp>

#include "CRemap.h"
#include "blob_registry.h"
#include "deployment_settings.h"
#include "descriptor.h"
#include "chain_dataset.h"
#include "chain_verification.h"
#include "distributed_solver.h"
#include "TUdpParams.h"
#include "udp_dll_wrapper.h"
#include "udp_registry.h"
#include "pagmo/algorithms/de.hpp"
#include "pagmo/algorithms/gaco.hpp"
#include "mo_benchmark.h"
#include "so_benchmark.h"

namespace scgms_distributed_solver
{
    // How long to wait for the expected workers before giving up, unless configured otherwise.
    constexpr std::chrono::milliseconds Default_Worker_Connect_Timeout{60'000};

    // Deployment settings. Read through Read_Env; Solve() supplies a fallback for each, so
    // none of them is required.
    constexpr const char* Env_Controller_Address = "SCGMS_DISTRIBUTED_CONTROLLER";
    constexpr const char* Env_Worker_Count = "SCGMS_DISTRIBUTED_WORKERS";
    constexpr const char* Env_Udp_Library = "SCGMS_DISTRIBUTED_UDP_LIB";
    constexpr const char* Env_Cache_Dir = "SCGMS_DISTRIBUTED_CACHE_DIR";

    inline std::string Read_Env(const char* name)
    {
#ifdef _MSC_VER
        char* buffer = nullptr;
        size_t length = 0;
        if (_dupenv_s(&buffer, &length, name) != 0 || !buffer)
            return {};

        std::string value{buffer};
        free(buffer);
        return value;
#else
        const char* value = std::getenv(name);
        return value ? std::string{value} : std::string{};
#endif
    }

    inline std::string Read_Env(const char* name, const std::string& fallback)
    {
        const std::string value = Read_Env(name);
        return value.empty() ? fallback : value;
    }

    inline size_t Read_Env(const char* name, const size_t fallback)
    {
        const std::string value = Read_Env(name);
        if (value.empty())
            return fallback;

        try
        {
            return static_cast<size_t>(std::stoull(value));
        }
        catch (...)
        {
            dist_logger()->warn("{} is not a number: {}; using {}", name, value, fallback);
            return fallback;
        }
    }

    // A resolved deployment value together with where it came from, so the log can name the source.
    template <typename T>
    struct TResolved
    {
        T value{};
        const char* source = "default";
    };

    // Resolution order is the configuration, then the environment variable, then the fallback.
    inline TResolved<std::string> Resolve(const std::optional<std::string>& configured,
                                          const char* envName, std::string fallback)
    {
        if (configured)
        {
            return {*configured, "ini"};
        }

        std::string fromEnv = Read_Env(envName);
        if (!fromEnv.empty())
        {
            return {std::move(fromEnv), "env"};
        }

        return {std::move(fallback), "default"};
    }

    inline TResolved<size_t> Resolve(const std::optional<size_t>& configured, const char* envName,
                                     const size_t fallback)
    {
        if (configured)
        {
            return {*configured, "ini"};
        }

        if (Read_Env(envName).empty())
        {
            return {fallback, "default"};
        }

        // Read again through the overload that parses and warns about a malformed value.
        return {Read_Env(envName, fallback), "env"};
    }

    //#############################################################################################
    //# CDistributed_Solver - SCGMS adapter
    //#############################################################################################
    class CDistributed_Solver
    {
    protected:
        solver::TSolver_Setup mSetup;
        CRemap mRemap;

    public:
        CDistributed_Solver(const solver::TSolver_Setup& setup)
            : mSetup(solver::Check_Default_Parameters(setup, 100'000, 100)),
              mRemap(setup)
        {
        }

        bool Solve(solver::TSolver_Progress& progress)
        {
            // 1) Initialize key variables
            //####################################################
            const size_t popSize = mSetup.population_size;
            const size_t generationCount = mSetup.max_generations;
            const size_t numOfObjectives = mSetup.objectives_count;
            // A chain problem is evaluated by the worker itself, so it needs a different library
            // than the self-contained benchmarks; see TSolver_Setup::optimizing_problem.
            const bool solveChain = mSetup.optimizing_problem != nullptr;
            const std::string libName = solveChain
                                            ? std::string{"tchain_udp"}
                                            : Read_Env(Env_Udp_Library, std::string{"tproblem_udp"});
            const void* originalData = mSetup.data;

            // Empty unless a settings filter was configured in this process; see deployment_settings.
            const auto configured = deployment_settings::Get();
            const auto configuredValues = configured.value_or(deployment_settings::TSettings{});

            const auto address = Resolve(configuredValues.controller_address, Env_Controller_Address,
                                         std::string{"tcp://0.0.0.0:5555"});
            const auto workers = Resolve(configuredValues.expected_workers, Env_Worker_Count,
                                         static_cast<size_t>(1));
            const auto cache = Resolve(configuredValues.cache_dir, Env_Cache_Dir, std::string{});
            const std::string& controllerAddress = address.value;
            const size_t expectedWorkerCount = workers.value;

            const auto connectTimeout = configuredValues.worker_connect_timeout_s
                ? std::chrono::milliseconds{std::llround(*configuredValues.worker_connect_timeout_s * 1000.0)}
                : Default_Worker_Connect_Timeout;

            if (!cache.value.empty())
            {
                udp_registry::get().set_local_cache_dir(cache.value);
            }

            // Prefer a library shipped with this build over fetching one from a peer.
            // The application dir, not Get_Dll_Dir(), because filters live one level deeper.
            udp_registry::get().add_lib_search_dir(Get_Application_Dir());

            progress = solver::Null_Solver_Progress;
            bool succeeded = false;

            // 2) Construct a pagmo UDP using our TProblem wrapper
            //####################################################
            // Maybe set up UDP registry?
            // The replayed events are the same for every work unit, so they are registered once
            // as a blob and the UDP only carries the key; see udp_base::required_blob_key().
            std::string datasetKey;
            if (solveChain)
            {
                auto dataset = chain_dataset::Encode(mSetup.optimizing_problem->events,
                                                     mSetup.optimizing_problem->event_count);
                datasetKey = blob_registry::make_key(dataset);
                blob_registry::get().put(datasetKey, std::move(dataset));
            }

            const chain_dataset::TChain_Udp_Data chainData{mSetup.optimizing_problem, datasetKey.c_str()};
            const void* udpData = solveChain ? static_cast<const void*>(&chainData) : originalData;
            TUdpParams udpParams = {udpData, ExtractSerializable(mSetup)};
            udp_dll_wrapper udp{libName, udpParams};
            const pagmo::problem prob{udp};

            // 3) Construct our Algorithms (MO or SO
            //####################################################
            std::vector<pagmo::algorithm> algos{};
            if (numOfObjectives == 1)
            {
                algos = construct_so_algorithms(generationCount);
            } else
            {
                algos = construct_mo_algorithms(generationCount);
            }

            // 4) Set up distributed solver + hints
            //####################################################
            distributed_solver distSolver{controllerAddress, expectedWorkerCount};

            // Enable logging
            distSolver.enable_logging();

            // Reported only now; before enable_logging() the library logger has no sinks.
            dist_logger()->info("Deployment: address {} ({}), expected workers {} ({}), cache dir {}"
                                " ({}), connect timeout {} s ({})",
                                controllerAddress, address.source, expectedWorkerCount, workers.source,
                                cache.value.empty() ? std::string{"built-in"} : cache.value, cache.source,
                                connectTimeout.count() / 1000.0,
                                configuredValues.worker_connect_timeout_s ? "ini" : "default");

            if (!configured)
            {
                const std::string msg = "No '" + Narrow_WString(GUID_To_WString(settings_filter_id))
                    + "' filter in the chain, the deployment settings come from the environment and"
                      " built-in defaults";
                dist_logger()->warn("{}", msg);
                std::cerr << msg << std::endl;
            }

            // Must stay after enable_logging(), the verdict goes to the library logger.
            if (solveChain && !Read_Env(Env_Verify).empty())
            {
                Verify_Chain_Problem(mSetup, mRemap, prob, 5);
            }

            // Without this wait the islands are handed to whoever happens to be connected,
            // which forces the even split and leaves the capability-based plan unused.
            if (!distSolver.wait_until_workers_connect(connectTimeout))
            {
                const size_t connected = distSolver.get_current_worker_count();
                if (connected == 0)
                {
                    const std::string msg = std::format(
                        "No worker connected within {} s, cannot run the distributed solver",
                        connectTimeout.count() / 1000.0);
                    dist_logger()->error("{}", msg);
                    std::cerr << msg << std::endl;
                    return false;
                }

                dist_logger()->warn("Only {} of {} workers connected, continuing with those",
                             connected, expectedWorkerCount);
            }

            std::vector<pagmo::vector_double> hints{};
            if (mSetup.hint_count > 0)
            {
                for (size_t i = 0; i < mSetup.hint_count; i++)
                {
                    hints.emplace_back(mRemap.Reduce_Solution(mSetup.hints[i]));
                }
                distSolver.set_initial_hints(hints);
            }

            // 5) Run the distributed evolution
            //####################################################
            distSolver.evolve(prob, algos, popSize); // Maybe set cycleCount?
            // Blocking call
            const auto& bestIndividual = distSolver.wait_until_completion();

            // 6) Set if evolution was successful
            //####################################################
            succeeded = distSolver.get_status() == pagmo::evolve_status::idle;

            // 7) Write back result and return
            //####################################################
            pagmo::vector_double champion_x(mRemap.problem_size(), std::numeric_limits<double>::quiet_NaN());
            champion_x = mRemap.Expand_Solution(bestIndividual);

            if (succeeded)
                std::copy(champion_x.begin(), champion_x.end(), mSetup.solution);

            return succeeded;
        }
    };
} // namespace scgms_distributed_solver
