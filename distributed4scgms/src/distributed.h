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

#include <scgms/rtl/SolverLib.h>

#include <algorithm>
#include <vector>
#include <numeric>
#include <limits>
#include <chrono>
#include <iostream>

#include <pagmo/types.hpp>
#include <pagmo/problem.hpp>

#include "CRemap.h"
#include "distributed_solver.h"
#include "TUdpParams.h"
#include "udp_dll_wrapper.h"
#include "udp_registry.h"
#include "pagmo/algorithms/de.hpp"
#include "pagmo/algorithms/gaco.hpp"
#include "scgms/iface/DistributedSolverIface.h"
#include "mo_benchmark.h"
#include "so_benchmark.h"

namespace scgms_distributed_solver
{
    // How long to wait for the expected workers before giving up.
    constexpr std::chrono::seconds Worker_Connect_Timeout{60};

    //#############################################################################################
    //# CDistributed_Solver - SCGMS adapter
    //#############################################################################################
    class CDistributed_Solver
    {
    protected:
        solver::TSolver_Setup mSetup;
        CRemap mRemap;
        solver::TDistributedSolver_Data* mSolverData;

    public:
        CDistributed_Solver(const solver::TSolver_Setup& setup)
            : mSetup(solver::Check_Default_Parameters(setup, 100'000, 100)),
              mRemap(setup)
        {
            // We assume that the user provides us with the right data object
            mSolverData = reinterpret_cast<solver::TDistributedSolver_Data*>(const_cast<void*>(mSetup.data));
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
            const std::string libName = solveChain ? "tchain_udp" : mSolverData->solver_lib_name;
            const std::string controllerAddress = mSolverData->controller_address;
            const size_t expectedWorkerCount = mSolverData->expected_worker_count;
            const void* originalData = mSolverData->solverData;

            if (!mSolverData->local_cache_dir.empty())
                udp_registry::get().set_local_cache_dir(mSolverData->local_cache_dir);

            progress = solver::Null_Solver_Progress;
            bool succeeded = false;

            // 2) Construct a pagmo UDP using our TProblem wrapper
            //####################################################
            // Maybe set up UDP registry?
            const void* udpData = solveChain ? static_cast<const void*>(mSetup.optimizing_problem) : originalData;
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

            // Without this wait the islands are handed to whoever happens to be connected,
            // which forces the even split and leaves the capability-based plan unused.
            if (!distSolver.wait_until_workers_connect(Worker_Connect_Timeout))
            {
                const size_t connected = distSolver.get_current_worker_count();
                if (connected == 0)
                {
                    const std::string msg = "No worker connected within "
                        + std::to_string(Worker_Connect_Timeout.count())
                        + " s, cannot run the distributed solver";
                    spdlog::error("{}", msg);
                    std::cerr << msg << std::endl;
                    return false;
                }

                spdlog::warn("Only {} of {} workers connected, continuing with those",
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
