#pragma once

#include <scgms/rtl/SolverLib.h>
#include <pagmo/problem.hpp>

#include <cstddef>

#include "CRemap.h"

namespace scgms_distributed_solver
{
    // Set this to any non-empty value to run the check before the evolution starts.
    constexpr const char* Env_Verify = "SCGMS_DISTRIBUTED_VERIFY";

    /**
     * Evaluates the same candidates twice: through the problem the workers will receive, and
     * through the objective function of this process. The two must agree, otherwise the worker
     * rebuilds the filter chain differently than the optimizer runs it.
     *
     * Meant to be run once on a small task before trusting a long one; a mismatch on 288
     * parameters is almost impossible to locate afterwards.
     *
     * @return the largest absolute difference found, or NaN if either side could not evaluate
     */
    double Verify_Chain_Problem(const solver::TSolver_Setup& setup, const CRemap& remap,
                                const pagmo::problem& problem, size_t sampleCount);
}
