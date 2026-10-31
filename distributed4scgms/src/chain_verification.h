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
     * Evaluates the same candidates twice, both here in this process: once through the problem
     * object that will be shipped to the workers, and once through TSolver_Setup::objective,
     * which is what the optimizer would use without distribution. No worker is involved, which
     * is exactly why the two results can be compared at all.
     *
     * They must agree, otherwise the shipped problem rebuilds the filter chain differently than
     * the optimizer runs it. It does not cover serialization or the network; it checks that the
     * problem itself is faithful.
     *
     * Meant to be run once on a small task before trusting a long one; a mismatch on 288
     * parameters is almost impossible to locate afterwards.
     *
     * @return the largest absolute difference found, or NaN if either side could not evaluate
     */
    double Verify_Chain_Problem(const solver::TSolver_Setup& setup, const CRemap& remap,
                                const pagmo::problem& problem, size_t sampleCount);
}
