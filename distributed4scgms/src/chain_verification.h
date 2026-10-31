#pragma once

#include <scgms/rtl/SolverLib.h>
#include <pagmo/problem.hpp>

#include <cstddef>

#include "CRemap.h"

namespace scgms_distributed_solver
{
    // Any non-empty value runs the check before the evolution starts.
    constexpr const char* Env_Verify = "SCGMS_DISTRIBUTED_VERIFY";

    /**
     * Evaluates the same candidates through the UDP that is sent to the workers and through
     * TSolver_Setup::objective. Both run in the calling process; no worker is involved.
     *
     * Covers the UDP's chain reconstruction only, not serialization or transport.
     *
     * @param sampleCount candidates taken at even steps between the remapped bounds
     * @return the largest absolute difference, or NaN if either side failed to evaluate
     */
    double Verify_Chain_Problem(const solver::TSolver_Setup& setup, const CRemap& remap,
                                const pagmo::problem& problem, size_t sampleCount);
}
