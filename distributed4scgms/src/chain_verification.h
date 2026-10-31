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
     * Evaluates the same candidates three ways in the calling process: through the UDP that is
     * sent to the workers, through a copy of it put through the same Boost archive the work
     * container uses, and through TSolver_Setup::objective. No worker is involved.
     *
     * Covers chain reconstruction and serialization, not transport.
     *
     * @param sampleCount candidates taken at even steps between the remapped bounds
     * @return the largest absolute difference, or NaN if either side failed to evaluate
     */
    double Verify_Chain_Problem(const solver::TSolver_Setup& setup, const CRemap& remap,
                                const pagmo::problem& problem, size_t sampleCount);
}
