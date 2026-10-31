#include "chain_verification.h"

#include "logger_init.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scgms_distributed_solver
{
    namespace
    {
        // Same value on both sides counts as agreement, and so does NaN on both.
        bool Same_Fitness(const double a, const double b)
        {
            if (std::isnan(a) && std::isnan(b))
            {
                return true;
            }

            return a == b;
        }
    }

    double Verify_Chain_Problem(const solver::TSolver_Setup& setup, const CRemap& remap,
                                const pagmo::problem& problem, const size_t sampleCount)
    {
        if (!setup.objective || sampleCount == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        const auto [lower, upper] = remap.get_bounds();
        const size_t objectives = std::min(setup.objectives_count, solver::Maximum_Objectives_Count);

        double worstDifference = 0.0;
        size_t mismatches = 0;

        for (size_t sample = 0; sample < sampleCount; sample++)
        {
            // Deterministic sweep from the lower bound to the upper one, so a failure is
            // reproducible and can be re-run with the same numbers.
            const double t = (sampleCount == 1) ? 0.5 : static_cast<double>(sample) / static_cast<double>(sampleCount - 1);

            pagmo::vector_double candidate(lower.size());
            for (size_t i = 0; i < lower.size(); i++)
            {
                candidate[i] = lower[i] + t * (upper[i] - lower[i]);
            }

            pagmo::vector_double remote;
            try
            {
                remote = problem.fitness(candidate);
            }
            catch (const std::exception& e)
            {
                dist_logger()->error("Verification sample {}: the worker's problem threw: {}", sample, e.what());
                return std::numeric_limits<double>::quiet_NaN();
            }

            const auto expanded = remap.Expand_Solution(candidate);
            solver::TFitness local = solver::Nan_Fitness;
            if (!setup.objective(setup.data, 1, expanded.data(), local.data()))
            {
                dist_logger()->error("Verification sample {}: the local objective failed", sample);
                return std::numeric_limits<double>::quiet_NaN();
            }

            for (size_t i = 0; i < objectives && i < remote.size(); i++)
            {
                if (Same_Fitness(remote[i], local[i]))
                {
                    continue;
                }

                const double difference = std::abs(remote[i] - local[i]);
                worstDifference = std::max(worstDifference, difference);
                mismatches++;

                dist_logger()->error("Verification sample {}, objective {}: worker {} vs local {} (difference {})",
                                     sample, i, remote[i], local[i], difference);
            }
        }

        if (mismatches == 0)
        {
            dist_logger()->info("Verification passed: {} samples agree with the local objective", sampleCount);
        }
        else
        {
            dist_logger()->error("Verification FAILED: {} mismatches, worst difference {}", mismatches, worstDifference);
        }

        return worstDifference;
    }
}
