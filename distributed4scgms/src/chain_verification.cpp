#include "chain_verification.h"

#include "logger_init.h"

#include <algorithm>
#include <cmath>
#include <chrono>
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
        std::chrono::steady_clock::duration shippedTime{}, inProcessTime{};

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

            pagmo::vector_double shipped;
            const auto shippedStart = std::chrono::steady_clock::now();
            try
            {
                shipped = problem.fitness(candidate);
            }
            catch (const std::exception& e)
            {
                dist_logger()->error("Verification sample {}: the chain problem threw: {}", sample, e.what());
                return std::numeric_limits<double>::quiet_NaN();
            }

            shippedTime += std::chrono::steady_clock::now() - shippedStart;

            const auto expanded = remap.Expand_Solution(candidate);
            const auto inProcessStart = std::chrono::steady_clock::now();
            solver::TFitness inProcess = solver::Nan_Fitness;
            if (!setup.objective(setup.data, 1, expanded.data(), inProcess.data()))
            {
                dist_logger()->error("Verification sample {}: the in-process objective failed", sample);
                return std::numeric_limits<double>::quiet_NaN();
            }
            inProcessTime += std::chrono::steady_clock::now() - inProcessStart;

            for (size_t i = 0; i < objectives && i < shipped.size(); i++)
            {
                if (Same_Fitness(shipped[i], inProcess[i]))
                {
                    continue;
                }

                const double difference = std::abs(shipped[i] - inProcess[i]);
                worstDifference = std::max(worstDifference, difference);
                mismatches++;

                dist_logger()->error("Verification sample {}, objective {}: chain problem {} vs in-process objective {} (difference {})",
                                     sample, i, shipped[i], inProcess[i], difference);
            }
        }

        if (mismatches == 0)
        {
            const auto ms = [](const std::chrono::steady_clock::duration d)
            {
                return std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
            };

            dist_logger()->info("Verification passed: {} samples agree; one fitness takes {} ms shipped, {} ms in-process",
                                sampleCount, ms(shippedTime) / sampleCount, ms(inProcessTime) / sampleCount);
        }
        else
        {
            dist_logger()->error("Verification FAILED: {} mismatches, worst difference {}", mismatches, worstDifference);
        }

        return worstDifference;
    }
}
