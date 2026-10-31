#include "chain_verification.h"

#include "logger_init.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace scgms_distributed_solver
{
    namespace
    {
        // Equal values, or NaN on both sides.
        bool Same_Fitness(const double a, const double b)
        {
            if (std::isnan(a) && std::isnan(b))
            {
                return true;
            }

            return a == b;
        }

        size_t Compare(const char* what, const size_t sample, const size_t objectives,
                       const pagmo::vector_double& candidate, const solver::TFitness& reference,
                       double& worstDifference)
        {
            size_t mismatches = 0;

            for (size_t i = 0; i < objectives && i < candidate.size(); i++)
            {
                if (Same_Fitness(candidate[i], reference[i]))
                {
                    continue;
                }

                const double difference = std::abs(candidate[i] - reference[i]);
                worstDifference = std::max(worstDifference, difference);
                mismatches++;

                dist_logger()->error("Verification sample {}, objective {}: {} {} vs in-process objective {} (difference {})",
                                     sample, i, what, candidate[i], reference[i], difference);
            }

            return mismatches;
        }
    }

    double Verify_Chain_Problem(const solver::TSolver_Setup& setup, const CRemap& remap,
                                const pagmo::problem& problem, const size_t sampleCount)
    {
        if (!setup.objective || sampleCount == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        // The same archive the work container travels in, so this covers what the workers receive.
        pagmo::problem restored;
        size_t archiveSize = 0;
        try
        {
            std::stringstream buffer;
            {
                boost::archive::binary_oarchive out{buffer};
                out << problem;
            }

            archiveSize = static_cast<size_t>(buffer.tellp());

            boost::archive::binary_iarchive in{buffer};
            in >> restored;
        }
        catch (const std::exception& e)
        {
            dist_logger()->error("Verification: the problem failed to survive serialization: {}", e.what());
            return std::numeric_limits<double>::quiet_NaN();
        }

        const auto [lower, upper] = remap.get_bounds();
        const size_t objectives = std::min(setup.objectives_count, solver::Maximum_Objectives_Count);

        double worstDifference = 0.0;
        size_t mismatches = 0;
        std::chrono::steady_clock::duration shippedTime{}, inProcessTime{};

        for (size_t sample = 0; sample < sampleCount; sample++)
        {
            // Even steps between the bounds, so a failing sample can be reproduced.
            const double t = (sampleCount == 1) ? 0.5 : static_cast<double>(sample) / static_cast<double>(sampleCount - 1);

            pagmo::vector_double point(lower.size());
            for (size_t i = 0; i < lower.size(); i++)
            {
                point[i] = lower[i] + t * (upper[i] - lower[i]);
            }

            pagmo::vector_double shipped, deserialized;
            const auto shippedStart = std::chrono::steady_clock::now();
            try
            {
                shipped = problem.fitness(point);
                deserialized = restored.fitness(point);
            }
            catch (const std::exception& e)
            {
                dist_logger()->error("Verification sample {}: the chain problem threw: {}", sample, e.what());
                return std::numeric_limits<double>::quiet_NaN();
            }
            shippedTime += std::chrono::steady_clock::now() - shippedStart;

            const auto expanded = remap.Expand_Solution(point);
            const auto inProcessStart = std::chrono::steady_clock::now();
            solver::TFitness inProcess = solver::Nan_Fitness;
            if (!setup.objective(setup.data, 1, expanded.data(), inProcess.data()))
            {
                dist_logger()->error("Verification sample {}: the in-process objective failed", sample);
                return std::numeric_limits<double>::quiet_NaN();
            }
            inProcessTime += std::chrono::steady_clock::now() - inProcessStart;

            mismatches += Compare("chain problem", sample, objectives, shipped, inProcess, worstDifference);
            mismatches += Compare("deserialized chain problem", sample, objectives, deserialized, inProcess, worstDifference);
        }

        const auto ms = [](const std::chrono::steady_clock::duration d)
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
        };

        if (mismatches == 0)
        {
            dist_logger()->info("Verification passed: {} samples agree before and after serialization "
                                "({} B archive); one fitness takes {} ms shipped, {} ms in-process",
                                sampleCount, archiveSize, ms(shippedTime) / (2 * sampleCount),
                                ms(inProcessTime) / sampleCount);
        }
        else
        {
            dist_logger()->error("Verification FAILED: {} mismatches, worst difference {}", mismatches, worstDifference);
        }

        return worstDifference;
    }
}
