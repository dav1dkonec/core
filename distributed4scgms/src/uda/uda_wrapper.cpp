#include "uda_wrapper.h"

uda_wrapper::uda_wrapper(const GUID& solver_id, const size_t max_generations, const size_t population_size): m_solver_id(solver_id),
    m_max_generations(max_generations),
    m_population_size(population_size)
{
}

pagmo::population uda_wrapper::evolve(pagmo::population pop) const
{
    if (pop.size() == 0)
        return pop;

    auto& prob = pop.get_problem();
    const auto [lb, ub] = prob.get_bounds();
    const size_t problem_size = lb.size();

    // Output buffer
    std::vector<double> solution = pop.champion_x();

    // Build hints for SCGMS solver
    const auto& dvs = pop.get_x();
    std::vector<const double*> hint_ptrs;
    hint_ptrs.reserve(dvs.size());
    for (const auto& dv : dvs)
        hint_ptrs.push_back(dv.data());

    // Context passed to the SCGMS objective function as the data parameter
    struct TCtx
    {
        pagmo::problem* prob;
        size_t problem_size;
    };
    TCtx ctx{&prob, problem_size};

    // Objective function - SCGMS calls this with batches of population individuals (solutions)
    auto objective = [](const void* data, const size_t count, const double* solutions,
                        double* const fitness) -> BOOL
    {
        const auto* ctx = reinterpret_cast<const TCtx*>(data);

        for (size_t i = 0; i < count; ++i)
        {
            const pagmo::vector_double x(solutions + i * ctx->problem_size,
                                         solutions + i * ctx->problem_size + ctx->problem_size);

            const pagmo::vector_double f = ctx->prob->fitness(x);

            double* slot = fitness + i * solver::Maximum_Objectives_Count;
            for (size_t j = 0; j < f.size(); ++j)
                slot[j] = f[j];
        }
        return TRUE;
    };

    solver::TSolver_Setup setup{
        problem_size, // number of parameters
        prob.get_nobj(), // number of objectives
        lb.data(), // lower bounds
        ub.data(), // upper bounds
        hint_ptrs.data(), // hints
        hint_ptrs.size(), // number of hints
        solution.data(), // output written here
        &ctx, // data for objective (see above)
        objective, // objective function (see above)
        nullptr, // no comparator
        m_max_generations,
        m_population_size,
        std::numeric_limits<double>::min()
    };

    // Solve
    solver::TSolver_Progress progress{ 0 };
    const HRESULT hr = solver::Solve_Generic(m_solver_id, setup, progress);
    if (!Succeeded(hr))
        throw std::runtime_error("solver::Solve_Generic failed");

    const pagmo::vector_double best_f = prob.fitness(solution);

    // Replace the worst individual - pagmo algorithms don't grow the population
    const auto worst_idx = pop.worst_idx();
    if (best_f[0] < pop.get_f()[worst_idx][0])
        pop.set_xf(worst_idx, solution, best_f);

    return pop;
}

std::string uda_wrapper::get_name() const
{
    return "scgms_solver_uda";
}

std::string uda_wrapper::get_extra_info() const
{
    return "Wrapper for solver::Solve_Generic to be used as UDA on Pagmo problems\n"
        "max_generations : " + std::to_string(m_max_generations) + "\n"
        "population_size : " + std::to_string(m_population_size);
}
