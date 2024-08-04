/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file LinearProgram2DSeidelSolver.hpp
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 16:01:57, August 03, 2024
 */
#include "LinearProgram.h"

#include <spdlog/spdlog.h>

#include <numeric>
#include <random>
#include <algorithm>
#include <array>


namespace tz::lp
{

Solution1D LP1D(double a, const std::vector<Constraint1D> &constraints)
{
    constexpr auto inf = std::numeric_limits<double>::infinity();
    if (a > 0) {
        Solution1D sol{.status = Status::OPTIMAL, .x = -inf};
        for (const auto &constraint : constraints) {
            if (constraint.a < 0) {
                sol.x = std::max(sol.x, constraint.b / constraint.a);
            }
        }
        return sol;
    } else if (a < 0) {
        Solution1D sol{.status = Status::OPTIMAL, .x = inf};
        for (const auto &constraint : constraints) {
            if (constraint.a > 0) {
                sol.x = std::min(sol.x, constraint.b / constraint.a);
            }
        }
        return sol;
    } else {
        return Solution1D{.status = Status::UNBOUNDED};
    }
}

Solution2D LP2D(const Objective2D &objective,
                const std::vector<Constraint2D> &constraints)
{
    constexpr double eps = std::numeric_limits<double>::epsilon();
    constexpr double inf = std::numeric_limits<double>::infinity();

    // std::shuffle(std::begin(constraints), std::end(constraints),
    //              std::mt19937());

    const auto n = constraints.size(); // constrains number
    // assign initial variable values
    Solution2D sol{
        .status = Status::OPTIMAL,
        .x = objective.a > 0 ? -inf : inf,
        .y = objective.b > 0 ? -inf : inf,
    };
    for (size_t i = 0; i < n; ++i) {
        const auto &[a, b, c] = constraints[i];

        // current opt vars satisfies the i-th contrains, continue
        if (a * sol.x + b * sol.y <= c) continue;

        // otherwise, the new opt vars lie on the line ax + by = c
        // replace y with y = (c - ax) / b, convert to 1d LP, we already
        // filtered the conditions that b == 0

        if (a == 0) {
            if (b > 0) {
                sol.y = std::min(sol.y, c / b);
            } else if (b < 0) {
                sol.y = std::max(sol.y, c / b);
            } else if (c < 0) { // 0 * x + 0 * y <= c, c must be >= 0
                SPDLOG_ERROR(
                    "{:.6g} * x + {:.6g} * y <= {:.6g} cannot be satified", a,
                    b, c);
                return Solution2D{.status = Status::INFEASIBLE};
            }
            continue;
        } else if (b == 0) {
            if (a > 0) {
                sol.x = std::min(sol.x, c / a);
            } else if (a < 0) {
                sol.x = std::max(sol.x, c / a);
            } else if (c < 0) {
                SPDLOG_ERROR(
                    "{:.6g} * x + {:.6g} * y <= {:.6g} cannot be satified", a,
                    b, c);
                return Solution2D{.status = Status::INFEASIBLE};
            }
            continue;
        } else {
            auto e = objective.a - objective.b * a / b;
            std::vector<Constraint1D> constraints_1d;
            for (size_t j = 0; j <= i; ++j) {
                auto [aj, bj, cj] = constraints[j];
                aj -= bj * a / b;
                cj -= bj * c / b;
                constraints_1d.emplace_back(aj, cj);
            }
            auto sol1d = LP1D(e, constraints_1d);
            if (sol1d.status == Status::OPTIMAL) {
                sol.x = sol1d.x;
                sol.y = (c - a * sol1d.x) / b;
            } else {
                SPDLOG_ERROR("1d LP failed");
                return Solution2D{.status = sol1d.status};
            }
        }
    }

    return sol;
}
} // namespace tz::lp
