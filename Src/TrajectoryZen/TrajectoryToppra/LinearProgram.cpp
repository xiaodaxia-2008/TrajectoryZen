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

namespace tz
{
// bool SolveLP2DWithSeidelAlgorithm(const Eigen::Vector2d &v,
//                                   const Eigen::VectorXd &a,
//                                   const Eigen::VectorXd &b,
//                                   const Eigen::VectorXd &c,
//                                   Eigen::Vector2d &opt_vars,
//                                   const Eigen::Vector2d &low_,
//                                   const Eigen::Vector2d &high_)
// {
//     constexpr double eps = std::numeric_limits<double>::epsilon();
//     constexpr double inf = std::numeric_limits<double>::infinity();

//     auto low = low_;
//     auto high = high_;
//     int n = a.size(); // constrains number
//     if (spdlog::should_log(spdlog::level::debug)) {
//         printf("%8s %8s %8s\n", "a", "b", "c");
//         for (int k = 0; k < n; k++)
//             printf("%3d: [%6.5lf, %6.5lf, %6.5lf],\n", k, a[k], b[k], c[k]);
//         printf("%8s %8s\n", "low", "high");
//         for (int k = 0; k < 2; k++)
//             printf("[%6.5lf, %6.5lf]\n", low[k], high[k]);
//     }
//     bool skip_constrain_idx[n]; // constrains index of which a == 0 or b == 0
//     for (int i = 0; i < n; i++) {
//         skip_constrain_idx[i] = false;
//         if (std::abs(a[i]) <= eps) {
//             skip_constrain_idx[i] = true;
//             if (b[i] > eps)
//                 high[1] = std::min(high[1], c[i] / b[i]);
//             else if (b[i] < -eps)
//                 low[1] = std::max(low[1], c[i] / b[i]);
//             else if (c[i] < -eps) // 0.0 + 0.0 <= -eps, infeasible
//             {
//                 SPDLOG_DEBUG(
//                     "a:{}, b: {}, c: {}, unsatisfied contrain: 0.0 < -eps",
//                     a[i], b[i], c[i]);
//                 return false;
//             }
//         } else if (std::abs(b[i]) <= eps) {
//             skip_constrain_idx[i] = true;
//             if (a[i] > eps)
//                 high[0] = std::min(high[0], c[i] / a[i]);
//             else if (a[i] < -eps)
//                 low[0] = std::max(low[0], c[i] / a[i]);
//             else if (c[i] < -eps) // 0.0 + 0.0 <= -eps, infeasible
//             {
//                 SPDLOG_DEBUG(
//                     "a:{}, b: {}, c: {}, unsatisfied contrain: 0.0 < -eps",
//                     a[i], b[i], c[i]);
//                 return false;
//             }
//         }
//     }
//     // assign initial variable values
//     for (int i = 0; i < 2; i++) opt_vars[i] = v[i] > 0.0 ? low[i] : high[i];
//     // select a random constrain start idx
//     int k = rand() % n;
//     for (int ii = k; ii < n + k; ii++) {
//         int i = ii % n;
//         if (skip_constrain_idx[i]) continue;
//         double a1 = a[i], b1 = b[i], c1 = c[i];
//         // current opt vars satisfies the i-th contrains, continue
//         if (a1 * opt_vars[0] + b1 * opt_vars[1] <= c1) continue;
//         // otherwise, project all constraints on the line defined by(a[i],
//         b[i],
//         // c[i])
//         double x0_right = inf, x1_right = inf;
//         double x0_left = -inf, x1_left = -inf;
//         for (int jj = k; jj < ii + 4; jj++) {
//             double a2 = 0.0, b2 = 0.0, c2 = 0.0;
//             if (jj < ii) {
//                 int j = jj % n;
//                 a2 = a[j];
//                 b2 = b[j];
//                 c2 = c[j];
//             } else if (jj == ii) {
//                 a2 = 1.0;
//                 b2 = 0.0;
//                 c2 = high[0];
//             } else if (jj == ii + 1) {
//                 a2 = 0.0;
//                 b2 = 1.0;
//                 c2 = high[1];
//             } else if (jj == ii + 2) {
//                 a2 = -1.0;
//                 b2 = 0.0;
//                 c2 = -low[0];
//             } else if (jj == ii + 3) {
//                 a2 = 0.0;
//                 b2 = -1.0;
//                 c2 = -low[1];
//             }
//             // calculate the intersection point
//             double parel = a1 * b2 - a2 * b1;
//             if (std::abs(parel) < eps) {
//                 // two parallel lines
//                 continue;
//             }
//             double x0 = (c1 * b2 - c2 * b1) / parel;
//             // double x1 = (c2 * a1 - c1 * a2) / parel;
//             // which direction is feasible
//             double x0_lower = x0 - 0.1;
//             double x1_lower = (c1 - a1 * x0_lower) / b1;
//             if (a2 * x0_lower + b2 * x1_lower <= c2) { // upbound of x0
//                 x0_right = std::min(x0_right, x0);
//             } else {
//                 x0_left = std::max(x0_left, x0);
//             }
//         }
//         if (x0_right - x0_left < -eps) {
//             SPDLOG_DEBUG("new constrains unfeasible, x0_left: {}, x0_right:
//             {}",
//                          x0_left, x0_right);
//             return false; // unfeasible
//         }
//         x1_right = std::abs(b1) > eps ? (c1 - a1 * x0_right) / b1 :
//         opt_vars[1]; x1_left = std::abs(b1) > eps ? (c1 - a1 * x0_left) / b1
//         : opt_vars[1]; if (std::abs(a1 * v[1] - b1 * v[0]) < eps) {
//             SPDLOG_DEBUG("Optimal variables are on an edge: [({:6.5lf}, "
//                          "{:6.5lf}), ({:6.5lf}, {:6.5lf})]",
//                          x0_left, x1_left, x0_right, x1_right);
//         }
//         if (v[0] * x0_right + v[1] * x1_right
//             <= v[0] * x0_left + v[1] * x1_left) {
//             opt_vars[0] = x0_right;
//             opt_vars[1] = x1_right;
//         } else {
//             opt_vars[0] = x0_left;
//             opt_vars[1] = x1_left;
//         }
//     }
//     return true;
// }

namespace lp
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
} // namespace lp
} // namespace tz