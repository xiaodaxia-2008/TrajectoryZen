/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file LinearProgram2DSeidelSolver.hpp
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 16:01:57, August 03, 2024
 */
#include "LinearProgram2DSeidelSolver.h"

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

namespace lp2d
{
Solution SeidelAlgorithm(const Objective &objective,
                         const std::vector<Constraint> &constraints)
{
    constexpr double eps = std::numeric_limits<double>::epsilon();
    constexpr double inf = std::numeric_limits<double>::infinity();
    auto near_zero = [](double v) { return std::abs(v) <= eps; };

    std::shuffle(std::begin(constraints), std::end(constraints),
                 std::mt19937());

    std::array<double, 2> low{-inf, -inf};
    std::array<double, 2> high{inf, inf};
    const auto n = constraints.size(); // constrains number
    if (spdlog::should_log(spdlog::level::debug)) {
        printf("%8s %8s %8s\n", "a", "b", "c");
        for (int k = 0; k < n; k++)
            printf("%3d: [%6.5lf, %6.5lf, %6.5lf],\n", k, constraints[k].a,
                   constraints[k].b, constraints[k].c);
        printf("%8s %8s\n", "low", "high");
        for (int k = 0; k < 2; k++)
            printf("[%6.5lf, %6.5lf]\n", low[k], high[k]);
    }
    bool skip_constrain_idx[n]; // constrains index of which a == 0 or b == 0
    for (size_t i = 0; i < n; i++) {
        skip_constrain_idx[i] = false;
        const auto &[a, b, c] = constraints[i];
        if (near_zero(a)) {
            skip_constrain_idx[i] = true;
            if (b > eps)
                high[1] = std::min(high[1], c / b);
            else if (b < -eps)
                low[1] = std::max(low[1], c / b);
            else if (c < -eps) { // 0*&x + 0*y <= -eps, infeasible
                SPDLOG_DEBUG(
                    "{:.6g} * x + {:.6g} * y <= {:.6g} cannot be satisfied", a,
                    b, c);
                return Solution{.status = Status::INFEASIBLE};
            }
        } else if ( near_zero(b)) {
            skip_constrain_idx[i] = true;
            if (a > eps)
                high[0] = std::min(high[0], c/ a);
            else if (a < -eps)
                low[0] = std::max(low[0], c / a);
            else if (c < -eps) { // 0.0 + 0.0 <= -eps, infeasible
                SPDLOG_DEBUG(
                    "{:.6g} * x + {:.6g} * y <= {:.6g} cannot be satisfied", a,
                    b, c);
                return Solution{.status = Status::INFEASIBLE};
            }
        }
    }

    // assign initial variable values
    Solution solution;
    solution.x = objective.a > 0 ? low[0] : high[0];
    solution.y = objective.b > 0 ? low[1] : high[1];
    // select a random constrain start idx
    int k = rand() % n;
    for (int ii = k; ii < n + k; ii++) {
        int i = ii % n;
        if (skip_constrain_idx[i]) continue;
        const auto& [a1, b1, c1] = constraints[i];
        // current opt vars satisfies the i-th contrains, continue
        if (a1 * solution.x + b1 * solution.y <= c1) continue;
        // otherwise, project all constraints on the line defined by(a[i], b[i],
        // c[i])
        double x0_right = inf, x1_right = inf;
        double x0_left = -inf, x1_left = -inf;
        for (int jj = k; jj < ii + 4; jj++) {
            double a2 = 0.0, b2 = 0.0, c2 = 0.0;
            if (jj < ii) {
                int j = jj % n;
                std::tie(a2, b2, c2) = constraints[j];
            } else if (jj == ii) {
                a2 = 1.0;
                b2 = 0.0;
                c2 = high[0];
            } else if (jj == ii + 1) {
                a2 = 0.0;
                b2 = 1.0;
                c2 = high[1];
            } else if (jj == ii + 2) {
                a2 = -1.0;
                b2 = 0.0;
                c2 = -low[0];
            } else if (jj == ii + 3) {
                a2 = 0.0;
                b2 = -1.0;
                c2 = -low[1];
            }
            // calculate the intersection point
            double parel = a1 * b2 - a2 * b1;
            if (std::abs(parel) < eps) {
                // two parallel lines
                continue;
            }
            double x0 = (c1 * b2 - c2 * b1) / parel;
            // double x1 = (c2 * a1 - c1 * a2) / parel;
            // which direction is feasible
            double x0_lower = x0 - 0.1;
            double x1_lower = (c1 - a1 * x0_lower) / b1;
            if (a2 * x0_lower + b2 * x1_lower <= c2) { // upbound of x0
                x0_right = std::min(x0_right, x0);
            } else {
                x0_left = std::max(x0_left, x0);
            }
        }
        if (x0_right - x0_left < -eps) {
            SPDLOG_DEBUG("new constrains unfeasible, x0_left: {}, x0_right: {}",
                         x0_left, x0_right);
            return Solution{.status = Status::INFEASIBLE}; // unfeasible
        }
        x1_right = std::abs(b1) > eps ? (c1 - a1 * x0_right) / b1 : solution.y;
        x1_left = std::abs(b1) > eps ? (c1 - a1 * x0_left) / b1 : solution.y;
        if (std::abs(a1 * objective.b - b1 * objective.a) < eps) {
            SPDLOG_DEBUG("Optimal variables are on an edge: [({:6.5lf}, "
                         "{:6.5lf}), ({:6.5lf}, {:6.5lf})]",
                         x0_left, x1_left, x0_right, x1_right);
        }
        if (v[0] * x0_right + v[1] * x1_right
            <= v[0] * x0_left + v[1] * x1_left) {
            solution.x = x0_right;
            solution.y = x1_right;
        } else {
            solution.x = x0_left;
            solution.y = x1_left;
        }
    }

    return solution;
}
} // namespace lp2d
} // namespace tz