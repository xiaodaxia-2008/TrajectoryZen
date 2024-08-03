/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file LinearProgram2DSeidelSolver.hpp
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 16:01:57, August 03, 2024
 */

#pragma once
#include <vector>

namespace tz
{

namespace lp2d
{
///@brief minimize ax + by
struct Objective {
    double a, b;
};

/// @brief ax + by <= c
struct Constraint {
    double a, b, c;
};

enum class Status { OPTIMAL, INFEASIBLE, UNBOUNDED };

struct Solution {
    Status status = Status::INFEASIBLE;
    double x, y;
};

/**
 * @brief See algorithm details at
 * https://www.cs.cmu.edu/~avrim/451/lectures/lect1021-lpII.pdf
 */
Solution SeidelAlgorithm(const Objective &objective,
                         const std::vector<Constraint> &constraints);
}; // namespace lp2d
} // namespace tz
