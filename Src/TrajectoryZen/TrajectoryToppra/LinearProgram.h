/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file LinearProgram2DSeidelSolver.hpp
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 16:01:57, August 03, 2024
 */

#pragma once
#include <TrajectoryZen/trajectoryzen_export.h>
#include <vector>

namespace tz
{
namespace lp
{
///@brief minimize ax + by
struct TRAJECTORYZEN_EXPORT Objective2D {
    double a, b;
};

/// @brief ax <= b
struct TRAJECTORYZEN_EXPORT Constraint1D {
    double a, b;
};

/// @brief ax + by <= c
struct TRAJECTORYZEN_EXPORT Constraint2D {
    double a, b, c;
};

enum class TRAJECTORYZEN_EXPORT Status { OPTIMAL, INFEASIBLE, UNBOUNDED };

struct TRAJECTORYZEN_EXPORT Solution1D {
    Status status = Status::INFEASIBLE;
    double x;
};

struct TRAJECTORYZEN_EXPORT Solution2D {
    Status status = Status::INFEASIBLE;
    double x, y;
};

/**
 * @brief Minimize e * x, s.t. ax <= b
 */
TRAJECTORYZEN_EXPORT Solution1D
LP1D(double a, const std::vector<Constraint1D> &constraints);

/**
 * @brief See algorithm details at
 * https://www.cs.cmu.edu/~avrim/451/lectures/lect1021-lpII.pdf
 * and https://homepages.math.uic.edu/~jan/mcs481/linearprogramming2d.pdf
 */
TRAJECTORYZEN_EXPORT Solution2D LP2D(
    const Objective2D &objective, const std::vector<Constraint2D> &constraints);


} // namespace lp
} // namespace tz
