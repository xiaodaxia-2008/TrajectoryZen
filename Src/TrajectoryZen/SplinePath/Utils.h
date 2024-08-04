/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file Utils.h
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 15:53:17, August 04, 2024
 */
#pragma once
#include <Eigen/Dense>

#include <ranges>
#include <algorithm>
#include <numeric>

namespace tz
{

std::vector<double> ComputeKnots(const std::vector<Eigen::VectorXd> &points)
{
    auto knots =
        points
        | std::views::adjacent_transform<2>(
            [](const auto &a, const auto &b) { return (b - a).norm(); })
        | std::ranges::to<std::vector>();
    knots.push_back(knots.back());
    std::exclusive_scan(knots.begin(), knots.end(), knots.begin(), 0.0);
    return knots;
}


} // namespace tz
