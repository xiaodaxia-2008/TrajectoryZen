/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file CubicSplinePath.cpp
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 15:57:19, August 04, 2024
 */
#include "CubicSplinePath.h"
#include "Utils.h"

namespace tz
{
CubicSpline::CubicSpline(const std::vector<PointT> &points)
{
    m_knots = ComputeKnots(points);
}
} // namespace tz