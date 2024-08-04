/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file Path.h
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 14:21:53, August 04, 2024
 */
#pragma once
#include "PointTypeTraits.h"
#include <TrajectoryZen/trajectoryzen_export.h>

#include <Eigen/Dense>

namespace tz
{

template <typename PointT_>
class TRAJECTORYZEN_EXPORT PathBase
{
public:
    /// @brief point type
    using PointT = PointT_;
    /// @brief first order derivative
    using TangentT = PointTypeTraits<PointT>::TangentT;
    /// @brief second order derivative
    using CurvatureT = PointTypeTraits<PointT>::CurvatureT;

    virtual ~PathBase() = default;
    virtual double GetLength() const = 0;
    virtual PointT EvalPoint(double s) const = 0;
    virtual TangentT EvalTangent(double s) const = 0;
    virtual CurvatureT EvalCurvature(double s) const = 0;

protected:
};

} // namespace tz
