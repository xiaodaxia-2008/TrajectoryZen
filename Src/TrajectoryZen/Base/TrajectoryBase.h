/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file TrajectoryBase.h
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 14:37:03, August 04, 2024
 */
#pragma once
#include "PointTypeTraits.h"
#include <TrajectoryZen/trajectoryzen_export.h>

namespace tz
{

template <typename PointT_>
class TRAJECTORYZEN_EXPORT TrajectoryBase
{
public:
    using PointT = PointT_;
    using TangentT = PointTypeTraits<PointT>::TangentT;
    using CurvatureT = PointTypeTraits<PointT>::CurvatureT;

    virtual ~TrajectoryBase() = default;

    virtual double GetDuration() const = 0;
    virtual PointT EvalPoint(double t) const = 0;
    virtual TangentT EvalTangent(double t) const = 0;
    virtual CurvatureT EvalCurvature(double t) const = 0;
};
} // namespace tz