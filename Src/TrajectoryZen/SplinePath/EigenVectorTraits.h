/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file EigenVectorTraits.h
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 15:46:23, August 04, 2024
 */
#pragma once
#include "../Base/PointTypeTraits.h"
#include <Eigen/Dense>

namespace tz
{
template <>
struct PointTypeTraits<Eigen::VectorXd> {
    using TangentT = Eigen::VectorXd;
    using CurvatureT = Eigen::VectorXd;
};

} // namespace tz
