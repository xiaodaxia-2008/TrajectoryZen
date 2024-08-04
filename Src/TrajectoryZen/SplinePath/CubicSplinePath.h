/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file CubicSplinePath.h
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 15:40:37, August 04, 2024
 */
#pragma once

#include "EigenVectorTraits.h"
#include "../Base/PathBase.h"
#include <Eigen/Dense>

namespace tz
{
class TRAJECTORYZEN_EXPORT CubicSpline : public PathBase<Eigen::VectorXd>
{
public:
    CubicSpline(const std::vector<PointT> &points);
    CubicSpline(const std::vector<PointT> &points, std::vector<double> knots,
                const TangentT &deriv1st_initial,
                const TangentT &deriv1st_final);
    virtual ~CubicSpline() override = default;
    virtual double GetLength() const override;
    virtual PointT EvalPoint(double s) const override;
    virtual TangentT EvalTangent(double s) const override;
    virtual CurvatureT EvalCurvature(double s) const override;

private:
    Eigen::MatrixXd m_coeffs;
    std::vector<double> m_knots;
};

} // namespace tz
