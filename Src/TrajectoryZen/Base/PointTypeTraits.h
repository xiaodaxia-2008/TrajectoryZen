/**
 * Copyright © 2024 Zen Shawn. All rights reserved.
 *
 * @file PointTypeTraits.h
 * @author Zen Shawn
 * @email xiaozisheng2008@hotmail.com
 * @date 14:59:59, August 04, 2024
 */
#pragma once
#include <type_traits>

namespace tz
{
template <typename PointT>
struct PointTypeTraits {
    using TangentT =
        std::decay_t<decltype(std::declval<PointT>() - std::declval<PointT>())>;
    using CurvatureT = std::decay_t<decltype(std::declval<TangentT>()
                                             - std::declval<TangentT>())>;
};

} // namespace tz