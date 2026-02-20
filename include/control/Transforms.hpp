#pragma once

#include "control/Vec.hpp"

namespace olinv {

Vec2 clarke(const Vec3& abc);
Vec3 inv_clarke(const Vec2& ab);
Vec2 park(const Vec2& ab, double theta_e);
Vec2 inv_park(const Vec2& dq, double theta_e);

}  // namespace olinv
