#include "control/Transforms.hpp"

#include <cmath>

namespace olinv {

Vec2 clarke(const Vec3& abc) {
    constexpr double INV_SQRT3 = 0.5773502691896257645;
    return {abc.a, (abc.a + 2.0 * abc.b) * INV_SQRT3};
}

Vec3 inv_clarke(const Vec2& ab) {
    constexpr double HALF = 0.5;
    constexpr double SQRT3_OVER_2 = 0.8660254037844386468;
    return {ab.x, -HALF * ab.x + SQRT3_OVER_2 * ab.y, -HALF * ab.x - SQRT3_OVER_2 * ab.y};
}

Vec2 park(const Vec2& ab, double theta_e) {
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    return {c * ab.x + s * ab.y, -s * ab.x + c * ab.y};
}

Vec2 inv_park(const Vec2& dq, double theta_e) {
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    return {c * dq.x - s * dq.y, s * dq.x + c * dq.y};
}

}  // namespace olinv
