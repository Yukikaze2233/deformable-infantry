#pragma once

#include <array>
#include <cmath>
#include <numbers>
#include <sstream>
#include <string>

namespace rmcs_core::chassis::suspension::geometry {

struct Vec3 {
    double x;
    double y;
    double z;

    [[nodiscard]] Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] Vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] Vec3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }
};

struct Plane {
    Vec3 normal;
    double d;
};

struct LocalGeometry {
    Vec3 center;
    Vec3 n;
    Vec3 x;
    Vec3 y;
    std::array<Vec3, 4> points;
    std::array<Vec3, 4> radial_directions;
    double l;
};

inline constexpr double kTolerance          = 1e-9;
inline constexpr double kGeometryTolerance  = 1e-5;
inline constexpr double kValidationTolerance = 1e-6;

[[nodiscard]] inline double dot(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

[[nodiscard]] inline Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

[[nodiscard]] inline double norm(const Vec3& vector) {
    return std::sqrt(dot(vector, vector));
}

[[nodiscard]] inline bool finite(const Vec3& vector) {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] inline Vec3 normalized(const Vec3& vector) {
    return vector / norm(vector);
}

[[nodiscard]] inline Vec3 project_to_plane(const Vec3& vector, const Vec3& normal) {
    return vector - normal * dot(vector, normal);
}

[[nodiscard]] inline Vec3 project_to_xoy(const Vec3& vector) {
    return {vector.x, vector.y, 0.0};
}

[[nodiscard]] inline double plane_eval(const Plane& plane, const Vec3& point) {
    return dot(plane.normal, point) - plane.d;
}

[[nodiscard]] inline double to_radians(double angle, bool angles_in_degrees) {
    return angles_in_degrees ? angle * std::numbers::pi / 180.0 : angle;
}

[[nodiscard]] inline double from_radians(double angle, bool angles_in_degrees) {
    return angles_in_degrees ? angle * 180.0 / std::numbers::pi : angle;
}

[[nodiscard]] inline Vec3 transform_local_point_to_world(
    const LocalGeometry& geometry, const Vec3& local_point) {
    return geometry.center + geometry.x * local_point.x + geometry.y * local_point.y
        + geometry.n * local_point.z;
}

[[nodiscard]] inline Vec3 rotate_x(const Vec3& vector, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {
        vector.x,
        c * vector.y - s * vector.z,
        s * vector.y + c * vector.z,
    };
}

[[nodiscard]] inline Vec3 rotate_y(const Vec3& vector, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {
        c * vector.x + s * vector.z,
        vector.y,
        -s * vector.x + c * vector.z,
    };
}

[[nodiscard]] inline Vec3 rotate_z(const Vec3& vector, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {
        c * vector.x - s * vector.y,
        s * vector.x + c * vector.y,
        vector.z,
    };
}

[[nodiscard]] inline Vec3 apply_zyx_rotation(
    const Vec3& vector, double yaw, double pitch, double roll) {
    return rotate_z(rotate_y(rotate_x(vector, roll), pitch), yaw);
}

inline bool normalize_plane(
    const Plane& input_plane, const Vec3& reference_normal, Plane& output_plane, std::string& error) {
    if (!finite(input_plane.normal) || !std::isfinite(input_plane.d)) {
        error = "plane coefficients must be finite.";
        return false;
    }

    const double plane_norm = norm(input_plane.normal);
    if (plane_norm <= kTolerance) {
        error = "plane normal must be non-zero.";
        return false;
    }

    output_plane.normal = input_plane.normal / plane_norm;
    output_plane.d      = input_plane.d / plane_norm;

    const double alignment = dot(output_plane.normal, reference_normal);
    if (alignment < -kTolerance) {
        output_plane.normal = output_plane.normal * -1.0;
        output_plane.d      = -output_plane.d;
    } else if (alignment <= kTolerance) {
        error = "plane normal must satisfy dot(m, n) > 0 after normalization.";
        return false;
    }

    return true;
}

[[nodiscard]] inline bool should_flip_plane_for_world_up(const Vec3& normal) {
    if (normal.z < -kTolerance) {
        return true;
    }
    if (std::abs(normal.z) <= kTolerance) {
        if (normal.y < -kTolerance) {
            return true;
        }
        if (std::abs(normal.y) <= kTolerance && normal.x < -kTolerance) {
            return true;
        }
    }
    return false;
}

inline bool normalize_plane_to_world_up(const Plane& input_plane, Plane& output_plane, std::string& error) {
    if (!finite(input_plane.normal) || !std::isfinite(input_plane.d)) {
        error = "plane coefficients must be finite.";
        return false;
    }

    const double plane_norm = norm(input_plane.normal);
    if (plane_norm <= kTolerance) {
        error = "plane normal must be non-zero.";
        return false;
    }

    output_plane.normal = input_plane.normal / plane_norm;
    output_plane.d      = input_plane.d / plane_norm;
    if (should_flip_plane_for_world_up(output_plane.normal)) {
        output_plane.normal = output_plane.normal * -1.0;
        output_plane.d      = -output_plane.d;
    }

    return true;
}

inline bool build_local_geometry(
    const std::array<Vec3, 4>& points, const Vec3& plane_normal, double l, LocalGeometry& geometry,
    std::string& error) {
    if (!finite(plane_normal) || !std::isfinite(l) || l <= 0.0) {
        error = "geometry parameters require finite plane_normal and l > 0.";
        return false;
    }

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!finite(points[i])) {
            error = "point_" + std::to_string(i + 1) + " must be finite.";
            return false;
        }
    }

    const double normal_norm = norm(plane_normal);
    if (normal_norm <= kTolerance) {
        error = "plane_normal must be non-zero.";
        return false;
    }

    geometry.points  = points;
    geometry.center  = {};
    geometry.n       = plane_normal / normal_norm;
    geometry.l       = l;
    for (const auto& point : points) {
        geometry.center = geometry.center + point;
    }
    geometry.center = geometry.center / static_cast<double>(points.size());

    const Vec3 y_seed = ((points[0] + points[3]) * 0.5) - geometry.center;
    const Vec3 y_span = project_to_plane(y_seed, geometry.n);
    if (norm(y_span) <= kTolerance) {
        error = "A_1/A_4 midpoint direction must span the local A-plane.";
        return false;
    }

    const Vec3 y_projection = project_to_xoy(y_span);
    if (norm(y_projection) <= kTolerance) {
        error = "A_1/A_4 midpoint direction must have a non-zero xoy projection.";
        return false;
    }

    geometry.y = normalized(y_span);

    const Vec3 x_axis = cross(geometry.y, geometry.n);
    if (norm(x_axis) <= kTolerance) {
        error = "local x axis is degenerate.";
        return false;
    }
    geometry.x = normalized(x_axis);
    geometry.y = normalized(cross(geometry.n, geometry.x));

    const double half_length   = l / 2.0;
    const double target_radius = l / std::sqrt(2.0);
    const double scale         = std::max(1.0, l);
    const std::array<Vec3, 4> expected_local_points = {{
        {+half_length, +half_length, 0.0},
        {+half_length, -half_length, 0.0},
        {-half_length, -half_length, 0.0},
        {-half_length, +half_length, 0.0},
    }};

    for (std::size_t i = 0; i < points.size(); ++i) {
        const Vec3 relative_point = points[i] - geometry.center;
        const double plane_residual = dot(relative_point, geometry.n);
        if (std::abs(plane_residual) > kGeometryTolerance * scale) {
            error = "point_" + std::to_string(i + 1) + " is not on the configured A-plane.";
            return false;
        }

        const double radius = norm(relative_point);
        if (std::abs(radius - target_radius) > kGeometryTolerance * scale) {
            error = "point_" + std::to_string(i + 1) + " radius is inconsistent with l.";
            return false;
        }

        geometry.radial_directions[i] = relative_point / radius;

        const Vec3 local_point = {
            dot(relative_point, geometry.x),
            dot(relative_point, geometry.y),
            dot(relative_point, geometry.n),
        };
        const auto& expected = expected_local_points[i];
        if (std::abs(local_point.x - expected.x) > kGeometryTolerance * scale
            || std::abs(local_point.y - expected.y) > kGeometryTolerance * scale
            || std::abs(local_point.z - expected.z) > kGeometryTolerance * scale) {
            std::ostringstream stream;
            stream << "point_" << (i + 1)
                   << " does not match the required clockwise A_i ordering in the shared local frame.";
            error = stream.str();
            return false;
        }
    }

    return true;
}

inline bool build_local_geometry_from_attitude(
    double yaw, double pitch, double roll, double l, LocalGeometry& geometry, std::string& error) {
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(roll) || !std::isfinite(l)
        || l <= 0.0) {
        error = "geometry parameters require finite yaw/pitch/roll and l > 0.";
        return false;
    }

    geometry.center = {0.0, 0.0, 0.0};
    geometry.x = normalized(apply_zyx_rotation({1.0, 0.0, 0.0}, yaw, pitch, roll));
    geometry.y = normalized(apply_zyx_rotation({0.0, 1.0, 0.0}, yaw, pitch, roll));
    geometry.n = normalized(apply_zyx_rotation({0.0, 0.0, 1.0}, yaw, pitch, roll));
    geometry.l = l;

    if (norm(project_to_xoy(geometry.y)) <= kTolerance) {
        error = "A_1/A_4 midpoint direction must have a non-zero xoy projection.";
        return false;
    }

    const double half_length = l / 2.0;
    const std::array<Vec3, 4> local_points = {{
        {+half_length, +half_length, 0.0},
        {+half_length, -half_length, 0.0},
        {-half_length, -half_length, 0.0},
        {-half_length, +half_length, 0.0},
    }};

    for (std::size_t i = 0; i < local_points.size(); ++i) {
        geometry.points[i] = transform_local_point_to_world(geometry, local_points[i]);
        geometry.radial_directions[i] = normalized(geometry.points[i] - geometry.center);
    }

    return true;
}

} // namespace rmcs_core::chassis::suspension::geometry
