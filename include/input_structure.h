#pragma once
#include <optional>
#include "math.hpp"

namespace NavA {
struct D_GNSS {
	std::optional<matrix::Vector2d> pos{std::nullopt};
	std::optional<matrix::Vector3d> vel{std::nullopt};
};

struct D_IMU {
	std::optional<matrix::Vector3d> acc{std::nullopt};
	std::optional<matrix::Vector3d> gyr{std::nullopt};
	std::optional<matrix::Vector3d> mag{std::nullopt};
};

struct D_IN
{
	D_IMU imu;
	D_GNSS gnss;
};
}