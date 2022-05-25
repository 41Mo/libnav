#pragma once
#include <optional>
#include "math.hpp"

struct D_GNSS {
	std::optional<matrix::Vector2f> pos{std::nullopt};
	std::optional<matrix::Vector3f> vel{std::nullopt};
};

struct D_IMU {
	std::optional<matrix::Vector3f> acc{std::nullopt};
	std::optional<matrix::Vector3f> gyr{std::nullopt};
};

struct D_IN
{
	D_IMU imu;
	D_GNSS gnss;
};