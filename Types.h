#ifndef VSSS_TYPES_H
#define VSSS_TYPES_H

#include<Eigen/Dense>
#include <string>
#include <cmath>


template <typename T>
inline std::string str(const T& x) {
	return std::to_string(x);
}
float to_rads(float degrees);
float wrap(float angle);

struct Pose {
	float x;
	float y;
	float theta;
	float v;
	float w;
	float mag_offset;

	explicit Pose(const Eigen::Matrix<float, 6, 1> &pose_vec) :
			x(pose_vec(0, 0)), y(pose_vec(1, 0)),
			theta(pose_vec(2, 0)), v(pose_vec(3, 0)),
			w(pose_vec(4, 0)), mag_offset(pose_vec(5, 0)) {}

	Pose(float x, float y, float theta, float v, float w, float mag_offset) :
			x(x), y(y), theta(theta), v(v), w(w), mag_offset(mag_offset) {}

	Pose() : x(0), y(0), theta(0), v(0), w(0), mag_offset(0) {}

	Eigen::Matrix<float, 6, 1> to_vec() {
		Eigen::Matrix<float, 6, 1> vec;
		vec(0, 0) = x;
		vec(1, 0) = y;
		vec(2, 0) = theta;
		vec(3, 0) = v;
		vec(4, 0) = w;
		vec(5, 0) = mag_offset;
		return vec;
	}

	Pose or_backwards(bool backwards) {
		static constexpr float PI = 3.1415926f;
		if (!backwards) return *this;
		else return {x, y, wrap(theta + PI), -v, w, mag_offset};
	}
};

struct Controls {
	float lin_accel;
	float ang_accel;

	explicit Controls(Eigen::Matrix<float, 2, 1> control_vec) :
			lin_accel(control_vec(0, 0)),
			ang_accel(control_vec(1, 0)) {}

	Controls(float lin_accel, float ang_accel) :
			lin_accel(lin_accel), ang_accel(ang_accel) {}

	Eigen::Matrix<float, 2, 1> to_vec() {
		Eigen::Matrix<float, 2, 1> vec;
		vec(0, 0) = lin_accel;
		vec(1, 0) = ang_accel;
		return vec;
	}
};

struct SensorData {
	float mag_theta;
	float gyro_w;
	float vel_left;
	float vel_right;

	SensorData(float mag_theta, float gyro_w,
			   float vel_left, float vel_right) :
			mag_theta(mag_theta), gyro_w(gyro_w),
			vel_left(vel_left), vel_right(vel_right) {}

	SensorData() : mag_theta(0), gyro_w(0), vel_left(0), vel_right(0) {}

	Eigen::Matrix<float, 4, 1> to_vec() {
		Eigen::Matrix<float, 4, 1> vec;
		vec(0, 0) = mag_theta;
		vec(1, 0) = gyro_w;
		vec(2, 0) = vel_left;
		vec(3, 0) = vel_right;
		return vec;
	}
};

struct VisionData {
	float x = 0;
	float y = 0;
	float theta = 0;
	float mag_offset = 0;

	VisionData() = default;

	VisionData(float x, float y, float theta, float mag_offset) :
			x(x), y(y), theta(theta), mag_offset(mag_offset) {}

	Eigen::Matrix<float, 4, 1> to_vec() {
		Eigen::Matrix<float, 4, 1> vec;
		vec(0, 0) = x;
		vec(1, 0) = y;
		vec(2, 0) = theta;
		vec(3, 0) = mag_offset;
		return vec;
	}
};

template<int POSE_SIZE, int SENSOR_SIZE, int VISION_SIZE, int CONTROL_SIZE>
class EKFTypes {
	public:
	template<int x, int y> using Matrix = Eigen::Matrix<float, x, y>;
	using PoseVec = Matrix<POSE_SIZE, 1>;
	using PoseMat = Matrix<POSE_SIZE, POSE_SIZE>;
	using ControlVec = Matrix<CONTROL_SIZE, 1>;

	using SensorVec = Matrix<SENSOR_SIZE, 1>;
	using SensorMat = Matrix<SENSOR_SIZE, SENSOR_SIZE>;
	using HSensorMat = Matrix<SENSOR_SIZE, POSE_SIZE>;
	using KSensorMat = Matrix<POSE_SIZE, SENSOR_SIZE>;

	using VisionVec = Matrix<VISION_SIZE, 1>;
	using VisionMat = Matrix<VISION_SIZE, VISION_SIZE>;
	using HVisionMat = Matrix<VISION_SIZE, POSE_SIZE>;
	using KVisionMat = Matrix<POSE_SIZE, VISION_SIZE>;
    
    using UKFSigmaMat = Matrix<POSE_SIZE, 2 * POSE_SIZE + 1>;
	using UKFSensorSigmaMat = Matrix<SENSOR_SIZE, 2 * POSE_SIZE + 1>;
    using UKFVisionSigmaMat = Matrix<VISION_SIZE, 2 * POSE_SIZE + 1>;
};

#endif //VSSS_TYPES_H
