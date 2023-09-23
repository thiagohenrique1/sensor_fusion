#ifndef VSSS_EKFMODEL_H
#define VSSS_EKFMODEL_H

#include <Types.h>

const float ROBOT_SIZE = 0.0675f;

class EkfModel {
	private:
	void process_noise(float time);

	public:
	using EKF = EKFTypes<5, 3, 3, 2>;

//	EKF::PoseMat F;
	EKF::PoseMat R;

//	EKF::HSensorMat H;
	EKF::SensorMat Q;

//	EKF::HVisionMat Hv;
	EKF::VisionMat Qv;

	EkfModel();
	EKF::PoseVec prediction(const EKF::PoseVec &prev_x,
							const EKF::ControlVec &controls, float time);
	EKF::SensorVec sensor_measurement_error(const EKF::PoseVec &x, const EKF::SensorVec &z);
	EKF::SensorVec sensor_measurement_model(const EKF::PoseVec &x);
	EKF::VisionVec vision_measurement_error(const EKF::PoseVec &x, const EKF::VisionVec &z);
	EKF::VisionVec vision_measurement_model(const EKF::PoseVec &x);
	void use_encoders(bool use);
};

#endif //VSSS_EKFMODEL_H
