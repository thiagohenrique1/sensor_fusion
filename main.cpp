#include<Eigen/Dense>
#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<filesystem>
#include<iomanip>

#include<UKF.h>

struct Data {
    double time;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    Eigen::Vector3d mag;
    // left and right wheel velocity in m/s
    Eigen::Vector2d wheel_vel;
};

// split comma separated string into vector of strings
std::vector<std::string> split(std::string input) {
    std::vector<std::string> output;
    std::string current;
    for (char c : input) {
        if (c == ',') {
            output.push_back(current);
            current = "";
        } else {
            current += c;
        }
    }
    output.push_back(current);
    return output;
}

std::vector<Data> read_csv(std::string filename) {
    std::ifstream file("/home/thiago/Workspace/vsss/sensor_fusion/vsss_esp_data/" + filename);
    std::vector<Data> data;
    std::string line;
    while (std::getline(file, line)) {
        // file starts with many invalid lines
        // is is valid if it can be broken into 12 comma separated values
        auto split_line = split(line);
        if (split_line.size() == 12) {
            Data d;
            // time is an integer in microseconds, convert to seconds in double
            d.time = std::stod(split_line[0]) / 1000000;
            d.gyro = Eigen::Vector3d(std::stod(split_line[1]), std::stod(split_line[2]), std::stod(split_line[3]));
            d.accel = Eigen::Vector3d(std::stod(split_line[4]), std::stod(split_line[5]), std::stod(split_line[6]));
            d.mag = Eigen::Vector3d(std::stod(split_line[7]), std::stod(split_line[8]), std::stod(split_line[9]));
            d.wheel_vel = Eigen::Vector2d(std::stod(split_line[10]) / -2.0, std::stod(split_line[11]) /2.0);
            data.push_back(d);
        }
    }
    return data;
}

// Robot is still, so acceleration is only due to gravity
// Acceleration is in m/s^2
Eigen::Quaterniond orientation_from_accel(Eigen::Vector3d accel) {
    Eigen::Vector3d gravity(0, 0, -9.81);
    Eigen::Vector3d axis = accel.cross(gravity);
    double angle = std::acos(accel.dot(gravity) / (accel.norm() * gravity.norm()));
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

// Gyro is in rad/s
Eigen::Quaterniond update_orientation(Eigen::Quaterniond q, Eigen::Vector3d gyro, double dt) {
    Eigen::Quaterniond dq;
    dq.w() = 1;
    dq.x() = gyro[0] * dt / 2;
    dq.y() = gyro[1] * dt / 2;
    dq.z() = gyro[2] * dt / 2;
    return q * dq;
}

// Wrap angle from -pi to pi
double wrap_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

double rad_to_deg(double rad) {
    return rad * 180 / M_PI;
}

// in roll, pitch, yaw, each angle from -180 to 180
// limit to 2 decimal places
// separate by tab
void print_quat_in_euler(Eigen::Quaterniond q) {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    double roll = rad_to_deg(wrap_angle(euler[0]));
    double pitch = rad_to_deg(wrap_angle(euler[1]));
    double yaw = rad_to_deg(wrap_angle(euler[2]));
    // limit to 2 decimal places, separate by two tabs
    std::cout << std::fixed << std::setprecision(2) << roll << "\t\t" << pitch << "\t\t" << yaw << std::endl;
}

// compute gyro bias by averaging the data, removing the first and last 10% of the data
// assuming the robot is still
Eigen::Vector3d compute_gyro_bias(std::vector<Data> data) {
    int start = data.size() / 10;
    int end = data.size() - start;
    Eigen::Vector3d bias(0, 0, 0);
    for (int i = start; i < end; i++) {
        bias += data[i].gyro;
    }
    bias /= (end - start);
    return bias;
}

// compute gyro scale by comparing to the angular velocity computed from the velocity of both wheels
// velocity of each wheel is in m/s
// robot is spinning in z axis, angular velocity is proportional to wheel velocity
// robot is differentially driven
double compute_gyro_scale(std::vector<Data> data) {
    double robot_radius = 0.07;
    std::vector<double> proportion;
    // remove first and last 10% of data
    int start = data.size() / 10;
    int end = data.size() - start;
    for (int i = start; i < end; i++) {
        Data d = data[i];
        double angular_velocity = (d.wheel_vel[1] - d.wheel_vel[0]) / (robot_radius);
        double gyro_norm = d.gyro.norm();
        proportion.push_back(angular_velocity / gyro_norm);
        // std::cout << "angular velocity: " << angular_velocity << ", gyro norm: " << gyro_norm << ", gyro z: " << d.gyro[2] << ", proportion: " << proportion.back() << std::endl;
    }
    double sum = 0;
    for (double p : proportion) {
        sum += p;
    }
    return sum / proportion.size();
}

std::vector<Data> read_and_adjust_gyro(std::string filename) {
    std::vector<Data> data = read_csv(filename);
    // remove bias
    std::vector<Data> data_still = read_csv("still.csv");
    Eigen::Vector3d bias = compute_gyro_bias(data_still);
    for (auto& d : data) {
        d.gyro -= bias;
    }
    // compute scale
    std::vector<Data> data_scale = read_csv("spin.csv");
    double scale = compute_gyro_scale(data_scale);
    for (auto& d : data) {
        d.gyro *= scale;
    }
    return data;
}

void print_data(std::vector<Data> data) {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    for (int i = 1; i < data.size(); i++) {
        double dt = data[i].time - data[i - 1].time;
        q = update_orientation(q, data[i].gyro, dt);
        std::cout << "g: ";
        print_quat_in_euler(q);
        // also print the orientation from the accelerometer
        Eigen::Quaterniond q_accel = orientation_from_accel(data[i].accel);
        std::cout << "a: ";
        print_quat_in_euler(q_accel);
    }
}

int main() {
    std::vector<Data> data = read_and_adjust_gyro("circular.csv");

    // using 2d model of robot
    UKF ukf;

    for (int i = 1; i < data.size(); i++) {
        double dt = data[i].time - data[i - 1].time;

        Data prev_data = data[i - 1];

        float gyro_ang_accel = (data[i].gyro[2] - prev_data.gyro[2]) / dt;
        float prev_wheel_lin_vel = (prev_data.wheel_vel[0] + prev_data.wheel_vel[1]) / 2;
        float wheel_lin_vel = (data[i].wheel_vel[0] + data[i].wheel_vel[1]) / 2;
        float wheel_accel = (wheel_lin_vel - prev_wheel_lin_vel) / dt;

        
        UKF::T::ControlVec controls {
            wheel_accel,
            gyro_ang_accel
        };

        ukf.predict(controls, dt);

        UKF::T::SensorVec sensor_data {
            data[i].mag[2],
            data[i].gyro[2],
            data[i].wheel_vel[0],
            data[i].wheel_vel[1]
        };

        ukf.update_on_sensor_data(sensor_data);

        // print ukf orientation in z
        std::cout << ukf.x(2, 0) << std::endl;
    }

    //  print orientation from gyro
    // Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    // for (int i = 1; i < data.size(); i++) {
    //     double dt = data[i].time - data[i - 1].time;
    //     q = update_orientation(q, data[i].gyro, dt);
    //     print_quat_in_euler(q);
    // }

    return 0;
}