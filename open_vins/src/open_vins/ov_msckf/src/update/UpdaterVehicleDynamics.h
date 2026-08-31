/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_UPDATER_VEHICLE_DYNAMICS_H
#define OV_MSCKF_UPDATER_VEHICLE_DYNAMICS_H

#include <Eigen/Eigen>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "utils/NoiseManager.h"
#include "utils/sensor_data.h"

namespace ov_msckf {

class State;

/**
 * @brief Soft ground-vehicle dynamics / non-holonomic constraint updater.
 *
 * This follows the P-VIDO vehicle-frame measurement idea that the vehicle
 * velocity is mostly forward and angular velocity is mostly yaw. If wheel speed
 * and steering measurements are unavailable, this updater uses pseudo
 * measurements v_y = 0, v_z = 0, omega_x = 0, omega_y = 0, with an optional
 * Ackermann yaw-rate bound from wheelbase and maximum front wheel angle.
 */
class UpdaterVehicleDynamics {

public:
  UpdaterVehicleDynamics(const NoiseManager &noises, const Eigen::Matrix<double, 4, 1> &q_imu_to_vehicle, double wheelbase,
                         double max_steering_angle_deg, double sigma_lateral_velocity, double sigma_vertical_velocity,
                         double sigma_roll_pitch_rate, double sigma_yaw_rate_bound, double min_forward_speed,
                         double yaw_rate_bound_margin, bool use_yaw_rate_bound, double chi2_multiplier,
                         double max_lateral_velocity_residual, double max_vertical_velocity_residual,
                         double max_roll_pitch_rate_residual, double max_yaw_rate_bound_residual, bool log_gate,
                         const std::string &gate_log_path);

  /**
   * @brief Feed function for inertial data.
   * @param message Contains timestamp and inertial information.
   * @param oldest_time Time that we can discard measurements before.
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1);

  /**
   * @brief Remove old IMU measurements.
   * @param oldest_time Time that we can discard measurements before.
   */
  void clean_old_imu_measurements(double oldest_time);

  /**
   * @brief Apply the soft vehicle dynamics update at the current state time.
   * @param state State of the filter.
   * @return True if an EKF update was applied.
   */
  bool update(std::shared_ptr<State> state);

protected:
  /// Log a vehicle-dynamics gate decision for offline diagnosis.
  void log_gate_decision(double timestamp, double timestamp_imu, const std::string &decision, const std::string &reason, int rows,
                         bool have_imu, bool add_yaw_bound, const Eigen::Vector3d &v_V, const Eigen::Vector3d &w_V,
                         double yaw_bound, double res_vy, double res_vz, double res_wx, double res_wy,
                         double res_yaw_bound, double chi2, double chi2_threshold);

  /// Latest IMU data used for roll/pitch/yaw-rate constraints.
  std::vector<ov_core::ImuData> imu_data;

  /// IMU noise characteristics.
  NoiseManager _noises;

  /// Rotation from IMU frame into vehicle dynamics frame.
  Eigen::Matrix3d _R_ItoV = Eigen::Matrix3d::Identity();

  /// Vehicle wheelbase in meters.
  double _wheelbase = 1.66;

  /// Maximum front wheel steering angle in radians.
  double _max_steering_angle = 30.0 * 3.14159265358979323846 / 180.0;

  /// Soft non-holonomic measurement noises.
  double _sigma_lateral_velocity = 0.20;
  double _sigma_vertical_velocity = 0.20;
  double _sigma_roll_pitch_rate = 0.10;
  double _sigma_yaw_rate_bound = 0.20;

  /// Minimum forward speed before the yaw-rate bound is meaningful.
  double _min_forward_speed = 0.20;

  /// Extra allowed yaw-rate above the Ackermann bound.
  double _yaw_rate_bound_margin = 0.20;

  /// If true, add an inequality-style residual when yaw rate exceeds the Ackermann bound.
  bool _use_yaw_rate_bound = true;

  /// Innovation gate multiplier for rejecting vehicle-dynamics pseudo-measurements.
  double _chi2_multiplier = 5.0;

  /// Absolute residual sanity limits before applying an EKF update.
  double _max_lateral_velocity_residual = 5.0;
  double _max_vertical_velocity_residual = 5.0;
  double _max_roll_pitch_rate_residual = 2.0;
  double _max_yaw_rate_bound_residual = 3.0;

  /// Optional CSV log of accepted/rejected vehicle-dynamics updates.
  bool _log_gate = false;
  std::string _gate_log_path;
  std::ofstream _gate_log;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_VEHICLE_DYNAMICS_H
