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

#include "UpdaterVehicleDynamics.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

#include <boost/math/distributions/chi_squared.hpp>

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterVehicleDynamics::UpdaterVehicleDynamics(const NoiseManager &noises, const Eigen::Matrix<double, 4, 1> &q_imu_to_vehicle,
                                               double wheelbase, double max_steering_angle_deg, double sigma_lateral_velocity,
                                               double sigma_vertical_velocity, double sigma_roll_pitch_rate, double sigma_yaw_rate_bound,
                                               double min_forward_speed, double yaw_rate_bound_margin, bool use_yaw_rate_bound,
                                               double chi2_multiplier, double max_lateral_velocity_residual,
                                               double max_vertical_velocity_residual, double max_roll_pitch_rate_residual,
                                               double max_yaw_rate_bound_residual, bool log_gate, const std::string &gate_log_path)
    : _noises(noises), _wheelbase(wheelbase), _sigma_lateral_velocity(sigma_lateral_velocity),
      _sigma_vertical_velocity(sigma_vertical_velocity), _sigma_roll_pitch_rate(sigma_roll_pitch_rate),
      _sigma_yaw_rate_bound(sigma_yaw_rate_bound), _min_forward_speed(min_forward_speed),
      _yaw_rate_bound_margin(yaw_rate_bound_margin), _use_yaw_rate_bound(use_yaw_rate_bound), _chi2_multiplier(chi2_multiplier),
      _max_lateral_velocity_residual(max_lateral_velocity_residual),
      _max_vertical_velocity_residual(max_vertical_velocity_residual), _max_roll_pitch_rate_residual(max_roll_pitch_rate_residual),
      _max_yaw_rate_bound_residual(max_yaw_rate_bound_residual), _log_gate(log_gate), _gate_log_path(gate_log_path) {

  Eigen::Matrix<double, 4, 1> q = ov_core::quatnorm(q_imu_to_vehicle);
  _R_ItoV = ov_core::quat_2_Rot(q);
  _wheelbase = std::max(_wheelbase, 1e-3);
  _max_steering_angle = std::abs(max_steering_angle_deg) * 3.14159265358979323846 / 180.0;
  _sigma_lateral_velocity = std::max(_sigma_lateral_velocity, 1e-6);
  _sigma_vertical_velocity = std::max(_sigma_vertical_velocity, 1e-6);
  _sigma_roll_pitch_rate = std::max(_sigma_roll_pitch_rate, 1e-6);
  _sigma_yaw_rate_bound = std::max(_sigma_yaw_rate_bound, 1e-6);
  _min_forward_speed = std::max(_min_forward_speed, 0.0);
  _yaw_rate_bound_margin = std::max(_yaw_rate_bound_margin, 0.0);
  _chi2_multiplier = std::max(_chi2_multiplier, 0.0);
  _max_lateral_velocity_residual = std::max(_max_lateral_velocity_residual, 0.0);
  _max_vertical_velocity_residual = std::max(_max_vertical_velocity_residual, 0.0);
  _max_roll_pitch_rate_residual = std::max(_max_roll_pitch_rate_residual, 0.0);
  _max_yaw_rate_bound_residual = std::max(_max_yaw_rate_bound_residual, 0.0);

  if (_log_gate && !_gate_log_path.empty()) {
    _gate_log.open(_gate_log_path, std::ios::out | std::ios::trunc);
    if (_gate_log.is_open()) {
      _gate_log << "timestamp,timestamp_imu,decision,reason,rows,have_imu,add_yaw_bound,"
                   "v_vehicle_x,v_vehicle_y,v_vehicle_z,w_vehicle_x,w_vehicle_y,w_vehicle_z,yaw_bound,"
                   "res_vy,res_vz,res_wx,res_wy,res_yaw_bound,chi2,chi2_gate_threshold,"
                   "max_lateral_residual,max_vertical_residual,max_roll_pitch_rate_residual,max_yaw_rate_bound_residual\n";
      _gate_log.flush();
    } else {
      PRINT_WARNING(YELLOW "[VDYN]: unable to open gate log file: %s\n" RESET, _gate_log_path.c_str());
    }
  }
}

void UpdaterVehicleDynamics::log_gate_decision(double timestamp, double timestamp_imu, const std::string &decision,
                                               const std::string &reason, int rows, bool have_imu, bool add_yaw_bound,
                                               const Eigen::Vector3d &v_V, const Eigen::Vector3d &w_V, double yaw_bound,
                                               double res_vy, double res_vz, double res_wx, double res_wy,
                                               double res_yaw_bound, double chi2, double chi2_threshold) {
  if (!_log_gate || !_gate_log.is_open()) {
    return;
  }
  _gate_log << std::setprecision(15) << timestamp << "," << timestamp_imu << "," << decision << "," << reason << "," << rows << ","
            << (have_imu ? 1 : 0) << "," << (add_yaw_bound ? 1 : 0) << "," << v_V(0) << "," << v_V(1) << "," << v_V(2) << ","
            << w_V(0) << "," << w_V(1) << "," << w_V(2) << "," << yaw_bound << "," << res_vy << "," << res_vz << "," << res_wx
            << "," << res_wy << "," << res_yaw_bound << "," << chi2 << "," << chi2_threshold << ","
            << _max_lateral_velocity_residual << "," << _max_vertical_velocity_residual << "," << _max_roll_pitch_rate_residual
            << "," << _max_yaw_rate_bound_residual << "\n";
  _gate_log.flush();
}

void UpdaterVehicleDynamics::feed_imu(const ov_core::ImuData &message, double oldest_time) {
  imu_data.emplace_back(message);
  clean_old_imu_measurements(oldest_time - 0.10);
}

void UpdaterVehicleDynamics::clean_old_imu_measurements(double oldest_time) {
  if (oldest_time < 0) {
    return;
  }
  auto it0 = imu_data.begin();
  while (it0 != imu_data.end()) {
    if (it0->timestamp < oldest_time) {
      it0 = imu_data.erase(it0);
    } else {
      it0++;
    }
  }
}

bool UpdaterVehicleDynamics::update(std::shared_ptr<State> state) {

  if (state == nullptr || state->_timestamp < 0) {
    return false;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  double timestamp_imu = state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0);

  // Vehicle-frame velocity: R_GtoV = R_ItoV * R_GtoI.
  Eigen::Matrix3d R_GtoI = state->_imu->Rot();
  Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
  Eigen::Matrix3d R_GtoV = _R_ItoV * R_GtoI;
  Eigen::Matrix3d R_GtoV_jacob = _R_ItoV * R_GtoI_jacob;
  Eigen::Vector3d v_V = R_GtoV * state->_imu->vel();
  if (!R_GtoV.allFinite() || !R_GtoV_jacob.allFinite() || !v_V.allFinite()) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "nonfinite_velocity", 0, false, false, v_V, Eigen::Vector3d::Zero(),
                      nan, nan, nan, nan, nan, nan, nan, nan);
    PRINT_WARNING(YELLOW "[VDYN]: rejected non-finite vehicle-frame velocity\n" RESET);
    return false;
  }

  // Use the latest IMU sample at or before the current IMU time for angular-rate pseudo measurements.
  bool have_imu = false;
  ImuData imu_meas;
  imu_meas.timestamp = 0.0;
  imu_meas.wm.setZero();
  imu_meas.am.setZero();
  for (const auto &data : imu_data) {
    if (data.timestamp <= timestamp_imu) {
      imu_meas = data;
      have_imu = true;
    }
  }

  Eigen::Vector3d w_V = Eigen::Vector3d::Zero();
  Eigen::Matrix3d dw_dbg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dw_dba = Eigen::Matrix3d::Zero();
  if (have_imu) {
    Eigen::Matrix3d Dw = State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
    Eigen::Matrix3d Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
    Eigen::Matrix3d Tg = State::Tg(state->_calib_imu_tg->value());
    Eigen::Matrix3d R_GYROtoIMU = state->_calib_imu_GYROtoIMU->Rot();
    Eigen::Matrix3d R_ACCtoIMU = state->_calib_imu_ACCtoIMU->Rot();
    Eigen::Vector3d a_hat = R_ACCtoIMU * Da * (imu_meas.am - state->_imu->bias_a());
    Eigen::Vector3d w_hat = R_GYROtoIMU * Dw * (imu_meas.wm - state->_imu->bias_g() - Tg * a_hat);
    w_V = _R_ItoV * w_hat;
    dw_dbg = -_R_ItoV * R_GYROtoIMU * Dw;
    dw_dba = _R_ItoV * R_GYROtoIMU * Dw * Tg * R_ACCtoIMU * Da;
    if (!w_V.allFinite() || !dw_dbg.allFinite() || !dw_dba.allFinite()) {
      log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "nonfinite_angular_rate", 0, have_imu, false, v_V, w_V, nan, nan,
                        nan, nan, nan, nan, nan, nan);
      PRINT_WARNING(YELLOW "[VDYN]: rejected non-finite vehicle-frame angular rate\n" RESET);
      return false;
    }
  }

  // Rows: lateral velocity, vertical velocity, optional roll/pitch angular rates, optional yaw-rate bound residual.
  int rows = 2 + (have_imu ? 2 : 0);
  bool add_yaw_bound = false;
  double yaw_bound_sign = 1.0;
  double yaw_bound = 0.0;
  if (_use_yaw_rate_bound && have_imu && std::abs(v_V(0)) > _min_forward_speed) {
    yaw_bound = std::abs(v_V(0)) / _wheelbase * std::tan(_max_steering_angle) + _yaw_rate_bound_margin;
    if (std::abs(w_V(2)) > yaw_bound) {
      add_yaw_bound = true;
      yaw_bound_sign = (w_V(2) >= 0.0) ? 1.0 : -1.0;
      rows++;
    }
  }

  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->v());
  Hx_order.push_back(state->_imu->bg());
  Hx_order.push_back(state->_imu->ba());

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, 12);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(rows);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(rows, rows);

  Eigen::Matrix3d dv_dtheta = ov_core::skew_x(R_GtoV_jacob * state->_imu->vel());
  Eigen::Matrix3d dv_dv = R_GtoV_jacob;
  int row = 0;

  // z = 0, h = v_vehicle_y / v_vehicle_z, residual = z - h.
  res(row) = -v_V(1);
  H.block(row, 0, 1, 3) = -dv_dtheta.row(1);
  H.block(row, 3, 1, 3) = -dv_dv.row(1);
  R(row, row) = _sigma_lateral_velocity * _sigma_lateral_velocity;
  row++;

  res(row) = -v_V(2);
  H.block(row, 0, 1, 3) = -dv_dtheta.row(2);
  H.block(row, 3, 1, 3) = -dv_dv.row(2);
  R(row, row) = _sigma_vertical_velocity * _sigma_vertical_velocity;
  row++;

  if (have_imu) {
    // z = 0, h = omega_vehicle_x / omega_vehicle_y.
    res(row) = -w_V(0);
    H.block(row, 6, 1, 3) = -dw_dbg.row(0);
    H.block(row, 9, 1, 3) = -dw_dba.row(0);
    R(row, row) = _sigma_roll_pitch_rate * _sigma_roll_pitch_rate;
    row++;

    res(row) = -w_V(1);
    H.block(row, 6, 1, 3) = -dw_dbg.row(1);
    H.block(row, 9, 1, 3) = -dw_dba.row(1);
    R(row, row) = _sigma_roll_pitch_rate * _sigma_roll_pitch_rate;
    row++;
  }

  if (add_yaw_bound) {
    // Inequality-style soft residual for |omega_z| <= |v_x| / L * tan(delta_max) + margin.
    double tan_delta_over_l = std::tan(_max_steering_angle) / _wheelbase;
    double forward_speed_sign = (v_V(0) >= 0.0) ? 1.0 : -1.0;
    double h = yaw_bound_sign * w_V(2) - std::abs(v_V(0)) * tan_delta_over_l - _yaw_rate_bound_margin;
    res(row) = -h;
    H.block(row, 0, 1, 3) = forward_speed_sign * tan_delta_over_l * dv_dtheta.row(0);
    H.block(row, 3, 1, 3) = forward_speed_sign * tan_delta_over_l * dv_dv.row(0);
    H.block(row, 6, 1, 3) = -yaw_bound_sign * dw_dbg.row(2);
    H.block(row, 9, 1, 3) = -yaw_bound_sign * dw_dba.row(2);
    R(row, row) = _sigma_yaw_rate_bound * _sigma_yaw_rate_bound;
    PRINT_DEBUG(YELLOW "[VDYN]: yaw-rate bound active |wz| %.3f > %.3f rad/s\n" RESET, std::abs(w_V(2)), yaw_bound);
    row++;
  }

  double res_vy = res(0);
  double res_vz = res(1);
  double res_wx = have_imu ? res(2) : nan;
  double res_wy = have_imu ? res(3) : nan;
  double res_yaw_bound = add_yaw_bound ? res(rows - 1) : nan;

  if (std::abs(res(0)) > _max_lateral_velocity_residual || std::abs(res(1)) > _max_vertical_velocity_residual) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "velocity_residual", rows, have_imu, add_yaw_bound, v_V, w_V,
                      yaw_bound, res_vy, res_vz, res_wx, res_wy, res_yaw_bound, nan, nan);
    PRINT_DEBUG(YELLOW "[VDYN]: rejected velocity residual vy %.3f / vz %.3f\n" RESET, std::abs(res(0)), std::abs(res(1)));
    return false;
  }
  if (have_imu && (std::abs(res(2)) > _max_roll_pitch_rate_residual || std::abs(res(3)) > _max_roll_pitch_rate_residual)) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "roll_pitch_rate_residual", rows, have_imu, add_yaw_bound, v_V,
                      w_V, yaw_bound, res_vy, res_vz, res_wx, res_wy, res_yaw_bound, nan, nan);
    PRINT_DEBUG(YELLOW "[VDYN]: rejected roll/pitch-rate residual wx %.3f / wy %.3f\n" RESET, std::abs(res(2)), std::abs(res(3)));
    return false;
  }
  if (add_yaw_bound && std::abs(res(rows - 1)) > _max_yaw_rate_bound_residual) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "yaw_rate_bound_residual", rows, have_imu, add_yaw_bound, v_V, w_V,
                      yaw_bound, res_vy, res_vz, res_wx, res_wy, res_yaw_bound, nan, nan);
    PRINT_DEBUG(YELLOW "[VDYN]: rejected yaw-rate bound residual %.3f\n" RESET, std::abs(res(rows - 1)));
    return false;
  }
  if (!H.allFinite() || !res.allFinite() || !R.allFinite()) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "nonfinite_update_matrices", rows, have_imu, add_yaw_bound, v_V,
                      w_V, yaw_bound, res_vy, res_vz, res_wx, res_wy, res_yaw_bound, nan, nan);
    PRINT_WARNING(YELLOW "[VDYN]: rejected non-finite update matrices\n" RESET);
    return false;
  }

  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  if (!P_marg.allFinite() || !S.allFinite()) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "nonfinite_covariance", rows, have_imu, add_yaw_bound, v_V, w_V,
                      yaw_bound, res_vy, res_vz, res_wx, res_wy, res_yaw_bound, nan, nan);
    PRINT_WARNING(YELLOW "[VDYN]: rejected non-finite covariance\n" RESET);
    return false;
  }
  Eigen::LLT<Eigen::MatrixXd> llt(S);
  if (llt.info() != Eigen::Success) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "innovation_covariance_not_spd", rows, have_imu, add_yaw_bound, v_V,
                      w_V, yaw_bound, res_vy, res_vz, res_wx, res_wy, res_yaw_bound, nan, nan);
    PRINT_WARNING(YELLOW "[VDYN]: rejected update with non-positive innovation covariance\n" RESET);
    return false;
  }
  double chi2 = res.dot(llt.solve(res));
  boost::math::chi_squared chi_squared_dist(rows);
  double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
  double chi2_gate_threshold = _chi2_multiplier * chi2_check;
  if (!std::isfinite(chi2) || chi2 > chi2_gate_threshold) {
    log_gate_decision(state->_timestamp, timestamp_imu, "rejected", "chi2", rows, have_imu, add_yaw_bound, v_V, w_V, yaw_bound,
                      res_vy, res_vz, res_wx, res_wy, res_yaw_bound, chi2, chi2_gate_threshold);
    PRINT_DEBUG(YELLOW "[VDYN]: rejected chi2 %.3f > %.3f\n" RESET, chi2, chi2_gate_threshold);
    return false;
  }

  log_gate_decision(state->_timestamp, timestamp_imu, "accepted", "accepted", rows, have_imu, add_yaw_bound, v_V, w_V, yaw_bound,
                    res_vy, res_vz, res_wx, res_wy, res_yaw_bound, chi2, chi2_gate_threshold);
  StateHelper::EKFUpdate(state, Hx_order, H, res, R);
  PRINT_DEBUG(CYAN "[VDYN]: accepted chi2 %.3f < %.3f | v_vehicle = %.3f, %.3f, %.3f | w_vehicle = %.3f, %.3f, %.3f\n" RESET,
              chi2, chi2_gate_threshold, v_V(0), v_V(1), v_V(2), w_V(0), w_V(1), w_V(2));
  return true;
}
