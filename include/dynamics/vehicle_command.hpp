/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once
#include "dynamics/physical_vehicle_parameters.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

namespace adore
{
namespace dynamics
{

struct VehicleCommandLimits
{
  VehicleCommandLimits() {};
  VehicleCommandLimits( double max_steer, double min_acc, double max_acc ) :
    max_steering_angle( max_steer ),
    max_acceleration( max_acc ),
    min_acceleration( min_acc ) {};

  double max_steering_angle = 0.58;
  double max_acceleration   = 1.0;
  double min_acceleration   = -2.0;
};

struct VehicleCommand
{
  VehicleCommand() {}

  VehicleCommand( double steering_angle, double acceleration ) :
    steering_angle( steering_angle ),
    acceleration( acceleration )
  {}

  // Member function to convert to Eigen::Vector2d
  Eigen::Vector2d to_eigen_vector() const;

  // Static member function to create from Eigen::Vector2d
  static VehicleCommand from_eigen_vector( const Eigen::Vector2d& vec );

  // Member function to clamp command within given limits
  void clamp_within_limits( const PhysicalVehicleParameters& limits );

  double steering_angle;
  double acceleration;
};

} // namespace dynamics
} // namespace adore
