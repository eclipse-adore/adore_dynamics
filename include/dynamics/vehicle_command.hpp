#pragma once

/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Mikkel Skov Maarssø
 *    Marko Mizdrak
 ********************************************************************************/
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

  double max_steering_angle = 0.7;
  double max_acceleration   = 1.0;
  double min_acceleration   = -1.0;
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
  void clamp_within_limits( const VehicleCommandLimits& limits );

  double steering_angle;
  double acceleration;
};

} // namespace dynamics
} // namespace adore
