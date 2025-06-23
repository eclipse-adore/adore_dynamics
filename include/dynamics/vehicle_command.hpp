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
#include "dynamics/physical_vehicle_parameters.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

namespace adore
{
namespace dynamics
{

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
