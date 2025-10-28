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

#include "dynamics/vehicle_command.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

namespace adore
{
namespace dynamics
{


// Member function to convert to Eigen::Vector2d
Eigen::Vector2d
VehicleCommand::to_eigen_vector() const
{
  // Assuming the order is [acceleration, steering_angle]
  return Eigen::Vector2d( acceleration, steering_angle );
}

// Static member function to create from Eigen::Vector2d
VehicleCommand
VehicleCommand::from_eigen_vector( const Eigen::Vector2d& vec )
{
  // vec[0] = acceleration, vec[1] = steering_angle
  return VehicleCommand( vec[1], vec[0] );
}

// Member function to clamp command within given limits
void
VehicleCommand::clamp_within_limits( const PhysicalVehicleParameters& params )
{
  // Check if acceleration is NaN, fallback to 0 if NaN
  if( std::isnan( acceleration ) )
  {
    acceleration = 0.0; // Default value for NaN acceleration
  }

  // Clamp acceleration within params
  acceleration = std::clamp( acceleration, params.acceleration_min, params.acceleration_max );

  // Check if steering_angle is NaN, fallback to 0 if NaN
  if( std::isnan( steering_angle ) )
  {
    steering_angle = 0.0; // Default value for NaN steering angle
  }

  // Clamp steering_angle within params
  steering_angle = std::clamp( steering_angle, params.steering_angle_min, params.steering_angle_max );
}


} // namespace dynamics
} // namespace adore
