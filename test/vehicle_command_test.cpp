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

#include <gtest/gtest.h>

#include <limits>

#include "dynamics/physical_vehicle_parameters.hpp"

using adore::dynamics::PhysicalVehicleParameters;
using adore::dynamics::VehicleCommand;

TEST( VehicleCommand, ToFromEigenRoundTrip )
{
  // steering_angle, acceleration
  VehicleCommand cmd_in( 0.25, 1.5 );

  Eigen::Vector2d vec = cmd_in.to_eigen_vector();

  // Implementation: vec[0] = acceleration, vec[1] = steering_angle
  EXPECT_DOUBLE_EQ( vec[0], cmd_in.acceleration );
  EXPECT_DOUBLE_EQ( vec[1], cmd_in.steering_angle );

  VehicleCommand cmd_out = VehicleCommand::from_eigen_vector( vec );

  // from_eigen_vector should invert to_eigen_vector
  EXPECT_DOUBLE_EQ( cmd_out.acceleration, cmd_in.acceleration );
  EXPECT_DOUBLE_EQ( cmd_out.steering_angle, cmd_in.steering_angle );
}

TEST( VehicleCommand, ClampWithinLimitsAndNaNHandling )
{
  PhysicalVehicleParameters params;

  // Set sane clamp bounds (override the defaults if needed)
  params.acceleration_min   = -2.0;
  params.acceleration_max   = 3.0;
  params.steering_angle_min = -0.5;
  params.steering_angle_max = 0.5;

  // Start with values outside the limits
  VehicleCommand cmd( 1.0, 10.0 ); // steering_angle, acceleration

  cmd.clamp_within_limits( params );

  EXPECT_DOUBLE_EQ( cmd.acceleration, params.acceleration_max );
  EXPECT_DOUBLE_EQ( cmd.steering_angle, params.steering_angle_max );

  // Now test NaN handling: both should be reset to 0, then clamped
  cmd.acceleration   = std::numeric_limits<double>::quiet_NaN();
  cmd.steering_angle = std::numeric_limits<double>::quiet_NaN();

  cmd.clamp_within_limits( params );

  EXPECT_DOUBLE_EQ( cmd.acceleration, 0.0 );
  EXPECT_DOUBLE_EQ( cmd.steering_angle, 0.0 );
}
