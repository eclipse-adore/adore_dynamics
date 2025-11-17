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

namespace adore::dynamics
{
struct ComfortSettings
{
  double max_speed                = 13.7; // [m/s] 50 km/h, can be adjusted based on vehicle parameters
  double max_acceleration         = 2.0;  // [m/s^2] softer than physical max
  double min_acceleration         = -2.0; // [m/s^2] softer than physical min (braking)
  double max_lateral_acceleration = 2.0;  // [m/s^2] cornering comfort limit
  double speed_fraction_of_limit  = 1.0;  // [0.0 - 1.0] target cruising speed as fraction of road limit

  double time_headway     = 3.0; // [s] time headway
  double distance_headway = 4.0; // [m] safety distance after vehicle length

  ComfortSettings() = default;

  ComfortSettings( double max_acc, double min_acc, double lat_acc, double speed_fraction ) :
    max_acceleration( max_acc ),
    min_acceleration( min_acc ),
    max_lateral_acceleration( lat_acc ),
    speed_fraction_of_limit( speed_fraction )
  {}

  void
  clamp( const PhysicalVehicleParameters& params )
  {
    max_acceleration = std::min( max_acceleration, params.acceleration_max );
    min_acceleration = std::max( min_acceleration, params.acceleration_min );
    // ensure fraction is between 0 and 1
    speed_fraction_of_limit = std::clamp( speed_fraction_of_limit, 0.0, 1.0 );
    // ensure distance and time headway are positive
    time_headway     = std::max( time_headway, 0.1 );
    distance_headway = std::max( distance_headway, 0.1 );
  }
};
} // namespace adore::dynamics