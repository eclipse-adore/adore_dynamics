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
#include "adore_math/angles.h"
#include "adore_math/distance.h"

#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace dynamics
{


struct Trajectory
{
  std::vector<VehicleStateDynamic> states;
  std::string                      label;


  VehicleStateDynamic get_state_at_time( double time ) const;

  template<typename Point>
  VehicleStateDynamic
  get_nearest_state( const Point& reference_point ) const
  {
    if( states.size() < 2 )
    {
      throw std::runtime_error( "Not enough states to interpolate." );
    }

    double min_distance  = std::numeric_limits<double>::max();
    size_t nearest_index = 0;

    for( size_t i = 0; i < states.size(); i++ )
    {
      double distance = adore::math::squared_distance_2d( reference_point, states[i] );
      if( distance < min_distance )
      {
        min_distance  = distance;
        nearest_index = i;
      }
    }

    // Handle edge cases where the nearest state is the first or last in the list
    if( nearest_index == 0 || nearest_index == states.size() - 1 )
    {
      return states[nearest_index];
    }

    // Find the two nearest states
    const auto& state1 = states[nearest_index];
    const auto& state2 = states[nearest_index + 1];

    // Compute the distances
    double distance1 = adore::math::distance_2d( reference_point, state1 );
    double distance2 = adore::math::distance_2d( reference_point, state2 );

    // Interpolation factor
    double alpha = distance1 / ( distance1 + distance2 );

    return interpolate_states_linear( state1, state2, alpha );
  }

  friend std::ostream&
  operator<<( std::ostream& os, const Trajectory& trajectory )
  {
    os << "Trajectory(label=\"" << trajectory.label << "\", size=" << trajectory.states.size() << ")\n";
    for( const auto& s : trajectory.states )
    {
      os << s << '\n'; // relies on VehicleStateDynamic::operator<<
    }
    return os;
  }

  void
  adjust_start_time( double start_time )
  {
    if( states.empty() )
      return;
    double offset = start_time - states[0].time;
    for( auto& state : states )
    {
      state.time += offset;
    }
  }

  template<typename Model>
  void
  infer_steering_angles( const Model& model )
  {
    if( states.size() < 2 )
      return;

    for( size_t i = 0; i < states.size() - 1; ++i )
    {
      VehicleStateDynamic&       s0 = states[i];
      const VehicleStateDynamic& s1 = states[i + 1];

      double dt = s1.time - s0.time;
      if( dt <= 0.0 )
        continue;

      // Approximate yaw rate
      double dyaw     = adore::math::normalize_angle( s1.yaw_angle - s0.yaw_angle );
      double yaw_rate = dyaw / dt;

      // Estimate steering angle using the inverse of the kinematic bicycle model:
      // yaw_rate = v * tan(steering_angle) / wheelbase
      if( std::abs( s0.vx ) > 1e-2 ) // avoid division by zero
      {
        double steering_angle = std::atan2( yaw_rate * model.params.wheelbase, s0.vx );
        s0.steering_angle     = steering_angle;

        // Approximate steering rate
        double next_steering_angle = std::atan2( ( s1.yaw_rate * model.params.wheelbase ), s1.vx );
        s0.steering_rate           = ( next_steering_angle - steering_angle ) / dt;
      }
      else
      {
        s0.steering_angle = 0.0;
        s0.steering_rate  = 0.0;
      }
    }

    // Set last state equal to previous one (or zero) for consistency
    states.back().steering_angle = states[states.size() - 2].steering_angle;
    states.back().steering_rate  = 0.0;
  }
};


} // namespace dynamics
} // namespace adore
