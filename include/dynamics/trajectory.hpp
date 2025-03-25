/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
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
};


} // namespace dynamics
} // namespace adore
