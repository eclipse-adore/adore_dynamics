/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#include "dynamics/trajectory.hpp"

namespace adore
{
namespace dynamics
{

VehicleStateDynamic
Trajectory::get_state_at_time( double query_time ) const
{
  // Check if the trajectory is empty
  if( states.empty() )
  {
    throw std::runtime_error( "Trajectory is empty." );
  }
  // Handle times before the first point
  if( query_time <= states.front().time )
  {
    return states.front();
  }
  // Handle times after the last point
  if( query_time >= states.back().time )
  {
    return states.back();
  }
  // Find the trajectory point immediately after the query_time
  auto upper_it = std::lower_bound( states.begin(), states.end(), query_time,
                                    []( const VehicleStateDynamic& state, double t ) { return state.time < t; } );

  auto lower_it = std::prev( upper_it );

  // Time difference between the two trajectory states
  double t0 = lower_it->time;
  double t1 = upper_it->time;

  // Interpolation factor
  double alpha = ( query_time - t0 ) / ( t1 - t0 );

  return interpolate_states_linear( *lower_it, *upper_it, alpha );
}
} // namespace dynamics
} // namespace adore
