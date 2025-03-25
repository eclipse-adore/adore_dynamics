
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
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace dynamics
{


// Member function to convert to VehicleStateSimple
VehicleStateSimple
VehicleStateDynamic::to_vehicle_state_simple() const
{
  return VehicleStateSimple( x, y, yaw_angle, vx );
}

// Static member function to create from VehicleStateSimple
VehicleStateDynamic
VehicleStateDynamic::from_vehicle_state_simple( const VehicleStateSimple& simple_state )
{
  return VehicleStateDynamic( simple_state.x, simple_state.y,
                              0.0, // z
                              simple_state.vx,
                              0.0, // vy
                              simple_state.yaw_angle,
                              0.0, // yaw_rate
                              0.0, // ax
                              0.0  // ay
  );
}

// Definition of operator<< for VehicleStateDynamic
std::ostream&
operator<<( std::ostream& os, const VehicleStateDynamic& state )
{
  os << "("
     << "x: " << state.x << ", y: " << state.y << ", z: " << state.z << ", vx: " << state.vx << ", vy: " << state.vy
     << ", yaw_angle: " << state.yaw_angle << ", yaw_rate: " << state.yaw_rate << ", steering_angle: " << state.steering_angle
     << ", steering_rate: " << state.steering_rate << ", ax: " << state.ax << ", ay: " << state.ay << ", time: " << state.time << ")";
  return os;
}

// Definition of operator<< for VehicleStateSimple
std::ostream&
operator<<( std::ostream& os, const VehicleStateSimple& state )
{
  os << "("
     << "x: " << state.x << ", y: " << state.y << ", yaw_angle: " << state.yaw_angle << ", vx: " << state.vx << ")";
  return os;
}

VehicleStateDynamic
interpolate_states_linear( const VehicleStateDynamic& state1, const VehicleStateDynamic& state2, double alpha )
{
  VehicleStateDynamic interpolated_state;
  // Ensure alpha is between 0 and 1
  alpha = std::clamp( alpha, 0.0, 1.0 );
  // Linear interpolation for state variables
  interpolated_state.x              = ( 1 - alpha ) * state1.x + alpha * state2.x;
  interpolated_state.y              = ( 1 - alpha ) * state1.y + alpha * state2.y;
  interpolated_state.z              = ( 1 - alpha ) * state1.z + alpha * state2.z;
  interpolated_state.vx             = ( 1 - alpha ) * state1.vx + alpha * state2.vx;
  interpolated_state.vy             = ( 1 - alpha ) * state1.vy + alpha * state2.vy;
  interpolated_state.yaw_rate       = ( 1 - alpha ) * state1.yaw_rate + alpha * state2.yaw_rate;
  interpolated_state.ax             = ( 1 - alpha ) * state1.ax + alpha * state2.ax;
  interpolated_state.ay             = ( 1 - alpha ) * state1.ay + alpha * state2.ay;
  interpolated_state.steering_angle = ( 1 - alpha ) * state1.steering_angle + alpha * state2.steering_angle;
  interpolated_state.steering_rate  = ( 1 - alpha ) * state1.steering_rate + alpha * state2.steering_rate;

  // Interpolate yaw angle
  double delta_yaw             = adore::math::normalize_angle( state2.yaw_angle - state1.yaw_angle );
  interpolated_state.yaw_angle = adore::math::normalize_angle( state1.yaw_angle + alpha * delta_yaw );

  // Interpolated time
  interpolated_state.time = ( 1 - alpha ) * state1.time + alpha * state2.time;

  return interpolated_state;
}


} // namespace dynamics
} // namespace adore
