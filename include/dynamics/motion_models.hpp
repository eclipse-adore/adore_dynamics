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

#include <cmath>

#include <algorithm> // for std::clamp

#include "dynamics/physical_vehicle_parameters.hpp"
#include "dynamics/vehicle_command.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace dynamics
{


static inline VehicleStateDynamic
dynamic_bicycle_model( const VehicleStateDynamic& state, const PhysicalVehicleParameters& params, const VehicleCommand& cmd )
{
  VehicleStateDynamic ds; // derivative of state

  // Unpack state.
  double psi   = state.yaw_angle;
  double vx    = state.vx;
  double vy    = state.vy;
  double omega = state.yaw_rate;

  // Use the commanded acceleration and steering angle.
  double a_cmd = cmd.acceleration;
  double delta = cmd.steering_angle;

  // Avoid division by zero
  constexpr double epsilon = 1e-6;

  // Prevent reversing: If velocity is near zero and acceleration is negative, stop the vehicle
  if( vx < epsilon && a_cmd < 0 )
  {
    ds.x              = 0.0;
    ds.y              = 0.0;
    ds.z              = 0.0;
    ds.vx             = 0.0; // No reverse movement
    ds.vy             = 0.0;
    ds.yaw_angle      = 0.0;
    ds.yaw_rate       = 0.0;
    ds.steering_angle = 0.0;
    ds.steering_rate  = 0.0;
    ds.ax             = 0.0;
    ds.ay             = 0.0;
    ds.time           = 1.0;
    return ds;
  }

  // Effective lateral velocity
  double lambda = 0.0;
  double vyr    = vy - lambda * omega;

  constexpr double gravity = 9.81;

  // Compute tire force contributions
  double fyf_fzf = -params.friction_coefficient * ( params.rear_axle_to_cog / params.wheelbase ) * gravity * params.front_tire_stiffness
                 * ( ( ( vyr ) + params.wheelbase * omega ) / vx - delta );
  double fyr_fzr = -params.friction_coefficient * ( params.cog_to_front_axle / params.wheelbase ) * gravity * params.rear_tire_stiffness
                 * ( vyr / vx );

  double ay_dynamic = fyf_fzf + fyr_fzr;
  double domega = ( 1.0 / params.rotational_inertia_div_mass ) * ( params.cog_to_front_axle * fyf_fzf - params.rear_axle_to_cog * fyr_fzr );

  // Compute state derivatives
  ds.x              = std::cos( psi ) * vx - std::sin( psi ) * vy;
  ds.y              = 0.0;
  ds.z              = 0.0;
  ds.vx             = a_cmd;
  ds.vy             = ay_dynamic;
  ds.yaw_angle      = omega;
  ds.yaw_rate       = domega;
  ds.steering_angle = state.steering_angle;
  ds.steering_rate  = 0.0;
  ds.ax             = 0.0;
  ds.ay             = 0.0;
  ds.time           = 1.0;
  return ds;
}

static inline VehicleStateDynamic
kinematic_bicycle_model( const VehicleStateDynamic& state, const PhysicalVehicleParameters& params, const VehicleCommand& cmd )
{
  VehicleStateDynamic ds;

  // Unpack state
  double psi = state.yaw_angle;
  double vx  = state.vx;

  // Use commanded acceleration
  double           a_cmd   = cmd.acceleration;
  double           delta   = cmd.steering_angle;
  constexpr double epsilon = 1e-6;

  // Prevent reversing: If velocity is near zero and acceleration is negative, stop the vehicle
  if( vx < epsilon && a_cmd < 0 )
  {
    ds.x              = 0.0;
    ds.y              = 0.0;
    ds.z              = 0.0;
    ds.vx             = 0.0;
    ds.vy             = 0.0;
    ds.yaw_angle      = 0.0;
    ds.yaw_rate       = 0.0;
    ds.steering_angle = 0.0;
    ds.steering_rate  = 0.0;
    ds.ax             = 0.0;
    ds.ay             = 0.0;
    ds.time           = 1.0;
    return ds;
  }

  // Compute kinematic updates
  ds.x         = std::cos( psi ) * vx;
  ds.y         = std::sin( psi ) * vx;
  ds.z         = 0.0;
  ds.vx        = a_cmd;
  ds.vy        = 0.0;
  ds.yaw_angle = vx * std::tan( delta ) / params.wheelbase;
  ds.yaw_rate  = 0.0;

  // Simple steering dynamics
  ds.steering_angle = state.steering_rate;
  ds.steering_rate  = 0.0;
  ds.ax             = 0.0;
  ds.ay             = 0.0;
  ds.time           = 1.0;
  return ds;
}

} // namespace dynamics
} // namespace adore