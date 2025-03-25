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
#include "adore_math/angles.h"

#include "dynamics/integration.hpp"
#include "dynamics/vehicle_command.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

namespace adore
{
namespace dynamics
{


enum GearState
{
  NEUTRAL,
  DRIVING,
  REVERSE,
  PARKING
};

struct VehicleInfo
{
  GearState gear_state;
  double    wheel_speed;
  bool      left_indicator_on;
  bool      right_indicator_on;
  bool      automatic_steering_on;
  bool      automatic_acceleration_on;
  bool      automatic_acceleration_active;
  bool      clearance;
};

struct VehicleStateSimple
{
  VehicleStateSimple() {}

  VehicleStateSimple( double x, double y, double yaw_angle, double vx ) :
    x( x ),
    y( y ),
    yaw_angle( yaw_angle ),
    vx( vx )
  {}

  double x         = 0.0;
  double y         = 0.0;
  double yaw_angle = 0.0;
  double vx        = 0.0;

  // Friend declaration for operator<<
  friend std::ostream& operator<<( std::ostream& os, const VehicleStateSimple& state );
};

// VehicleStateDynamic Struct
struct VehicleStateDynamic
{
  VehicleStateDynamic() {}

  VehicleStateDynamic( double x, double y, double z, double vx, double vy, double yaw_angle, double yaw_rate, double ax, double ay ) :
    x( x ),
    y( y ),
    z( z ),
    vx( vx ),
    vy( vy ),
    yaw_angle( yaw_angle ),
    yaw_rate( yaw_rate ),
    ax( ax ),
    ay( ay )
  {}

  // Member function to convert to VehicleStateSimple
  VehicleStateSimple to_vehicle_state_simple() const;

  // Static member function to create from VehicleStateSimple
  static VehicleStateDynamic from_vehicle_state_simple( const VehicleStateSimple& simple_state );

  double x              = 0.0;
  double y              = 0.0;
  double z              = 0.0;
  double vx             = 0.0;
  double vy             = 0.0;
  double yaw_angle      = 0.0;
  double yaw_rate       = 0.0;
  double steering_angle = 0.0;
  double steering_rate  = 0.0;
  double ax             = 0.0;
  double ay             = 0.0;
  double time           = 0.0;

  // Friend declaration for operator<<
  friend std::ostream& operator<<( std::ostream& os, const VehicleStateDynamic& state );
};

VehicleStateDynamic interpolate_states_linear( const VehicleStateDynamic& state1, const VehicleStateDynamic& state2, double alpha );

// Overload operator+ to add two VehicleStateDynamic objects.
static inline VehicleStateDynamic
operator+( const VehicleStateDynamic& a, const VehicleStateDynamic& b )
{
  VehicleStateDynamic result;
  result.x              = a.x + b.x;
  result.y              = a.y + b.y;
  result.z              = a.z + b.z;
  result.vx             = a.vx + b.vx;
  result.vy             = a.vy + b.vy;
  result.yaw_angle      = a.yaw_angle + b.yaw_angle;
  result.yaw_rate       = a.yaw_rate + b.yaw_rate;
  result.steering_angle = a.steering_angle + b.steering_angle;
  result.steering_rate  = a.steering_rate + b.steering_rate;
  result.ax             = a.ax + b.ax;
  result.ay             = a.ay + b.ay;
  result.time           = a.time + b.time;
  return result;
}

// Overload operator* for scalar multiplication: scalar * state.
static inline VehicleStateDynamic
operator*( double scalar, const VehicleStateDynamic& state )
{
  VehicleStateDynamic result;
  result.x              = scalar * state.x;
  result.y              = scalar * state.y;
  result.z              = scalar * state.z;
  result.vx             = scalar * state.vx;
  result.vy             = scalar * state.vy;
  result.yaw_angle      = scalar * state.yaw_angle;
  result.yaw_rate       = scalar * state.yaw_rate;
  result.steering_angle = scalar * state.steering_angle;
  result.steering_rate  = scalar * state.steering_rate;
  result.ax             = scalar * state.ax;
  result.ay             = scalar * state.ay;
  result.time           = scalar * state.time;
  return result;
}

// Overload operator* for state * scalar.
static inline VehicleStateDynamic
operator*( const VehicleStateDynamic& state, double scalar )
{
  return scalar * state;
}
} // namespace dynamics
} // namespace adore
