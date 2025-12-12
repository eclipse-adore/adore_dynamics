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

#include <gtest/gtest.h>

#include "dynamics/integration.hpp"

namespace
{

struct TestState
{
  double x{ 0.0 };
  double time{ 0.0 };
};

// State + State
inline TestState
operator+( const TestState& a, const TestState& b )
{
  TestState r;
  r.x    = a.x + b.x;
  r.time = a.time + b.time;
  return r;
}

// scalar * State (this is what integration.hpp uses)
inline TestState
operator*( double scalar, const TestState& s )
{
  TestState r;
  r.x    = scalar * s.x;
  r.time = scalar * s.time;
  return r;
}

// Simple constant-velocity "motion model"
// Control is velocity v; derivative is dx/dt = v, d(time)/dt = 1.
struct ConstantVelocityModel
{
  TestState
  operator()( const TestState& /*state*/, double v ) const
  {
    TestState ds;
    ds.x    = v;
    ds.time = 1.0;
    return ds;
  }
};

} // namespace

TEST( Integration, EulerConstantVelocity )
{
  TestState             s0{};
  ConstantVelocityModel model;
  const double          v  = 3.0;
  const double          dt = 0.5;

  TestState s1 = adore::dynamics::integrate_euler( s0, v, dt, model );

  EXPECT_DOUBLE_EQ( s1.x, v * dt );
  EXPECT_DOUBLE_EQ( s1.time, dt );
}

TEST( Integration, Rk4ConstantVelocity )
{
  TestState             s0{};
  ConstantVelocityModel model;
  const double          v  = 2.0;
  const double          dt = 0.25;

  TestState s1 = adore::dynamics::integrate_rk4( s0, v, dt, model );

  // For a linear system with constant derivative, RK4 is exact
  EXPECT_DOUBLE_EQ( s1.x, v * dt );
  EXPECT_DOUBLE_EQ( s1.time, dt );
}

TEST( Integration, IntegrateUpToTime )
{
  TestState s{};
  s.time = 0.0;

  ConstantVelocityModel model;
  const double          v        = 1.2;
  const double          t_target = 2.0;

  adore::dynamics::integrate_up_to_time( s, v, t_target, model );

  EXPECT_NEAR( s.x, v * t_target, 1e-12 );
  EXPECT_NEAR( s.time, t_target, 1e-12 );
}
