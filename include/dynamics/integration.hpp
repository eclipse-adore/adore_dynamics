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

namespace adore
{
namespace dynamics
{

template<typename State, typename Control, typename MotionModel>
inline State
integrate_rk4( const State& current_state, const Control& control, double dt, const MotionModel& motion_model )
{
  State k1 = motion_model( current_state, control );
  State k2 = motion_model( current_state + 0.5 * dt * k1, control );
  State k3 = motion_model( current_state + 0.5 * dt * k2, control );
  State k4 = motion_model( current_state + dt * k3, control );

  return current_state + ( dt / 6.0 ) * ( k1 + 2 * k2 + 2 * k3 + k4 );
}

template<typename State, typename Control, typename MotionModel>
inline State
integrate_euler( const State& current_state, const Control& control, double dt, const MotionModel& motion_model )
{
  return current_state + dt * motion_model( current_state, control );
}

template<typename State, typename Control, typename MotionModel>
void
integrate_up_to_time( State& state, const Control& control, double time, const MotionModel& motion_model )
{
  double dt = time - state.time;
  state     = integrate_rk4( state, control, dt, motion_model );
}


} // namespace dynamics
} // namespace adore
