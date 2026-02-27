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

#include <functional>
#include <memory>
#include <string>

#include "dynamics/motion_models.hpp"
#include "dynamics/physical_vehicle_parameters.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace dynamics
{

struct PhysicalVehicleModel
{
  PhysicalVehicleParameters                                                               params;
  std::function<VehicleStateDynamic( const VehicleStateDynamic&, const VehicleCommand& )> motion_model;

  PhysicalVehicleModel( const std::string& file_location, bool dynamic = false )
  {
    params = PhysicalVehicleParameters( file_location );
    if( dynamic )
    {
      motion_model = [p = params]( const VehicleStateDynamic& state, const VehicleCommand& command ) {
        return dynamic_bicycle_model( state, p, command );
      };
    }
    else
    {
      motion_model = [p = params]( const VehicleStateDynamic& state, const VehicleCommand& command ) {
        return kinematic_bicycle_model( state, p, command );
      };
    }
  };

  PhysicalVehicleModel()
  {
    params       = PhysicalVehicleParameters();
    motion_model = [p = params]( const VehicleStateDynamic& state, const VehicleCommand& command ) {
      return kinematic_bicycle_model( state, p, command );
    };
  };

  PhysicalVehicleModel( const PhysicalVehicleParameters& vehicle_parameters )
  {
    params       = vehicle_parameters;
    motion_model = [p = params]( const VehicleStateDynamic& state, const VehicleCommand& command ) {
      return kinematic_bicycle_model( state, p, command );
    };
  };
};

} // namespace dynamics
} // namespace adore
