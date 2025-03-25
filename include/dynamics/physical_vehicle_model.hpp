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

  PhysicalVehicleModel() {};
};

} // namespace dynamics
} // namespace adore
