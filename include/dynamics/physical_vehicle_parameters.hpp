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

#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace adore
{
namespace dynamics
{

struct PhysicalVehicleParameters
{
  double cog_to_front_axle           = 1.014;
  double rear_axle_to_cog            = 1.676;
  double wheelbase                   = 2.69;
  double front_axle_to_front_border  = 0.97;
  double rear_border_to_rear_axle    = 1.12;
  double mass                        = 1800;
  double friction_coefficient        = 0.8;
  double cog_height                  = 0.5;
  double front_tire_stiffness        = 10.8;
  double rear_tire_stiffness         = 17.8;
  double rotational_inertia_div_mass = 1.57;
  double front_track_width           = 1.7;
  double rear_track_width            = 1.7;
  double steering_ratio              = 1.0;
  double steering_angle_max          = 0.7;
  double steering_angle_min          = -0.7;
  double cornering_stiffness         = 63000.0;
  double brake_balance_front         = 0.6;
  double acceleration_balance_front  = 0.4;

  double body_width  = 1.82;
  double body_length = 4.78;
  double body_height = 1.5;

  // Constructor that loads parameters from a JSON file.
  PhysicalVehicleParameters( const std::string& file_path )
  {
    std::ifstream ifs( file_path );
    if( !ifs.is_open() )
    {
      throw std::runtime_error( "Could not open file: " + file_path );
    }
    nlohmann::json j;
    ifs >> j;

    cog_to_front_axle           = j.at( "cog_to_front_axle" ).get<double>();
    rear_axle_to_cog            = j.at( "rear_axle_to_cog" ).get<double>();
    wheelbase                   = j.at( "wheelbase" ).get<double>();
    front_axle_to_front_border  = j.at( "front_axle_to_front_border" ).get<double>();
    rear_border_to_rear_axle    = j.at( "rear_border_to_rear_axle" ).get<double>();
    mass                        = j.at( "mass" ).get<double>();
    friction_coefficient        = j.at( "friction_coefficient" ).get<double>();
    cog_height                  = j.at( "cog_height" ).get<double>();
    front_tire_stiffness        = j.at( "front_tire_stiffness" ).get<double>();
    rear_tire_stiffness         = j.at( "rear_tire_stiffness" ).get<double>();
    rotational_inertia_div_mass = j.at( "rotational_inertia_div_mass" ).get<double>();
    front_track_width           = j.at( "front_track_width" ).get<double>();
    rear_track_width            = j.at( "rear_track_width" ).get<double>();
    body_width                  = j.at( "body_width" ).get<double>();
    steering_ratio              = j.at( "steering_ratio" ).get<double>();
    steering_angle_max          = j.at( "steering_angle_max" ).get<double>();
    steering_angle_min          = j.at( "steering_angle_min" ).get<double>();
    cornering_stiffness         = j.at( "cornering_stiffness" ).get<double>();
    brake_balance_front         = j.at( "brake_balance_front" ).get<double>();
    acceleration_balance_front  = j.at( "acceleration_balance_front" ).get<double>();
    body_length                 = front_axle_to_front_border + rear_border_to_rear_axle + wheelbase;
    std::cerr << "LOADED VEHICLE PARAMETERS " << file_path << std::endl;
  }

  PhysicalVehicleParameters() {}; // default values
};

} // namespace dynamics
} // namespace adore
