/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Sanath Himasekhar Konthala
 *    Giovanni Lucente
 *    Marko Mizdrak
 ********************************************************************************/
#include "dynamics/traffic_participant.hpp"

namespace adore
{
namespace dynamics
{
math::Polygon2d
TrafficParticipant::get_corners() const
{
  // Compute half dimensions.
  double half_length = physical_parameters.body_length / 2.0;
  double half_width  = physical_parameters.body_width / 2.0;

  // Define the four corners in local coordinates.
  // The order here is: rear-right, rear-left, front-left, front-right.
  math::Polygon2d corners;
  corners.points = { math::Point2d( -half_length, -half_width ), math::Point2d( -half_length, half_width ),
                     math::Point2d( half_length, half_width ), math::Point2d( half_length, -half_width ) };

  double cos_yaw = std::cos( state.yaw_angle );
  double sin_yaw = std::sin( state.yaw_angle );

  // Convert local corners to global coordinates using the vehicle's pose.
  for( auto& corner : corners.points )
  {
    corner.x = state.x + corner.x * cos_yaw - corner.y * sin_yaw;
    corner.y = state.y + corner.x * sin_yaw + corner.y * cos_yaw;
  }

  return corners;
}

void
TrafficParticipantSet::update_traffic_participants( const TrafficParticipant& new_participant_data )
{
  // check if participant is within the validity area
  if( validity_area && !validity_area->point_inside( new_participant_data.state ) )
  {
    return;
  }


  if( participants.count( new_participant_data.id ) == 0 )
  {
    participants[new_participant_data.id] = new_participant_data;
    return;
  }

  participants[new_participant_data.id].state               = new_participant_data.state;
  participants[new_participant_data.id].physical_parameters = new_participant_data.physical_parameters;

  if( new_participant_data.goal_point.has_value() )
  {
    participants[new_participant_data.id].goal_point = new_participant_data.goal_point.value();
  }
  if( new_participant_data.trajectory.has_value() )
  {
    participants[new_participant_data.id].trajectory = new_participant_data.trajectory.value();
  }
  if( new_participant_data.route.has_value() )
  {
    participants[new_participant_data.id].route = new_participant_data.route.value();
  }
};

void
TrafficParticipantSet::remove_old_participants( double max_age, double current_time )
{
  for( auto it = participants.begin(); it != participants.end(); )
  {
    if( current_time - it->second.state.time > max_age )
    {
      it = participants.erase( it );
    }
    else
    {
      ++it;
    }
  }
};

} // namespace dynamics


} // namespace adore
